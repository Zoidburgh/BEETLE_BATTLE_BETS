# Smoothness Plan — combat spikes, render stepping, network jitter
2026-07-16. Full audit of "jittering / hit them and it skips up / tilts super fast / make networking feel better".

**STATUS 2026-07-16: S1+S2+S3+S4 ALL IMPLEMENTED (uncommitted, awaiting user testing).**
Deviations from plan: edge tipping NOT routed through pending (already dt-scaled continuous;
a pending tail would feel floaty falling off edges); AIRBORNE_TILT_SPEED re-defaulted 900→14
instead of adding a second cap param (slider 4–200 restores old feel); camera still samples raw
physics positions (0.1 lerp masks it, fall-state branches not worth the churn); own-beetle
correction stays per-packet (the S4.1 visual offset hides the bursts; per-frame spreading
deemed not worth the restructure risk). S4.4 (interp buffer) not built — evaluate after testing.
Protocol bumped v5→v6 (both builds must update to match).

## MULTIPLAYER TEST CHECKLIST (user couldn't test multi on 2026-07-16 — run when possible)
Setup: 2-PC, or one PC with `--simlag 80 --simloss 5`. BOTH sides need the new build (v6
refuses v5). N-key opens the net HUD; snap_count still counts SIM-level snaps — expected;
the point is you should no longer SEE them.
1. Horn grind between two players: opponent motion should be visibly less twitchy
   (30Hz + angular-rate extrapolation), no skip-up pops (S1 physics).
2. Big impulse hit on YOUR beetle: should read as a quick ~4-frame glide, not a teleport
   (visual error-offset). Same for the opponent after a hard divergence.
3. Ball match: guest-side ball should roll smoothly (was corrected only per-packet);
   spin/stripe should match the host's rolling.
4. Own-beetle feel while walking uncontested: the old constant "drag" should be gone
   (velocity blend now inside the deadzone).
5. Opponent mid-air flips: should rotate continuously, not ratchet toward stale angles.
6. Sustained packet loss (`--simloss 15`): opponent should coast-then-correct with glides;
   watch for anything that still hard-jumps.
If jitter persists after all this → S4.4 (snapshot interpolation buffer) is the next lever.
SINGLE-PLAYER regression checks that need no second PC: rest-buzz watch (beetle standing
still ~10s, no vertical vibration — fractional Y is new), tilt glide on flips, Y glide on
lifts/climbs, yaw-grind feel (now smoothed via pending).

## The complaint → cause map (audit results)

Four audits (vertical impulses, tilt/rotation, render pipeline, network sync) traced each symptom
to specific code. Line numbers verified 2026-07-16 on branch `arena_mods`.

### "It skips up" — single-step vertical injections
| # | Source | Where | Why it spikes |
|---|--------|-------|---------------|
| V1 | **Shaft-vs-shaft vertical separation** | 16465-16476 | Direct **Y position teleport** (`b.y += _p1y`), up to ~0.45 vox/step at depth, repeated every step while shafts cross. Down-push is suppressed when grounded (16467-16470), so grounded beetles only ever teleport **UP**. No velocity, no smoothing, no cap. The single most teleport-like source in combat. |
| V2 | **Yaw-grind lift** | 20586, 20613 | Direct `vy += YAW_GRIND_LIFT(80)/60 ≈ 1.33/step`, every step a yaw key is held in contact, applied to EVERY contacting beetle, zero decay, no cap below the global 20. Also teleports `pitch -= YAW_GRIND_TILT(0.03)` per step. |
| V3 | **Shaft-penetration lift cap** | 16257-16274 | `pending_lift += min(..., 4.0)` — per-application cap is **3× the horn-lift cap** (12/LIFT_STEP_DIV = 1.33 at 16673-16681). Oversized queued lift when a body rides a low shaft. |
| V4 | **Body impulse_y w/ biased normal** | 16570-16571, 17295, 17311-17314 | Contact normal gets `normal_y += horn_leverage * 2.5` upward bias; body-to-body contacts then inject `impulse_y` directly into vy in one step (horn contacts are gated off at 17292). Only cap is global MAX_VERTICAL_VELOCITY=20. |
| V5 | **pending_* tail-snap** | 2129-2130, 2135-2136, 2141-2142 | The 20%/step drain dumps the ENTIRE remainder in one frame once drain < 0.05 (lift) / 0.02 (pitch/roll) — a small pop at the end of every queued impulse. |
| V6 | **Integer render Y** | 8119, 23906 | Beetle render Y is floored to a whole voxel and `owner_frac_offset[slot].y` is hardcoded 0.0 — every lift/climb pops in **1-full-voxel (~0.4u) jumps**. (The ball DOES get fractional Y at 9136.) Deliberate: hides the floor-settle limit-cycle buzz. |

### "Tilts super fast" — pitch/roll effectively uncapped + instant torque injections
| # | Source | Where | Why it spikes |
|---|--------|-------|---------------|
| T1 | **`AIRBORNE_TILT_SPEED = 900`** | 18664 (blame: Nov 2025 "more tumbling" era) | The airborne pitch/roll velocity cap is 900 rad/s = **~859° per physics step** — i.e. no cap. Every hit that lifts a beetle runs under this branch. Grounded cap is hardcoded 8.0 (2340). Yaw, by contrast, is hard-clamped at MAX_ANGULAR_SPEED=8.0 (17448-17451) precisely "to prevent jarring 180° spins" — pitch/roll never got the same protection. |
| T2 | **`GROUND_TILT_ANGLE = 300`°** | 18665 (Nov 2025) | Grounded angle clamp at 300° = no clamp. |
| T3 | **Instant (non-pending) torque injections** | body tilt 17091-17096 (up to ~2.9 rad/s per step, re-fires every overlap step); edge tipping 23168-23169; spray flip 3562-3593 (~21 rad/s one shot); comet 14013-14020 (up to ~540 rad/s one shot) | These write straight to `pitch_velocity`/`roll_velocity`, bypassing the 20%/step pending drain that smooths the horn lift/tip/shaft torques. With T1 disabled, nothing restrains the result. |
| T4 | **Raw pitch/roll rasterization** | 23919; contrast yaw at 23911-23916 | Yaw is quantized to 0.05 rad and the residual glides sub-voxel in the extract; pitch/roll are stamped RAW into the voxel lattice every frame — tilt shimmers/pops at voxel granularity, making fast tilts look even harsher. renderer.py already supports 3-axis rot residual (ball uses it, 9142-9147). |

NOTE ON INTENT: TUMBLE_MULTIPLIER 6.2, HORN_LIFT_STRENGTH 1.36, YAW_GRIND_PUSH/LIFT 75/80 are the
user's 2026-07-12 slider tune — the *magnitudes* are wanted. The problem is HOW they land
(single-step, uncapped-rate, position teleports), not how big they are. The plan adds rate caps and
smoothing WITHOUT retuning the 07-12 values; flips still happen, they just take a few frames to
develop so the eye can track them.

### General jitter — render pipeline findings (mostly healthy)
- Frame loop is a correct fixed-timestep (1/60) accumulator with render interpolation (19323-20155,
  alpha at 23259). At 30-50 FPS the game runs true-speed. Below ~15 FPS the
  MAX_PHYSICS_STEPS_PER_FRAME=4 cap breaks WITHOUT draining the accumulator (20157-20159) → alpha
  can exceed 1.0 → one-frame extrapolation overshoot. Cheap fix: clamp alpha to 1.0.
- Camera targets RAW physics positions (19645-19646), not interpolated render_*; masked by the 0.1
  lerp but a free consistency win.
- Per-voxel animation (legs/horn/tail) steps voxel-to-voxel by design — out of scope.
- Floor-settle buzz: gravity vs floor-clamp vs restoring torque limit-cycles y by a sub-voxel
  fraction every tick at rest; currently hidden by integer render Y (V6's reason to exist).

### Networking — the guest experience
| # | Weakness | Where |
|---|----------|-------|
| N1 | **No interpolation buffer.** Guest keeps ONE latest target per opponent and forward-extrapolates by wall-clock age (`time.time()`, 20008-20011) with per-frame 0.15 lerp toward it — packet arrival jitter maps DIRECTLY to visible motion jitter. Docs intended interpolation; code extrapolates. | 19982-20062 |
| N2 | **Big host impulses arrive as snaps or rubber-bands.** err > SYNC_SNAP_DIST(6.0) → hard teleport (own beetle too: 19919-19930; opponents 20024-20031). Every physics fix in S1/S2 reduces these directly: predictable velocity-based responses extrapolate well; position teleports never do. | |
| N3 | **Own-beetle velocity blended 30% EVERY packet with no deadzone** (19937-19939) — constant felt "drag" even when position is inside the 0.75 deadzone. Position correction is per-packet bursts (~20 Hz), not continuous. | |
| N4 | **Ball corrected only per-packet** at lerp 0.45 + hard velocity adoption, no per-frame smoothing — steppiest object on screen. No ball spin synced. | 19955-19980 |
| N5 | **Angular velocity not synced** (no yaw/pitch/roll rates in the 37B/beetle state) → rotation can't be extrapolated, only lerped at 0.2 toward stale absolutes. | network.py 717-748 |
| N6 | Extrapolation cap 0.25s → sustained loss = coast → freeze → jump. Sync is 20 Hz unreliable; guest spends 2 of 3 frames extrapolating. | 20008 |
| N7 | Input frame numbers effectively ignored (host applies `last_known` bits at its own frame; jitter-buffer scaffolding unused). Fine for feel — listed for completeness, not planned work. | network.py 1641-1659 |

---

## The plan

Ordering rationale: **S1/S2 (physics spike removal) come first because they are ALSO networking
fixes** — teleports and one-step impulses are exactly what the guest cannot predict, so every one
removed prevents a correction snap. S3 is independent render polish. S4 is the dedicated net work.
Physics changes run identically on host and guest (same code) — no protocol impact until S4.3.

### Phase S1 — kill single-step vertical injections
1. **V1 SVS vertical → velocity, not position.** Replace `b.y += _p1y` with a capped `vy`
   contribution (or route through `pending_lift`), keeping the grounded up-only rule and the
   `SVS_VERTICAL_SCALE` slider semantics (0 still = horizontal-only). Add a per-step rate cap
   slider (`SVS_LIFT_RATE`, start ~1.5/step equivalent).
2. **V2 yaw-grind lift → pending_lift.** Route the 20586/20613 direct `vy +=` through
   `pending_lift` so it rides the 20%/step drain like all other horn lift. Same for the per-step
   `pitch -=` teleport → `pending_pitch`. Keep YAW_GRIND_* slider values unchanged.
3. **V3 align shaft-penetration lift cap** 4.0 → 12/LIFT_STEP_DIV (≈1.33) to match horn lift; make
   it a param (`SHAFT_PEN_LIFT_CAP`).
4. **V4 cap body impulse_y** — either remove the `horn_leverage*2.5` up-bias from the *body-contact*
   impulse path (bias exists to help horn lifts, which are gated off here anyway), or clamp the
   per-step vertical component (~1.5). Prefer removing the bias from the body path: simpler, honest.
5. **V5 fix tail-snap:** drain `min(pending, max(pending*0.2, 0.05))` — constant minimum drain to
   zero instead of dumping the remainder.

Gate: user in-game feel test (stag+hercules grind, yaw-grind hold, shaft rides). Canary 3x median
to confirm no perf/clip regression (deep_clip_events comparable; horn_cross rebaselines, ignore).

### Phase S2 — tilt rate governance (pitch/roll get what yaw already has)
1. **T1 real airborne tilt-speed cap.** New param + slider `MAX_TILT_SPEED_AIR` (start ~12-15
   rad/s; grounded stays 8.0, also promote the hardcoded 2340 value to a param). Leave
   `AIRBORNE_TILT_SPEED=900` semantics behind — the new cap is a separate param so the Nov-2025
   "dramatic tumbling" value stops silently disabling protection. A capped 15 rad/s still flips a
   beetle in ~12 frames — dramatic but trackable.
2. **T2 grounded angle clamp** — leave at 300 for now (beetles rolling over on the ground may be
   wanted for tipping gameplay); revisit only if the user still sees ground flips after T1.
3. **T3 route instant torques through pending:** body tilt (17091-17096) and edge tipping
   (23168-23169) → `pending_pitch/roll`. Spray flip and comet stay direct (intentional one-shot
   hazard drama) — the new T1 velocity cap now bounds their worst case anyway (comet's ~540 rad/s
   becomes 15 sustained over frames = still a violent launch, just watchable).
4. Retune pass with the user: with caps live, TUMBLE_MULTIPLIER 6.2 may read differently — sliders
   already exist; user drives.

Design-rule check: all of S1/S2 is response-shaping (rate caps + smoothing), no per-type forces —
consistent with the "describe shape/motion only" rule.

### Phase S3 — render smoothness (independent, do in any order)
1. **T4 pitch/roll residual glide.** Quantize render pitch/roll to 0.05 rad like yaw (23911) and
   pass the residuals through `owner_rot_residual` — renderer.py already applies 3-axis residuals
   for the ball (1473-1492); beetles just never fed it. Biggest pure-visual win for tilt readability.
2. **V6 fractional beetle render Y** — two sub-steps, in order:
   a. Kill the settle buzz at the SOURCE: when grounded, not in combat contact, and |vy| below a
      small epsilon for N steps, freeze y to the floor-rest height (a settle latch, cleared on any
      impulse/contact/input). The buzz is a physics limit cycle; latching it is honest at rest.
   b. Then populate `owner_frac_offset[slot].y` from the render fraction like the ball (9136), so
      lifts/climbs glide instead of popping a voxel at a time.
   Ship (a) and (b) together behind one toggle — if (a) misses a buzz path, (b) re-exposes it
   (this is exactly the failure that made Y integer in the first place; see collision-notes
   "anti-rectifier" lesson before touching settle logic).
3. **alpha clamp** to 1.0 at 23259 (cap-break overshoot below ~15 FPS).
4. **Camera targets interpolated positions** (render_x/y/z at 19645-19646). Minor.

Gate: user visual check — at-rest stillness (buzz), slow shaft ride (Y glide), fast flip (tilt glide).

### Phase S4 — networking feel
1. **S4.1 Visual error-offset smoothing (do this FIRST — small, no protocol change).**
   Whenever ANY correction (lerp burst or snap) moves a beetle or the ball, accumulate the applied
   position delta into a per-slot `visual_offset`; render at `pos - visual_offset`; decay the offset
   ~12-15%/frame (slider). Sim state stays exactly as today (corrections still land instantly for
   physics), but the EYE sees a short glide instead of a snap. Covers own beetle, opponents, ball,
   and death snaps in one mechanism. Cap offset magnitude (~6u, the snap distance) and zero it on
   respawn/teleport events. Rotation gets the same treatment via a `visual_rot_offset` on
   yaw/pitch/roll.
2. **S4.2 Correction cadence fixes (no protocol change):**
   - Deadzone the own-beetle velocity blend (skip the 0.3 blend when position err < deadzone) — N3.
   - Spread own-beetle position correction over frames (reuse the opponent per-frame path with own
     tunables) instead of one burst per packet.
   - Per-frame smoothing for the ball between packets (dead-reckon from synced velocity like
     opponents get) — N4.
   - `recv_time` → `time.perf_counter()` (monotonic) — N1's secondary jitter injector.
3. **S4.3 Protocol v6 — sync angular velocity + ball spin, raise rate.**
   Add per-beetle yaw/pitch/roll rates (3 float32 or quantized int16) and ball
   angular/pitch/roll velocity to `send_state_sync`; bump PROTOCOL_VERSION (project rule: mismatched
   builds must refuse to match). Raise sync to 30 Hz (`physics_frame % 2`) — packet is ~180-250B,
   bandwidth is trivial (~7 KB/s), and correction bursts halve. Rotation extrapolation then uses
   real rates instead of 0.2-lerping stale absolutes — this is what makes flips look smooth on the
   guest's screen.
4. **S4.4 (evaluate after S4.1-4.3, likely unnecessary):** a true snapshot-interpolation buffer
   (render remotes 2-3 snapshots in the past, interpolate between brackets, extrapolate only when
   dry). Classic and correct, but it interacts with guest-side collision prediction (remotes would
   render ~100-150ms behind where the guest's sim collides with them). Only reach for it if
   jitter is still visible after the offset-smoothing + 30 Hz + angular-rate work.

Gate per rung: 2-PC (or `--simlag 80 --simloss 5`) with deliberate horn grinds and deaths; N-key
net HUD; user judges feel. S1/S2 should already be merged first — measure S4 against the improved
baseline.

## What we're NOT doing (and why)
- No rollback/re-simulation networking — massive complexity, the offset-smoothing + rate bump gets
  most of the perceived win for this game's pace.
- No retuning of the user's 2026-07-12 combat magnitudes — caps and smoothing change delivery, not
  strength; retune happens after, user-driven, via existing sliders.
- No per-type special-casing anywhere (design rule).
- No touching the input pipeline (N7) — zero-input-delay feel is a deliberate design win; stale-input
  repeat is imperceptible at this packet rate.
- Comet/spray stay violent by design; they just obey the new rate cap.

## Suggested execution order
1. S1 (vertical spikes) + S2 (tilt caps) — one session, feel-test together.
2. S3.1 + S3.3 + S3.4 (tilt glide, alpha clamp, camera) — quick.
3. S3.2 (fractional Y + settle latch) — its own careful session (buzz history).
4. S4.1 + S4.2 (offset smoothing + cadence) — no protocol bump needed.
5. S4.3 (protocol v6 angular rates + 30 Hz) — needs both builds updated.
6. Re-evaluate; S4.4 only if needed.
