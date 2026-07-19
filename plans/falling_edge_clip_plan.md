# Falling-Beetle Edge Clip Plan — lip fulcrum for death falls

**>>> ENTIRE ATTEMPT REVERTED 2026-07-19 (user decision: needs a
designed rework, not sequential patches). Code restored to 2b35a26;
the full attempt lives in plans/edge_death_attempt_v10.patch; the
distilled lessons + rework plan are in
plans/edge_death_retrospective.md — READ THAT FIRST. This file is the
raw blow-by-blow history (V1-V10) kept for reference. <<<**

2026-07-19. Status: F1 SHIPPED same day (uncommitted), awaiting user
edge-death testing. F0 SKIPPED per user (clipper known: mostly horn,
sometimes body/whole — all three points sampled from the start).
IMPLEMENTATION DELTAS from the design below:
- The clip happens mostly BEFORE is_falling: POINT_OF_NO_RETURN is
  y=-5, so the visible tumble (floor level -> -5) is a live beetle with
  no floor support. The fulcrum covers no-floor live beetles AND
  is_falling beetles, band -8 < y < 4, vy < 0 only.
- RE-LANDING CHECKPOINT RESOLVED: death event fires at DEPTH (y=-16
  explosion), no-return at -5. Recovery therefore uses the game's OWN
  existing rule: above -5 a horn-catch that keeps/returns you is a
  legitimate save (floor collision just resumes); below -5 nothing
  un-sets, death proceeds. No new game-rules surface.
- SIGN-SAFE response: no added torque (no wrong-direction risk) —
  vy damp 0.45 + pitch/roll friction 0.85 + yaw 0.92 + voidward slide
  0.15/substep around the caught point. The pivot look emerges from
  the body sliding/falling around the pinned tip.
- Samples: horn primary tip + spine front/rear (3 kernel calls per
  unsupported beetle per substep in the band only).
- Telemetry: lip_fulcrum_substeps in collision_stats + perf log.

Problem: a beetle knocked off an edge tips
(rear first — correct), enters `is_falling`, and from that moment is in
ZERO collision systems — so as it rotates, the front horn sweeps through
the lip voxels (user report: rhino butt knocked off, horn clips the
arena). Companion pattern: the ball's pit-edge deflection (commit
516c157) fixed the same bug class for the ball.

## HARD CONSTRAINTS (user-set, revised 2026-07-19)

1. HONEST PHYSICS, WHEREVER IT LEADS (user: "if the physics makes sense
   for a horn resurrection then let it be so — might be interesting").
   The fulcrum is a real contact: support bounded by what a static lip
   can provide (<= gravity counteraction — it can HOLD, never LAUNCH),
   inelastic (zero restitution — no bounce energy), pivot friction
   bleeds energy. If a beetle's trajectory genuinely carries it back
   over floor mid-pivot, recovery is ALLOWED (see the re-landing
   checkpoint). What is BANNED is energy injection and junk states —
   "if you're clearly dead, no weird physics interaction": below the
   lip band the beetle is committed and gets ZERO interaction, ever.
   NOTE physics reality check: edge tipping fires when the CENTER is
   over void — a horn-hung beetle's center of mass is past the lip, so
   gravity wins in almost every catch. Recoveries should be RARE and
   visibly earned (entered with spin/sideways momentum that pivots the
   center back over floor). If testing shows frequent or unearned
   recoveries, tighten the support cap — not the honesty.
2. ALL ARENA SHAPES. Circle rim, goal pits, hole mode, donut inner edge,
   X-stage, cut-square, figure8, squiggle, board-break holes — every
   mode's floor boundary must work identically.
3. DON'T TOUCH DETECTION. Edge tipping + the fall trigger work well
   today ("edge detection actually works pretty well when they die").
   This plan changes NOTHING about when/whether a beetle falls — only
   what happens after `is_falling` is set.

## Design: sample the REAL floor under the extremities (no analytic edges)

The all-shapes guarantee comes free if we never describe edges at all:
while falling, sample `check_floor_collision(x, z)` under the beetle's
extremity points. Wherever ANY mode has floor, the sample says so;
wherever it has void, it says void. Mode shapes are irrelevant — we read
the truth. (This also honors the analytic-detection ban: the floor grid
IS the grid.)

The clip is a FULCRUM situation: center over void (falling), horn tip
over solid floor, tip at/below floor top = the horn is resting on the
lip. Physical response: the beetle pivots AROUND the lip and slides off
it — never through it, never back up onto it.

### The response (honest contact, energy-bounded)

For a supported extremity point P (over floor, within the lip band):
- VERTICAL: support force counteracts gravity AT MOST (damp downward vy
  toward 0; the lip can hold weight, never throw it). Zero restitution
  — catches are inelastic thuds, not bounces. Net: vy can reach ~0 on
  a clean catch but the fulcrum itself never produces upward motion;
  any rise must come from the beetle's own pivot momentum.
- ROTATION: pitch_velocity driven by the real torque sense — gravity
  on the unsupported mass around the lip point. Rate-capped (reuse the
  smoothness-era tilt caps). Pivot friction damps rotation each substep
  so hanging bleeds energy (this is what makes recoveries rare and
  stalls impossible).
- SLIDE: horizontal motion follows the pivot — the contact constrains,
  it doesn't propel. Bounded (<= ~0.3/substep positional correction to
  keep the tip out of the floor).

### Junk-state guards (the "clearly dead" half of constraint 1)

- Lip band: point must be within ~3 voxels below floor top. Deeper =
  already past the lip, free fall (no late weird catches).
- Depth fade: entire system off once center y < ~-6 (well below floor)
  — a committed fall is a clean fall, zero interaction.
- Only engages while vy < 0 (an ascending death-launch arc gets no
  fulcrum — it isn't resting on anything).
- TELEMETRY (not a hard gate anymore): `fall_y_increases` counts
  substeps where a falling beetle's y rose. Expect near-zero; each one
  should correspond to a visible, earned pivot — spikes = energy leak,
  investigate.

### RE-LANDING CHECKPOINT (decides if recovery is even possible)

Find where the fall-death event actually fires (score/lives decrement:
at is_falling START, or at DEPTH e.g. the explosion threshold?).
- If at DEPTH: recovery is clean — add re-landing: while is_falling,
  inside the lip band, if the CENTER is back over solid floor with
  small |vy|, clear is_falling (normal floor collision resumes; the
  beetle re-lands). No un-scoring needed since nothing fired yet.
- If at FALL START: recovery would require un-scoring a death across
  the network — OUT OF SCOPE; ship the fulcrum as tumble-shaping only
  and record the finding here. (Do NOT move the death event as a side
  effect — that's a game-rules change the user decides separately.)

## Perf note (why F1 starts with ONE sample)

check_floor_collision is a KERNEL call (~0.15ms round-trip on CPU
backend — see the July floor-batching negative result: these are NOT
free). Budget: F1 samples ONLY the horn tip (the visible clipper —
the butt falls first and clean per the user report) = 1 extra kernel
call per FALLING beetle per substep, only inside the lip band
(~0.3-0.5s per death). Transient ~0.4ms/frame during a death tumble,
nothing at any other time. F2 adds body endpoints ONLY if body clipping
is actually visible after F1 (2 more calls, same math) — do not add
them speculatively.

## Phases

### F0 — confirm the clipper (15 min, no behavior change)
Debug print/trace while is_falling: horn tip position, floor sample
under it, over-floor + below-top flag. One rhino edge death at a goal
pit + one at a mode edge. Confirms the horn tip is the part in the
floor and gives the band numbers for F1. (BALL_TRACE precedent — flag
`--falltrace`, strip after F1 ships or leave behind the flag.)

### F1 — horn-tip fulcrum
Implement the response above for the horn tip only (tip = the CONTACT
arm's tip from horn_collision_segments — memoized, already per-arm
correct). Includes the RE-LANDING CHECKPOINT investigation (where does
the death event fire?) — recovery ships in F1 only if it's the clean
at-DEPTH case. Gates:
- Rhino butt-first death at pit edge + arena rim: horn visibly pivots/
  slides off the lip instead of knifing through. User feel verdict.
- fall_y_increases stays near zero and every nonzero corresponds to a
  visible earned pivot (no invisible energy leaks).
- Typical deaths still die: fall duration to explosion within normal
  variance; a beetle must never STALL hanging on a lip (pivot friction
  guarantees energy bleed — verify with a max-fall-duration stat).
- If re-landing shipped: recoveries are RARE and read as earned saves,
  never as glitches. If they're common, tighten the support cap.

### F2 — body endpoints (ONLY if F1 leaves visible body clipping)
Same fulcrum for the spine capsule front/rear endpoints
(_body_spine_capsule). Same invariants. Skip entirely if F1 suffices.

### F3 — all-shapes sweep + canary
- Manual: one death at each edge type — goal pit mouth, hole mode,
  donut inner edge, X-stage corner, cut-square corner, board-break
  hole, figure8/squiggle narrows. All should read as pivot-and-slide.
- Canary (--local4 bots, lives mode so deaths occur): FPS within
  baseline, fall_y_increases 0, no bot death hangs (a beetle must
  never fulcrum-stall on a lip — the voidward slide guarantees exit,
  but verify: max fall duration stat if needed).
- Multiplayer note: host-authoritative — fulcrum runs host-side only,
  guests see the synced result. No protocol change. (v6 field test
  still pending separately.)

## V10 (2026-07-19): THE PIVOT — measured, not theorized (--tiptrace)
V9's lever sum was measured about the WHOLE-BODY COG: the trace showed
torque collapsing 441->213->89->34->~0 within ~15 substeps of walking
off (cnt steady ~360) — once the over-edge cloud surrounds the COG its
levers SELF-CANCEL, so every fall got rotation only from the first
fraction of a second, then coasted ("minor/shitty tilt", direction
decided by noise). Physical model: tipping pivots about the SUPPORT
region, not the COG. torque = sum(over-edge voxels) - count*support_
centroid (linear identity) — one-sided by construction while any
support exists, GROWS with overhang; fully unsupported = free fall
(momentum carries). Torque now also gated on sup_count>0. LESSON FOR
THE FILE: three consecutive theory-only fixes (V7 frame, V9 signs+sum)
each moved the wrong lever; ONE --tiptrace run found it. Instrument
first. (--tiptrace flag kept for future tuning.)

## V9 (2026-07-19): MASS-TRUE tip direction (user-diagnosed)
The tip direction came from the TOP-3 FARTHEST over-edge voxels
(winner-take-all within 90% of max lever) — the horn tip, always the
farthest thing, HIJACKED the direction whenever it hung over ANY void
(far side of a hole, a second edge), out-voting dozens of body voxels
over the real edge. Replaced with the physical model: net torque = SUM
of ALL over-edge lever vectors (per-voxel torque ~ lever arm); opposing
overhangs cancel. x3.0 parity constant keeps felt strength near the old
tuning; EDGE_TIP_SCALE is the live dial. edge_tipping_vy now scales
with ONE-SIDEDNESS (|lever sum| / (count*max)) — a beetle centered over
a big hole drops straight (gravity only, no rotation, no extra kick);
a one-sided overhang gets the full pull. Emergent bonus: symmetric
support loss = clean straight drop, which was previously random.

## V8 (2026-07-19): FULL AUDIT + falling-body physics gate
Complete audit of every writer of beetle pitch/roll/vy/on_ground (agent
audit, findings verified): on_ground is STICKY and host/local walk-off
deaths NEVER clear it, so death tumbles ran under STANDING physics —
(1) level-restoring springs un-tumbled the dying beetle each substep
(the "weak arbitrary tips"), (2) the 92-deg ground tilt wall froze
tumble rotation, (3) the grounded 8.0 tilt-speed cap throttled spin,
(4) guests DO clear on_ground at death => host/guest ran DIFFERENT
death physics, (5) the floor-rest vy clamp was one stale flag away from
freezing falls entirely. ALSO: apply_bowl_tilt is DEAD CODE (never
called — there is NO slope posture for beetles at the rim), and legs
play ZERO role in support (center 7x7 scan only — user suspicion
confirmed). FIX SHIPPED: `_dying = is_falling or no_floor_below` gate
in update_physics — dying/over-void beetles skip ALL self-righting
(air+ground restoring, close-enough finish), skip the 92-deg wall, use
the airborne tilt-speed cap, and the rest clamp hardens against the
stale-flag freeze. Standing/combat physics untouched. SIDE EFFECT to
feel-check: beetles launched OVER A PIT mid-combat now tumble instead
of self-righting while over the void (arguably more natural; flag if
it reads wrong). Host/guest death physics now consistent.

## V7 (2026-07-19): THE frame bug — tipping torque was WORLD-frame
The kernel mapped world-space levers DIRECTLY to body pitch/roll
(lever_z -> pitch, lever_x -> roll; beetle rotation never passed in!) —
the tip direction was fixed in world axes, reading as random relative
to facing (user: "always tips the same way, half the time feels
normal"). Since the ORIGINAL kernel. Fixed: rotation is now an arg,
levers project onto facing/right; unsupported mass AHEAD -> nose dips,
RIGHT -> right side dips (bowl-tilt sign convention). All 3 call sites
updated (warmup, straddle lean, main tipping). Every downstream feel
judgment made before this fix (fulcrum torque strengths, lean scale)
deserves a re-test — they were tuned against corrupted rotation.

## V6 (2026-07-19): ROOT CAUSE of wrong-way tipping — the tipping
kernel's "over-edge" test was an ANALYTIC per-mode catalog (circle
radius + donut/X-stage/barbell/figure8 branches). Uncatalogued edges
(yin-yang eyes etc.) miscounted over-edge voxels -> tipping zero or
wrong-signed there (user: "the side closest to the BOARD dips"). Fixed:
grid-truth OR'd in — floor_height_cache[i,k] < -100 marks a column
over-edge regardless of mode. This also fixes the STRADDLE lean
direction and in-hole fall rotation at the source (same kernel).
Catalog branches kept as refinements. NOTE: kernel change = one-time
JIT recompile on next launch (~cache rebuild).

## V5 (2026-07-19): roll torque + side sample points
Left/right body points added (local +-4 lateral at spine mid height) —
the axis points have no lateral lever, so roll needed its own samples.
Roll sign mirrored from apply_bowl_tilt (right side uphill ->
target_roll NEGATIVE = right up): jam on the right -> roll_velocity
negative (right side rises over the rim). Same penetration-proportional
magnitude, +-6 cap. Sample count now 5 per falling beetle per substep
(still band-gated + depth-faded; transient death-fall cost only).

## V4 (2026-07-19): jams also TORQUE — the rim pushes up on the caught
part, rotating it over the edge (user: pitched-DOWN horn through a
yin-yang corner must TURN the beetle nose-up, not stop-and-clip).
Pitch sign verified against apply_bowl_tilt (NEGATIVE pitch = nose up):
front point jammed -> pitch_velocity negative (nose over rim); rear ->
positive. Penetration-proportional, depth-faded, |pitch_vel| capped 6.
Roll torque SKIPPED deliberately: all three sample points lie on the
local X axis (lateral offset ~0), so roll leverage is nil.

## V3 (2026-07-19, after v2 testing): rotation is a CONTACT CONSTRAINT
User: "we have the tilt and then the horn can instantly clip" — the tilt
rotation itself sweeps far-side parts into the rim (a small hole's far
rim is always in reach). V1 damped rotation ALWAYS (froze the tumble);
v2 damped NEVER (swept through). V3: tumble freely until a sampled part
actually PENETRATES rim floor (below floor top -0.2), then pitch/roll
stop AT the contact pose (x0.3, depth-faded) — the beetle WEDGES;
stronger voidward slide (0.3) scrapes it free; depth fade releases into
clean fall. Resting-band contact keeps mild vy damp + full rotation.
THE PRINCIPLE (belongs in memory eventually): free motion until touch,
constraint at touch — never blanket friction, never zero response.

## V2 (2026-07-19, after user hole-testing): three v1 mistakes fixed
1. HOLE SLOW-MO: inside a hole every substep has a rim catch — the
   constant 0.45 vy damp compounded into floating. Now: mild 0.25 damp
   x a depth fade that reaches ZERO at the point of no return (-5).
2. FROZEN TUMBLE: v1's blanket rotation friction FOUGHT the edge-tipping
   torque — the pose froze and parts swept through the rim. Rotation
   damping REMOVED entirely; tipping owns rotation ("the turn IS the
   natural clip resolution"). LESSON: never friction-damp a rotation
   another system is deliberately driving.
3. STIFF STRADDLE: deep lean now RELEASES the straddle (|pitch|+|roll| >
   STRADDLE_TIP_RELEASE, slider, default 0.5 rad) — leaned past
   recovery = center over the void = commit and fall in naturally.

## STRADDLE SUPPORT (added + shipped 2026-07-19, same session)

User follow-up: yin-yang / hole-mode holes in the 8-16 voxel band
swallowed beetles whose ~20-voxel stance visibly spanned them (the
center sample scans only 7x7). Shipped: when the center reads void,
sample the four rotated stance points (+-5 local); OPPOSING support
(front+rear or left+right) = bridging -> stand on the supporting floor
with a damped lean toward the hole (tipping kernel at 0.35, pitch/roll
only, applied on fresh samples so parked straddlers can't accumulate
tilt). One-sided support = genuine edge -> unchanged tip-and-fall (goal
pits stay deadly — their far side is always void). Mode overrides
(hole drop radius, board-break mask) replicated PER SAMPLE — the height
cache doesn't know dynamic holes, so without this a beetle could
phantom-stand on a broken board. Per-slot straddle cache (>1 voxel
movement re-sample) keeps parked-over-hole cost at zero kernels.
Telemetry: straddle_substeps. TEST: yin-yang eyes (stand + lean),
hole mode small vs grown hole (stand -> fall as it grows), board-break
(must NOT straddle broken cells), goal pit (must still die), donut
inner edge (one-sided -> falls).

## Explicitly out of scope
- Edge tipping / fall detection (works, untouched)
- Any pre-death edge contact for LIVE beetles (combat balance — the
  fulcrum exists only inside is_falling)
- Ball systems (already shipped)
