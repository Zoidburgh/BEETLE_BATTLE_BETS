# 4 PLAYER STEAM — Plan for 4 Real Accounts Connected and Smooth

Written 2026-07-07 (branch `arena_mods`, after the perf/collision sprint:
batched pair collision kernel, GGUI buffer cuts, horn anti-clip — brawl frame
~28ms → ~20ms). Companion docs: `last_network_stage.md` (detailed code map +
gotchas — READ IT FIRST), `NETWORKING_REWORK_PLAN.md` (architecture),
`PHASE4_INVENTORY.md` (per-slot GPU details).

## STATUS 2026-07-08 — M1 BUILD COMPLETE + COMBAT-FEEL SPRINT DONE (pushed through 3b35073)

NETWORK CODE IS UNCHANGED since the M1 build below — everything after
3eef8bd is combat feel + geometry, all offline-canary'd, none of it
touches protocol/netcode. **THE ONLY REMAINING M1 WORK IS THE TESTING
LADDER (Rungs 1-5 below) — start at Rung 1 (2-PC + 2 bots).** If picking
this up cold: read `last_network_stage.md`, then run Rung 1.

Post-M1 combat/geometry commits (e24ff7b..3b35073, pushed 2026-07-08):
- Predictive horn-tip gate REMOVED entirely (user call; feel + clip
  metrics both better without it)
- ALL lift delivery continuous per-step (LIFT_STEP_DIV=9): press-down,
  evenly-matched, advantage launch — no more 0.1s pulse pops
- Crisper tip approach (predictive push dist 3.0, contact tolerance +1)
- Yaw grind (left/right horn into a body) stronger + slider-tunable
  (YAW_GRIND_PUSH 60 / LIFT 40 / TILT 0.03)
- Hercules bottom jaw: constant-K curvature (prong slider extends
  instead of reshaping)
- Giraffe weevil: steeper neck (~61 deg), half-segment slider growth
  (every step a visible increase), head attachment climbs every step
- Giraffe anti-clip REBUILT (52f5b9c+3b35073): TWO-segment horn model
  (neck + head joined at pivot; the old straight base->tip chord passed
  ~10 voxels under the elbow = zero coverage there), plus fixed the
  always-red pivot bug (beetle.color is a voxel id, the string compare
  never matched — every giraffe used slot 1's pivot). Giraffe-bot
  canary: deep clips 0.4% of frames (best ever), frame ~18.4ms.

## STATUS 2026-07-07 EVENING — M1 BUILD WORK COMPLETE (e281b4f..3eef8bd, pushed)

ALL implementation steps below are DONE and committed:
- A1 host bots (--bots N via _handle_packet; solo bot match test rig:
  host + START BOT MATCH with no guest runs full online path on 1 machine)
- A2 match size from roster (apply_online_match_size at all 3 start sites)
- A3 guest my_slot everywhere + corrections fan out to ALL remote beetles
- A4 lobby max 4, START gate waits for ALL real guests (bots exempt)
- A5 protocol v5 (game_mode/team_of_slot/lives_per_player; auto-derives
  mode 1 + lives 3 for >2P rosters)
- A6 FFA-LIVES (lives[4]/eliminated[4], NO_CREDIT=255 victim events,
  last-standing win, offline auto-rematch) — GATE PASSED, full bot match
  played to eliminations + winner + auto-rematch; ALSO validated by user
  in solo bot match over the online path
- A7 lives HUD (text, per-slot, winner banner)
- Config relay (per-slot configs, host relays to other guests, slots 2/3
  apply via rebuild_beetle + p3/p4 palettes)
- A8 departures (slot-resolved, match continues, score_type=2 elimination
  broadcast keeps guests consistent)
- BONUS: ghost overlay left panel click-interception fixed (HOST button
  toggled fullscreen at large windows); engagement-resistance tuning
  (HORN_LOCK_TURN_FACTOR / HORN_DAMPING_CAP sliders) + depth-aware turn
  clamp — user-validated: faster fights, deep-clip severity better than
  the old hard blocks (min_shaft_center_dist 1.1 vs 0.4)

### WHAT'S LEFT (testing + small follow-ups)
1. 2-PC test: host `--bots 2`, second machine joins (BOTH on this build —
   v5 refuses stale builds). Verify: guest controls right beetle, bots
   smooth on guest, lives identical both ends, N-key HUD sane.
2. Same with `--simlag 80 --simloss 3`.
3. Departure gate: kill guest process mid-match — host + bots continue.
4. 4 real Steam accounts session (fan-out to >1 real guest is the only
   untested path; also friends' GPU perf fleet).
5. FOLLOW-UPS (known, non-blocking): online rematch flow after a lives
   win (banner stays; host currently re-creates lobby), P3/P4 voxel score
   digits + 4 rim digit stations (cosmetic), per-slot confetti colors
   (suppressed at 3+P by design), bot charge-commitment polish,
   per-peer disconnect timeouts (wall-clock timer is connection-global;
   Steam lobby events cover departures today).
6. BEFORE HUMANS PLAY SPIDER/SCORPION IN SLOTS 2/3: silk_charge_blue/red
   and venom_recharge_timer_blue/red are still 2P-named singletons ->
   [4]-arrays needed (bots are rhinos so the 2PC+bots tests don't hit
   this; slots 0/1 humans are fine). Also ~2ms intermittent unattributed
   cost in the hazards/downwash timing span during heavy bot-death churn
   (visible in the new 'hazards' perf bucket) - subdivide to pinpoint.

## Goal

**Four Steam accounts in one lobby, playing a 4-beetle FFA match that feels
like local play.** Score display works for 4. Everything else (2v2 teams,
beetleball at 4P, lives system) is explicitly LATER — but the plumbing built
now must not have to be torn up for it.

### Non-goals for this milestone (but shape decisions now)
- **2v2 team mode** (score + beetleball): needs a per-slot team id and
  team-aggregated scoring. NOT implemented now — but the protocol bump
  reserves the byte (see "one protocol bump" below).
- **Lives/elimination FFA** (and eventually lives for 1v1): needs per-slot
  lives + eliminated state. NOT implemented now — but scoring stays routed
  through the single MSG_SCORE event path so a lives decrement is a drop-in
  handler change later, not a protocol change.
- Reconnect mid-match (dormant `send_reconnect_state` stays dormant; a guest
  dropping must simply not break the remaining players).

## What already works (do not rebuild)

- Host-authoritative netcode, protocol v4: slots 0-3, MSG_SLOT_ASSIGN roster,
  inputs routed by Steam sender id, MSG_INPUTS_ALL (~13B) broadcast every
  frame, state sync v4 at 20Hz carrying `player_count` beetles, MSG_SCORE
  with scorer+victim slots, `pending_scores` queue (survives simultaneous
  deaths).
- Game simulation is fully slot-generic: `beetles[0..3]`,
  `active_player_count`, per-slot physics/collision/death/respawn/render all
  verified under `--local4` with bots. `scores[4]` exists.
- Perf headroom: 4-beetle brawl ~20ms frame on dev machine (CPU backend).
  Perf log has a Network Stats section; `net_log_<role>_<ts>.csv` auto-logs
  1 row/sec online; `--simlag/--simloss` for degraded-network testing.

## Design decisions (flexibility without building modes now)

1. **One protocol bump: v4 → v5, containing everything this milestone AND
   the reserved mode fields.** Mismatched builds already refuse to match, so
   bumps are cheap but each one splits testers — do ONE. v5 adds:
   - Lobby/match config in `send_game_options`: `game_mode` byte
     (0 = FFA-score, reserved: 1 = FFA-lives, 2 = 2v2-score, 3 = 2v2-ball),
     `team_of_slot` packed byte (2 bits/slot, all-zero for now),
     `lives_per_player` byte (0 = score mode, ignored for now).
     Receivers must parse and store these even though only mode 0 ships.
   - Whatever Stage A needs (nothing else known — MSG_SLOT_ASSIGN already
     carries player_count).
2. **All scoring flows through MSG_SCORE(scorer, victim) → pending_scores.**
   No new score paths. The 2P-shaped handlers get generalized to "apply to
   slot N" — then FFA-lives later = same event, different application; 2v2
   later = same event, sum by `team_of_slot`. Death *credit attribution*
   (who gets the point) stays host-side only.
3. **No kill attribution, ever (user decision 2026-07-07).** Tangled deaths
   (two beetles falling while touching, knocked by a third) make credit
   heuristics wrong exactly when fights are most chaotic. FFA at 3-4P ships
   as **game_mode 1: FFA-LIVES** — deaths are victim-only events
   (MSG_SCORE scorer = NO_CREDIT 255), each death decrements the victim's
   lives, 0 lives = eliminated (no respawn, spectate), last standing wins.
   2P online keeps the current kill-score mode for now (tested path; the
   1v1 lives switch later is just the game_mode byte). Full mode/system
   matrix in "SCORING & PRESENTATION ACROSS MODES" below.
4. **4P score HUD: text/GUI first, voxel digits later.** The floating voxel
   digits are blue/red only (types 23/24). For this milestone show 4 scores
   in the existing GUI HUD (colored labels per slot). Voxel digits for P3/P4
   are cosmetic backlog.
5. **Bots are the test fleet.** Host-side bots must traverse the REAL
   packet path (`_handle_packet` with a registered fake peer) so that with 2
   humans + 2 bots, every byte a real 4-human match would send is exercised
   except Steam fan-out to >1 real guest — which is exactly what the friends
   test at the end covers.

## SCORING & PRESENTATION ACROSS MODES (planned 2026-07-07)

Every system hooked to score/death events, and what it does per mode. The
"ladybug shot" = the referee beam: today a death starts a delay timer, at
~0.15s left the referee fires a spiral beam at the SCORER's floating voxel
digit, and the +1 (with pop/squash bounce + particle burst) lands when the
beam hits. Timer fallback covers referee-off / beam-busy / simultaneous
deaths. That ceremony is gain-oriented and scorer-targeted — both invert
under lives, hence this matrix.

### The one event, all modes
`MSG_SCORE(scorer, victim, type, x, z)` — victim-centric, host decides.
score_type already distinguishes 0 = death, 1 = ball goal.
- FFA-score (2P today): scorer = the other slot (unchanged behavior)
- FFA-lives: scorer = NO_CREDIT (255); the event MEANS "victim lost a life"
- 2v2 (both variants): scorer = any slot of the OPPOSING team — no
  attribution needed in team modes, "the other team" is always the answer.
  USER RULE (2026-07-07): in 2v2-ball, a PLAYER falling into a goal pit
  scores for the other team exactly as if they'd scored the ball there —
  same team digit, same ceremony. (This generalizes today's 2P behavior:
  any death including goal-pit falls already gives the opponent +1.)
  Deaths in 2v2-ball cost the point only; respawn is normal (no lives).
Lives are derived deterministically from death events on BOTH ends — no
extra sync message. (State sync can carry lives later for reconnect.)

### System-by-system matrix

| System | FFA-score 2P (today) | FFA-lives 3-4P (A6) | 2v2 score/ball (later) |
|---|---|---|---|
| Score state | scores[2] up | lives[4] down, start N (default 3) | team totals from scores[] via team_of_slot |
| Referee beam target | scorer's digit | VICTIM's digit | scoring team's digit |
| Digit change on hit | +1, happy burst | -1, downward "deduction" burst | +1, happy burst |
| Elimination moment | n/a | digit bursts + removed; beetle stays out | n/a (team elim if lives 2v2 later) |
| Digits layout | 2 (above goal ends) | 4 rim digits at N/E/S/W spawn compass points, per-player colors | the SAME 2 goal-end stations become TEAM digits |
| Respawn | always | only if lives > 0; eliminated[slot] blocks respawn timer | per mode (score: always) |
| Win check | first to N kills | last beetle standing | first team to N / team elimination |
| Celebrations/confetti | winner slot (0/1 today) | SUPPRESSED at 3+ players for now (color-coded blue/red drops; per-slot colors decided later) | winning TEAM pair (team colors) |
| Crowd cheer on death | yes | yes (unchanged) | yes |
| Rematch reset | scores | lives | team totals |
| HUD text (A7) | 2 scores | lives per active slot, colored | 2 team scores |

### Refactors this dictates (do once, in A6)
- `blue_score_pending`/`red_score_pending`, `blue/red_score_delay_timer`,
  bounce/burst timers → **per-slot arrays [4]** (score_pending[slot] etc.).
  The beam then takes a SLOT target, not 'blue'/'red'. This is the enabler
  for every mode; the color names die here.
- `eliminated[4]` flag + lives[4]; respawn scheduling checks it.
- Referee beam meaning per mode: mode 0/2/3 -> beam at scorer/team digit
  (count up); mode 1 -> beam at victim digit (count down). Beam is single
  (one referee) — per-slot pending + existing timer fallback absorbs
  death bursts (already proven for simultaneous 2P deaths).
- Digit rendering: needs P3/P4 digit voxel types (+ palette; next free ids
  71+) and 4 rim anchor positions. SHIPPING ORDER: text HUD first (A7),
  beam simply skips slots without a digit station; 4 rim digits are the
  cosmetic follow-up — the beam/ceremony code is written against slots
  from day one so digits just plug in.

### Deliberately NOT decided yet (flagged for later, nothing blocks)
- Score/victory confetti colors at 3-4P (drops are blue/red-coded today):
  USER CALL 2026-07-07 — simply DON'T spawn confetti/celebration effects
  in 3+ player matches for now; design per-slot colors later. A6 gates
  those effects on active_player_count <= 2.
- Friendly-fire rule in 2v2 (teammate shove-offs — other team still scores
  per the death rule; question is whether to add any extra penalty)
- Lives count per mode/UI to change it (host option; byte already in v5)
- Whether eliminated players get a spectator camera (they see the match;
  fancy cam later)

## ROADMAP: MILESTONES TO FULL MODES

**M1 — TRUE 4P over Steam (current milestone, FFA-lives).**
Remaining: A6 lives + per-slot ceremony arrays, A7 text HUD, config relay,
A8 departure robustness → 2PC+2bots test → 4 real accounts. Ball stays
disabled at 3-4P (as today at network start). DONE = definition-of-done
checklist below.

**M2 — 2v2: score + beetleball.** Everything protocol-side already ships
in v5 (game_mode 2/3, team_of_slot). Work is game logic + UI:
- Lobby team assignment (host arranges slots into teams; team byte sent)
- Team spawns (teammates same side), team-colored accents if desired
- Scoring: victim dies OR ball goal -> opposing team +1 through the
  existing per-slot ceremony arrays aggregated by team_of_slot; the 2
  goal-end digit stations become the team digits (no new digit assets)
- 4P ball physics already works (ball is slot-agnostic; pits are 2-sided
  which is exactly the team layout)
- Win: first team to N; rematch resets
- Decide friendly-fire rule here

**M3 — lives everywhere.** FFA-lives machinery from M1 IS the lives
system; flipping 1v1 (and 2v2 if wanted) to lives = game_mode byte + a
host lobby option. No new plumbing.

Ordering rationale: M1 proves 4 real connections with the simplest ruleset
(nobody argues with "last standing"); M2 reuses M1's per-slot ceremony
arrays and the v5 team byte; M3 is configuration.

## MULTI-BALL ONLINE (MB5 — protocol v7, planned 2026-07-18)

Context: multi-ball SHIPPED OFFLINE 2026-07-18 (3 balls, billiards,
per-ball respawn pipeline, Ball Count menu slider; perf target met same
day — 3-ball scrum ~29 FPS, plans/ball_perf_plan.md). The network layer
is still deliberately single-ball. MB5 skeleton lives in
plans/multi_ball_plan.md; this section is the network-side plan + traps.

### Why 3 balls online breaks TODAY (v6) — three gaps, all known
1. State sync carries exactly ONE ball block (balls[0]). Balls 2/3 never
   cross the wire.
2. Guests never spawn the extras: game options sync only ball ON/OFF, not
   count; spawn_extra_balls() is only called from local UI paths. Host
   would play 3 balls, guest sees 1 — and gets shoved by invisible balls
   (their beetle corrects toward host state that includes collisions with
   balls the guest can't see).
3. Score/respawn/explode events don't carry a ball index — guest
   celebration + respawn machinery is single-ball.

### TRAP (guard SHIPPED 2026-07-18): the Ball Count menu slider persists
in MULTI_BALL_COUNT, and the in-game BEETLE BALL toggle spawns extras from
it — including while hosting online. GUARD: spawn_extra_balls() itself
clamps to 1 when game_state is ONLINE_PLAY AND a real guest is connected
(has_real_guests(); prints a notice) — solo bot matches keep full ball
count, they're the multi-ball test rig and have nobody to desync. Every
call site present and future is covered. REMOVE the clamp when v7 lands —
it is the single line keeping real online matches single-ball.

### Order of work (v6 must not stack untested under v7)
0. Commit outstanding physics work + add the online 1-ball guard.
1. Run the still-pending v6 test FIRST (Rung 1 below, ONE ball) — the
   30Hz angular-rate sync has never seen 2 real PCs; if it has issues,
   find them before v7 changes the same packets.
2. MB5 session (protocol v7, ~one session — offline per-ball plumbing
   from MB1 makes this protocol work, not physics work):
   - send_state_sync: ball_count byte + N ball blocks (exact mirror of
     the v4 player_count pattern). PROTOCOL_VERSION 6 → 7.
   - Guest apply: loop balls — per-ball correction targets + per-ball
     visual error offsets (de-singleton the apply path).
   - set_network_ball_mode + game options carry ball count (ALL
     send_game_options call sites, replace_all; one bump covers all).
   - Ball-index byte on MSG_BALL_EXPLODE + score/respawn events.
3. GATE MB5: --simlag/--simloss 3-ball run, then 2-PC 3-ball match.
   Verify: balls stay DISTINCT (sync by index; count fixed at match
   start so identities can't swap), goals credit correctly, per-ball
   respawns clean on guest, guest feel under correction with 3 balls
   moving, guest FPS (guest renders 3 balls too — the 2026-07-18 perf
   fixes apply to both ends but were only measured on host).

Host stays the ONLY ball authority — guests never simulate ball
outcomes, only display corrected copies (the offline-validated physics
feel is never at stake in this milestone).

## Implementation steps (each has a test gate; commit after each)

### A1. Host-side network bots — `--bots N`
- Reuse `get_bot_inputs(slot)` (bots already fence with horns).
- Each physics frame, host packs a real MSG_INPUT for each bot slot and
  feeds `network_manager._handle_packet(data, fake_steam_id, input_buffer)`.
- Register fake peers via `_register_peer(fake_steam_id)` when the match
  starts (AFTER real guests, so humans get low slots), so slot routing,
  last-known-input fallback, and MSG_INPUTS_ALL aggregation all run
  production code. Precedent: `--phantom-peer`.
- `_broadcast` to fake ids: sends fail per-peer and are already caught —
  verify no log spam (silence failures for flagged bot peers).
- Bots never send SYNC_READY → START gate must not wait on them (see A4).
- **Gate: 2 computers, 1 guest + 2 bots. Guest sees 4 beetles fighting,
  net CSV shows game_speed ~100% on both ends, N-key HUD sane.**

### A2. Match-start player count from the lobby (host)
- At SYNCING → ONLINE_PLAY transition: `active_player_count =
  network_manager.player_count` (humans + registered bots), never hardcoded.
- Ensure `beetles[2]/[3]` exist + `rebuild_beetle(slot, ...)` called (mirror
  `reset_match`, which already loops `range(2, active_player_count)`), and
  ring spawns via `get_spawn_position(slot)`.
- Slots 2/3 kernels JIT on first use — acceptable for now, note the ~stutter
  at match start; move their warmup into loading if it annoys.
- **Gate: same 2PC+2bots session, but player count comes from the roster —
  start with 1 bot (`--bots 1`) and confirm a 3-beetle match works too.**

### A3. Guest-side slot generalization (the known landmine)
- Kill BOTH hardcoded `local_player_id = 1` sites (join flow and START
  handler): use `network_manager.my_slot` for `local_player_id` AND
  `input_buffer.local_player_id`.
- Guest `active_player_count` from MSG_SLOT_ASSIGN's player_count (already
  parsed into `network_manager.player_count`).
- Generalize `guest_opp_target` (single dict for `1 - local_player_id`) to
  `guest_opp_targets[slot]` for every non-own slot — state sync v4 already
  delivers all beetles; the sync-apply loop just needs to fan out its
  correction to each.
- Grep for remaining `1 - local_player_id` / `1 - slot` in network paths
  (`opp = beetles[1 - slot]` in input processing is LOCAL gameplay pairing —
  fine for now, it maps 0↔1/2↔3 via slot^1).
- **Gate: 3 humans? Not yet — use 2 PCs where the real guest is deliberately
  assigned slot 2 (host + 1 bot registered first). Guest must control the
  RIGHT beetle and see corrections applied to all others.**

### A4. Lobby & start flow for 4
- `create_lobby(..., max_players=2)` → 4 at both call sites (or a host UI
  choice 2/3/4 — simplest: always 4, match starts with whoever's there).
- Host START gate: wait for SYNC_READY from ALL real guests (bots exempt).
  Track which peers are real vs bot in the peers dict.
- Lobby UI: list all connected players + slots (text is fine), host sees
  "START (N players)".
- **Gate: join order shuffling — guest joins before/after bots registered,
  START always produces a consistent roster on both machines.**
- NOTE: bots must be registered at a deterministic point (match start, not
  lobby) so lobby roster = humans only and slot assignment stays stable.

### A5. Protocol v5 bump — game options with mode/team/lives fields
- Extend `send_game_options` (+ its parser) with `game_mode`,
  `team_of_slot`, `lives_per_player` as described above. ALL call sites in
  beetle_physics.py updated (use replace_all; see memory note — the legacy
  byte-length compat cascade is gone, so length changes REQUIRE the bump).
- `PROTOCOL_VERSION = 5`.
- **Gate: v4 build vs v5 build refuse to match with a clear message; two v5
  builds match; options (arena modes etc.) still sync.**

### A6. FFA-lives for 3-4P matches (see SCORING & PRESENTATION matrix)
- Per-slot arrays replace blue/red score-pending/delay/bounce/burst state;
  referee beam takes a slot target. 2P keeps mode-0 behavior through the
  same arrays (slots 0/1).
- lives[4] + eliminated[4]; host death paths (fall + explosion) send
  MSG_SCORE(NO_CREDIT, victim) for 3-4P matches (game_mode 1); both ends
  decrement lives[victim]; at 0 set eliminated -> respawn timer skips.
- Win: last standing (mode 1) / first-to-N (mode 0). Rematch resets both.
- Guest score-apply handler generalized to any slot + mode-aware.
- host sets game_mode=1 + lives_per_player=3 in game options when
  active_player_count > 2; --local4 uses the same path offline (canary).
- **Gate: --local4 plays a full lives match to elimination + winner;
  solo bot match same over the online path; scores/lives identical on
  host and guest in the 2PC test.**

### A7. 4-slot score HUD (text first)
- In-match GUI: one line per active slot, colored label
  (BLUE/RED/GREEN/GOLD) + kill count. Hide inactive slots.
- Leave voxel score digits as-is for slots 0/1; add P3/P4 digits later
  (cosmetic backlog, needs new voxel types + palette entries).
- **Gate: eyeball in 4P match; scores update on kill for all slots.**

### A8. Departure robustness (not reconnect — just don't break)
- One guest quits/crashes mid-match: `_unregister_peer` fires, their beetle
  goes inactive (despawn or corpse-explode — pick despawn, it's simplest),
  match continues for the rest, scores keep working.
- Host quits: guests get the existing disconnect flow (back to menu).
- **Gate: kill the guest process mid-brawl; host + bots keep playing.
  Rejoin = new match (reconnect explicitly out of scope).**

## TESTING GUIDE (written 2026-07-07 night — build 6e7dd63+)

### Rung 1 — 2-PC + 2 bots (the workhorse, do this first)
1. Machine 2: `git pull` (protocol v5 refuses stale builds — a mismatch
   error means someone didn't pull).
2. HOST machine (the stronger PC): `py -3.12 beetle_physics.py --bots 2`
   → HOST ONLINE GAME → COPY LOBBY ID.
3. Guest machine: JOIN ONLINE GAME → paste ID → connect. (Or launch with
   `+connect_lobby <id>` to auto-join.)
4. Host clicks START MATCH (4 players).
CHECKLIST:
- [ ] Guest controls the RIGHT beetle (red, slot 1)
- [ ] All 4 beetles fight smoothly on BOTH screens (bots must not
      rubber-band on the guest — that's the A3 multi-target correction)
- [ ] Lives HUD identical on both machines through several deaths
- [ ] Eliminations stick; winner banner appears on both
- [ ] Save perf logs on both ends; net CSVs auto-record. Want:
      game_speed 98-102%, accumulator_drains ~0, snaps low (N-key HUD)
- [ ] Guest customizes beetle in lobby -> host sees it (config relay)
### Rung 2 — degraded network
Same setup + `--simlag 80 --simloss 3` on the host. Playable? Corrections
visible but not nauseating?
### Rung 3 — departure
Kill the guest's process mid-match (Task Manager). Host + bots must keep
playing; the guest's beetle despawns and is eliminated. Then guest
rejoins the lobby -> fresh match works.
### Rung 4 — 3 humans + 1 bot (first friend)
First test of host fan-out to TWO real guests (the only thing bots can't
simulate). Friend pulls the build; host `--bots 1`. Guest #2 gets slot 2
(green) — verify they control the GREEN beetle (A3's my_slot fix).
### Rung 5 — 4 real Steam accounts
Everyone on the same build. Full FFA-lives match to a winner. Collect
perf logs + net CSVs from every machine (friends' GPUs also feed the
FPSPROBLEM.md data). Celebrate.

## MENU & HOST AUTHORITY DESIGN (agreed 2026-07-07)

Principle: THE HOST OWNS THE MATCH SETTINGS, PLAYERS OWN THEIR BEETLES.
Already true in the plumbing: arenas/hazards sync host->guests via
MSG_GAME_OPTIONS (guest arena UI is already is_host_or_local-gated), and
the v5 options carry game_mode/team_of_slot/lives_per_player. The menu
work is exposing what the protocol already ships.

### Host controls (lobby, and where sensible mid-match)
- Board/arena selection + hazard toggles (EXISTS - already host-gated
  and synced; unchanged)
- Game mode picker: FFA-LIVES (3-4P default) | 2v2 SCORE | 2v2 BEETLEBALL
  (2v2 entries appear in M2; 2P defaults to classic score mode until the
  1v1-lives switch later). Sets the game_mode byte at START.
- Lives count (1-5, default 3) -> lives_per_player byte
- Team assignment for 2v2 (M2): host clicks slots to arrange teams ->
  team_of_slot byte; teammates spawn same side
- Bot fill: lobby button "ADD BOT" / "REMOVE BOT" replacing the --bots
  CLI flag (same register_bot_peers path, host-only)
- START MATCH (N players) — gated on all real guests SYNC_READY (exists)
### Each player controls (host included)
- Their own beetle: type, sizes, colors (config relay EXISTS; guests see
  each other's builds). Beetle editing locks at START.
- Their own camera/display settings (never synced)
### Guests see (read-only)
- Roster with slots/teams, chosen mode + lives, arena/hazard selection
  (mirrors via game options as today)

### Menu build order
- PHASE A (now/testing): what exists — text lobby, CLI bots, text lives
  HUD. Sufficient through Rung 5.
- PHASE B (before the friends session, small): lobby roster list showing
  slot colors + player names + chosen beetle types; ADD BOT button;
  lives count picker; mode label (FFA-LIVES fixed). All host-side UI over
  existing plumbing — no protocol work.
- PHASE C (M2 = 2v2): mode picker gains 2v2 SCORE / 2v2 BEETLEBALL,
  team assignment clicks, team-colored goal digits (the 2 existing digit
  stations), team spawns, friendly-fire decision. Protocol untouched.
- PHASE D (cosmetic backlog): 4 rim voxel digits for FFA, per-slot
  confetti colors, voxel-art lobby.

## Testing ladder (in order, don't skip rungs)

1. `--local4` regression after every step (bots still brawl, no errors) —
   this is the cheap canary, `--perfauto` for perf logs.
2. **2 PCs, 1 real guest + 2 bots** — the workhorse. Both machines save perf
   logs + net CSVs. Watch: game_speed ~100%, physics_per_render ~1.0,
   corrections/snap counts in N-key HUD, input latency feel.
3. Same but with `--simlag 80 --simloss 3` — degraded network must stay
   playable (host-authoritative should shrug; guest correction quality is
   what's being tested with 3 remote beetles now).
4. Join-order and departure chaos runs (A4/A8 gates).
5. **4 real Steam accounts** (friends session): the ONLY thing bots can't
   test is Steam fan-out to multiple real guests + real-world NAT variety.
   Everyone on the same v5 build (protocol gate catches mistakes). Collect
   perf logs + net CSVs from every machine — friends' RTX 3070s are also the
   GPU-perf test fleet (FPSPROBLEM.md history).

## Perf budget & watch items for 4P online

- Host: simulation already proven at 4 beetles (~20ms brawls on dev
  machine). Online adds per-frame MSG_INPUTS_ALL broadcast + 20Hz state sync
  to 3 peers — bytes are tiny; watch `network poll` cost in perf log if the
  peers dict grows the polling loop.
- Guest: now corrects 3 remote beetles instead of 1 — the correction loop is
  Python-cheap, but watch for `guest correction` induced snaps in the HUD
  with simlag.
- Known backlog if a machine struggles: contact-point fold into the batched
  pair kernel (~1ms), extract_all active-region (~1.9ms), Vulkan plan
  (GPU_CPU_OPT_PLAN.md) for the ~8ms render tax.

## Gotchas (hard-won, from last_network_stage.md + this sprint)

- Guest `local_player_id` hardcoded 1 in TWO places — A3 kills both.
- Steam: one account per machine per instance; `+connect_lobby <id>`
  auto-joins for fast testing.
- Commit after every verified step; never `git checkout <file>` reflexively.
- Taichi: `float()` wrap for kernel args from field reads; new kernels warm
  at load or first-use spike; branch-scoped kernel vars need defaults
  declared before conditionals.
- `send_game_options` length change without PROTOCOL_VERSION bump = silent
  desync of options — the compat cascade was deliberately deleted.
- Score events are a QUEUE (`pending_scores`) — never collapse to a single
  pending slot (simultaneous deaths).
- Bot fake steam ids must never collide with real ones (use an obvious
  reserved range) and must be excluded from SYNC_READY waits and real sends.

## Definition of done

- [ ] 4 real Steam accounts complete a full FFA match (score win + rematch)
      with no freezes, no desync, game_speed 98-102% on all machines.
- [ ] Any mix of 2-4 players + 0-2 bots works from the same build.
- [ ] A guest quitting mid-match doesn't disturb the survivors.
- [ ] Scores visible for all active slots on every machine.
- [ ] Protocol v5 carries mode/team/lives fields (parsed, stored, unused) so
      2v2 and lives modes are handler work, not protocol work.
- [ ] `--local4` still passes as the offline regression canary.
