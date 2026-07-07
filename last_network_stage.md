# LAST NETWORK STAGE — What's Left for Full 4-Player

> **SUPERSEDED 2026-07-08 — the "REMAINING WORK" below is ALL DONE.**
> The Stage A/B items (host bots, roster sizing, guest my_slot,
> lobby 4, protocol v5, FFA-lives, HUD, config relay, departures)
> shipped as M1 (commits e281b4f..d741bb5). **Current source of truth:
> `4_player_steam.md`** — its STATUS section + testing ladder (Rung 1:
> 2-PC + 2 bots) is the only remaining M1 work. This file stays as a
> code map: the "Where things stand" section and the per-site code
> pointers below are still accurate and useful.

Handoff doc for continuing the networking rework. Written 2026-07-07 at commit
`4fd0834` on branch `arena_mods` (pushed to `networking_host` on GitHub).
Read this FIRST, then `PHASE4_INVENTORY.md` (GPU/kernel details) and
`NETWORKING_REWORK_PLAN.md` (original architecture + STATUS section).

## Where things stand (ALL DONE, tested live)

- **Netcode**: host-authoritative, no freezes, 0 input delay. Protocol v4
  (`PROTOCOL_VERSION` in network.py; mismatch refuses match in lobby).
  Slots: 0 = host, 1-3 = guests, assigned via MSG_SLOT_ASSIGN, inputs routed
  by Steam sender id, host broadcasts MSG_INPUTS_ALL (~13B) every frame.
  State sync: variable-length, 20Hz, pos/rot/pitch/roll/vel/flags per beetle.
  Score events are a queue (`pending_scores`) — never a single slot (drops
  simultaneous deaths).
- **Game code is fully player-slot generic**: `beetles[slot]` (0-3),
  `active_player_count` (2 normally, 4 under `--local4`), every per-color
  pair converted to 4-slot arrays/loops: input application, physics
  integration, floor collision, edge tipping, collisions (pairwise loop),
  hazards (UFO/comet/spray hit any owner via `simulation.voxel_owner[]`),
  silk (per-slot factory kernels), death/explosion/respawn/hover/downwash,
  render interpolation/animation/dust/shadows, scores[4].
- **GPU**: per-slot geometry fields `beetle_geo[slot]`, factory-generated
  kernels `place_beetle_kernels[slot]`, `render_assembly_kernels[slot]`,
  `lowest_point_kernels[slot]`, `silk_collision_kernels[slot]`,
  `transform_funcs[slot]`. Voxel ids: P3 green = 51-57, P4 yellow = 58-64,
  assembly 65-70 (renderer palette done).
- **`--local4 [--bot-ai seek|random|idle]`**: 4-beetle offline brawl works —
  bots (slots 2/3) are fully simulated, die, respawn with assembly anim.
  Perf: ~51fps at 720p mid-explosion on dev machine; beetle rendering is
  only ~2.4ms for all four — scene_render (~9ms) dominates regardless.
- **Debug tooling**: N-key net HUD, auto CSV logs `net_log_<role>_<ts>.csv`
  (1 row/sec during online play), `--simlag <ms> --simloss <pct>`,
  `--phantom-peer` (host registers a fake guest to exercise broadcast).
  `[AssertPatch]` at boot = workaround for taichi#6513 imgui abort on
  modifier keys (`--no-assertpatch` disables; `--keyshield` is a fallback).

## REMAINING WORK (in order)

### Stage A — Network 4P plumbing (testable with the user's 2 computers)
1. **Host-side network bots** (`--bots N`, reuse `--bot-ai`): each frame the
   host builds a real 6-byte MSG_INPUT packet from `get_bot_inputs(slot)`
   (already exists in beetle_physics.py) and feeds it through
   `network_manager._handle_packet(data, synthetic_sender_id, input_buffer)`
   with a fake steam id registered via `_register_peer` — so slot routing,
   last-known fallback and MSG_INPUTS_ALL rebroadcast all exercise production
   code. Bots occupy slots after real guests. The real guest then sees a full
   4-player match over a genuine Steam link. NOTE: `_broadcast`/slot-assign
   sends to the fake ids will error — already caught per-peer (see
   `--phantom-peer` precedent). Skip SYNC_READY wait for bot peers.
2. **active_player_count from lobby**: currently only `--local4` sets it to 4.
   Online it must become `network_manager.player_count` (incl. bots) at match
   start (the SYNCING → ONLINE_PLAY transition), and beetles[2]/[3] must be
   created + `rebuild_beetle(slot, 12, 5)` called (see reset_match — it
   already does this for `range(2, active_player_count)`).
3. **Guest side**: guest's `active_player_count` from MSG_SLOT_ASSIGN's
   player_count. Guest own slot = `network_manager.my_slot` →
   `input_buffer.local_player_id` and `local_player_id` (currently hardcoded
   1 at match start — must use my_slot for players 2/3!). Guest sync-apply:
   `own_idx = local_player_id` already; the continuous opponent correction
   currently handles ONE opponent (`guest_opp_target`, `1 - local_player_id`)
   — generalize to a list of targets for all non-own slots (state sync v4
   already carries N beetles; the sync-apply loop reads `sync['beetles']`).
4. **Lobby**: `create_lobby("public", 2)` at two call sites → 4 (or a UI
   toggle). Host START gate: today any SYNC_READY triggers GO — wait for all
   REAL guests (bots don't send SYNC_READY).
5. **MSG_BEETLE_CONFIG relay**: guests' configs go host→other guests
   (currently only pairwise). Also `apply_remote_beetle_config` is 2P-shaped.
6. **Reconnect**: `send_reconnect_state` in network.py is DORMANT (never
   wired). Either wire it N-player or leave for later; disconnect of one
   guest must not break the others (peers dict handles roster, but test
   join/leave mid-match — `_unregister_peer` reassigns roster).

### Stage B — Phase 5: game rules + UI
1. **FFA scoring**: `scores[4]` exists. Death credit is currently 2P-guarded
   (`if active_player_count == 2` in the explosion loop + fall fallback in
   beetle_physics). Need: last-attacker attribution (or simple "everyone else
   +1"? user preference), MSG_SCORE already carries scorer+victim slots,
   guest score-apply handler (search "pending_scores.pop") is still 2P
   (scorer==0/1 branches) — generalize.
2. **Win condition + rematch** for N players (first to N kills).
3. **UI**: 4-slot lobby list, in-match score display for 4 (score digits are
   blue/red voxel types 23/24 — needs P3/P4 variants or a different HUD),
   celebrations/pulse/confetti and charge-glow color animation are still
   slots 0/1 only (`blue_celebrating` etc.), beetle config UI for slots 2/3
   (window.blue_*/red_* slider pairs; slots 2/3 use defaults 12/5/6/12).
4. **Ball mode at 4P**: goal pits are 2-team — decide design (disable ball
   at 4P initially is fine; `g['blue_score_pending']` ball keys are 2P).

### Stage C — Final validation
- `--local4` perf gate on the HOST machine (stronger PC hosts).
- 2-computer session with 2 host bots = full 4P pipeline over Steam.
- Join/leave/rejoin stress while bots play.
- One 3-4 human session with friends (fan-out to >1 real guest is the only
  path bots can't cover). All must run protocol v4+ builds.

## Gotchas / lessons (DO NOT RELEARN THESE)
- Taichi: wrap numpy/field-read floats with `float()` before kernel args
  (JIT re-specialization). New kernels need warmup calls during phase-2
  loading (offscreen coords `0.0, -100.0`); slots 2/3 assembly/silk kernels
  currently compile on first use — fine for testing.
- `g` in beetle_physics starts as a dict literal but is rebound to
  `globals()` inside the main loop — `g['x']` IS module global x.
- The merge technique that worked 6+ times: extract both color blocks with
  sed, normalize names, `diff` to verify identical, generate the loop from
  the blue text via script. Scripts live in the session scratchpad; the
  pattern matters more than the scripts.
- NEVER `git checkout <file>` casually — it destroyed uncommitted work once
  (recovered only because every transform was a re-runnable script). Commit
  after every verified step.
- Renderer `get_voxel_color` if/elif palette + shimmer gate must cover any
  new voxel ids. `simulation.voxel_owner[]`/`voxel_part[]` (O(1) lookups)
  are the ONLY sanctioned way for kernels to test beetle ownership.
- Boot test = `py -3.12 -u beetle_physics.py --res 720` (2 min warmup);
  Taichi errors only appear when kernels compile at warmup, not at import.
- Steam: 1 instance per account per machine. 2 PCs = 2 real players max;
  bots fill the rest. `+connect_lobby <id>` auto-joins.

## Key map (names, not line numbers — they drift)
- network.py: NetworkManager, PROTOCOL_VERSION, peers/slot_to_steam/my_slot,
  _register_peer, slot_for_sender, _broadcast, send_inputs_all,
  send_state_sync (v4), send_score(scorer, victim), pending_scores queue.
- beetle_physics.py: beetles[]/active_player_count/scores[] near "Create
  beetles"; InputBuffer (player_inputs per slot); merged per-slot loops all
  live in the main `while window.running` loop; get_bot_inputs;
  get_spawn_position(slot) ring spawns; rebuild_beetle(slot);
  beetle_geo[slot] + kernel factories (search "make_"); LOCAL4_MODE flag.
- simulation.py: voxel ids + PLAYER_VOXEL_IDS + voxel_owner/voxel_part +
  is_beetle_voxel/beetle_owner; p3_*/p4_* colors; silk_on[4]/silk_under[4],
  silk_counts_batched (10: slot*2/slot*2+1, ball at 8/9).
- Testing docs: NETWORKING_REWORK_PLAN.md STATUS section has the 2-computer
  test protocol; net_log CSVs are the measurement tool.
