# 4 PLAYER STEAM — Plan for 4 Real Accounts Connected and Smooth

Written 2026-07-07 (branch `arena_mods`, after the perf/collision sprint:
batched pair collision kernel, GGUI buffer cuts, horn anti-clip — brawl frame
~28ms → ~20ms). Companion docs: `last_network_stage.md` (detailed code map +
gotchas — READ IT FIRST), `NETWORKING_REWORK_PLAN.md` (architecture),
`PHASE4_INVENTORY.md` (per-slot GPU details).

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
3. **Score credit rule for 4P FFA now: killer-credit if known, else no
   credit.** Victim's death always broadcast (needed for respawn/lives
   later). Use the existing last-attacker style data if trivially available
   (contact within last ~3s via `pair_collision_last` / collision events);
   otherwise ship "fall = no credit" first and iterate — do NOT block
   plumbing on attribution polish.
4. **4P score HUD: text/GUI first, voxel digits later.** The floating voxel
   digits are blue/red only (types 23/24). For this milestone show 4 scores
   in the existing GUI HUD (colored labels per slot). Voxel digits for P3/P4
   are cosmetic backlog.
5. **Bots are the test fleet.** Host-side bots must traverse the REAL
   packet path (`_handle_packet` with a registered fake peer) so that with 2
   humans + 2 bots, every byte a real 4-human match would send is exercised
   except Steam fan-out to >1 real guest — which is exactly what the friends
   test at the end covers.

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

### A6. FFA scoring generalization (minimum for a playable 4P match)
- Death credit: remove the `active_player_count == 2` guards in the
  explosion/fall paths; host determines scorer per decision #3 and sends
  MSG_SCORE(scorer, victim) — `NO_CREDIT` sentinel (e.g. 255) allowed.
- Guest score-apply handler ("pending_scores.pop" site): scorer==0/1
  branches → `scores[scorer] += 1` for any slot; victim slot drives the
  respawn/celebration hooks it already drives.
- Win condition: first to N kills (existing 2P rule generalized: check all
  4). Rematch resets scores[0..3].
- **Gate: 2PC+2bots, play to win. Scores identical on host and guest at all
  times (log both), winner banner correct on both.**

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
