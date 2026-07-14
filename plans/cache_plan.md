# Kernel Cache / First-Impression Plan (commercial release)

**Problem:** Taichi JIT-compiles each kernel the first time it runs. A fresh
install has an empty offline cache, so a new player gets lag spikes on
first-of-everything (first beetle contact, first time each arena loads, first
ball-mode toggle, first of each effect) plus a slow initial load. A long
*loading screen* is forgivable; *spikes during gameplay* read as a janky game.
Goal: no player ever feels a compile hitch mid-match.

## Background / how the cache works (learned 2026-07-14)
- Offline cache = compiled kernels saved to disk; loaded instantly next run.
- Per-machine, starts empty for every new player. NOT shipped by default.
- Taichi writes the cache at a successful **dump** (roughly at clean process
  work-completion); a force-kill/crash while holding the lock orphans
  `ticache.lock` and silently blocks ALL future writes → permanent recompile
  every session. (This bit us; see DONE below.)
- CPU backend (`arch=ti.cpu`): cache = LLVM machine code, keyed partly by CPU
  features → portable across similar CPUs, graceful recompile (no crash) on a
  mismatch.
- Cache MISS is never fatal — worst case is a recompile spike, never a crash.
- This game generates MANY kernels dynamically (per-slot / per-type closure
  factories: make_place_beetle_kernel(slot), make_transform_func(slot),
  make_render_assembly_kernel(slot), transform_funcs, silk/lowest-point
  factories, etc.) plus per-arena floor/collision kernels. These are the ones
  that spike on first use and must be driven during warmup.

## DONE (committed c87b880 + uncommitted relocation)
- **Stale-lock guard** (simulation.py, before ti.init): delete any
  `ticache.lock` at startup — it's only held ~ms during a real dump, so a lock
  present at boot is always orphaned. Self-heals the cache after any crash /
  force-close, for us and shipped players.
- **Cache relocation** (simulation.py): moved off root-drive `C:/taichi_cache`
  (permission walls on locked-down machines) to
  `%LOCALAPPDATA%\BeetleBattle\taichi_cache` via `offline_cache_file_path`,
  with home-dir / Taichi-default fallback. Existing warm files migrated.

## PHASE 1 — Comprehensive front-loaded warmup ★ the real fix (~1 day)
Compile EVERYTHING behind the loading bar so nothing compiles during play.
The game already warms a subset (Phase 0-6 in beetle_physics.py — extract,
particles, spray, assembly, etc.); extend it to cover the gaps.

To enumerate and drive offscreen (place at y=-100 / offscreen coords, one call
each is enough to trigger compilation + cache write):
- **Every arena mode**: circle, ball, donut, x_stage, barbell, figure8,
  yinyang, square_bridge, square, cut_square, squiggle, hourglass — each has
  its own floor-height + edge-tipping + collision kernel specialization. Build
  each arena's floor cache + run one edge-tipping/floor-collision pass.
- **Every beetle horn type contact**: rhino/stag/hercules/scorpion/atlas/
  bombardier/spider/giraffe — run beetle_collision + horn-distance kernels for
  representative type pairs (at least each type vs itself + vs rhino) so the
  collision/horn-segment kernel variants compile.
- **Ball mode kernels**: ball-beetle collision, ball floor bounce, squash
  extract path, bounce dust — one offscreen ball contact + bounce.
- **Per-slot factories for all 4 slots** (place/transform/assembly/lowest-
  point/silk) — already warmed for slots in use; ensure 2-3 warmed too.
- **All effects**: death disintegration/explosion, spray, venom, silk,
  confetti, tornado/sandstorm/UFO/comet/ice/hole/board-break hazard spawns,
  arena-transition rings, downwash — one spawn each offscreen.
- Wrap in a progress-bar phase; kernel-arg TYPE consistency matters (wrap
  .to_numpy()/field reads with float() — see MEMORY taichi gotcha — or the
  first REAL call recompiles despite warmup).

Result: first launch = one honest load (progress bar) that also writes the
full cache; every launch after = fast from disk; zero in-game spikes on any
machine.
Risk/verify: measure first-load wall time after; if too long, gate the
heaviest warmups behind "first run only" (skip when cache already warm).

## PHASE 2 — Ship a pre-warmed cache (accelerant, ~half day, after P1)
Pre-compile on a build machine, bundle the `.tic` files with the game, seed
them into `%LOCALAPPDATA%\BeetleBattle\taichi_cache` on first run if empty.
Makes even the FIRST load fast for machines whose CPU features match. Graceful
recompile (via P1 warmup) for the rest. Layer on top of P1, never instead of.
Caveat: verify portability across a few different CPUs before relying on it;
keep P1 as the guarantee.

## OUT OF SCOPE
- **AOT compilation** (Taichi's no-JIT deploy path): correct in theory but a
  major refactor — AOT forbids the Python-driven dynamic kernel generation
  this codebase is built on. Not worth it unless P1+P2 prove insufficient.

## Order
relocation (done) → P1 comprehensive warmup → measure first-load → decide on
P2. Stale-lock guard already protects all of it from crash-bricking.
