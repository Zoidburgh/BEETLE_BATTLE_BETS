# BACKGROUND EFFECTS — Art & Optimization Plan

## STATUS 2026-07-08 EVENING: SHIPPED + USER-VALIDATED "general concept gonna work"
(commits 7fd68a5 + be7e8fa on arena_mods, NOT yet pushed as of writing)

### What shipped (steps 1, 2, 4 of the plan below)
- `bg_mute`/`bg_fog` per-voxel fields + `THEME_TREATMENT` table (simulation.py
  near the theme IDs; stamped onto each theme's voxel range in toggle_theme;
  compact_background_voxels + clear_background + clear_all_themes all carry
  them — if you add a bg field, update ALL FOUR places)
- `update_bg_cache(cam_x/y/z, sky_r/g/b, fog_start, fog_end, fog_max,
  mute_strength)` (simulation.py): presence desat/dim, then smoothstep fog
  toward sky color, radius taper (1 - 0.35*f), f>0.97 voxels culled before
  claiming a cache slot. TWO call sites in beetle_physics.py (warmup +
  main loop) — args wrapped in float() (Taichi recompile gotcha).
- Sliders in BACKGROUND panel under --- ATMOSPHERE ---: BIOME MUTE (0-2.0),
  FOG START (20-250), FOG END (60-400), FOG MAX (0-1). Globals
  BG_MUTE_STRENGTH / BG_FOG_START / BG_FOG_END / BG_FOG_MAX + THEME_SKY_COLORS
  + DEFAULT_SKY_COLOR live near window.background_color init in
  beetle_physics.py.
- Biome toggles set their sky color; CLEAR ALL / biome-off restores default.

### Lessons from user feedback (do not re-introduce)
- **NEVER scale animation offsets for muting.** First version multiplied
  bg_offset_* by presence to "calm motion" — but for traveling critters
  (scorpion/toad/dolphin/pterodactyl) the offset IS the patrol path: orbits
  shrank and splashes desynced when the slider moved. Presence is COLOR
  treatment only (desaturate + dim). Verified: positions bit-identical
  across mute 0..2.
- Fog's radius taper (far voxels shrink up to ~35%) is INTENDED depth cue,
  but the user noticed sizes changing with sliders — if it ever reads as
  wrong rather than as depth, the 0.35 constant in update_bg_cache is the
  one knob (soften or zero it).

### Verification methods that work (reuse these)
- Headless check (no window): ti.init(cpu), import simulation, toggle_theme,
  animate_background(1.0), update_bg_cache(...), read num_visible_bg +
  bg_cache_* .to_numpy(). IMPORTANT: cache order is nondeterministic
  (parallel atomic_add) — always lexsort by position before comparing runs.
- Stars contract check: run with (fog_max=0, mute=0) vs defaults, sorted
  color diff must be 0.0 (stars have bg_fog=0 so fog never touches them).
- --local4 --perfauto canary: 'background' perf bucket in perf_log.txt
  (~1.0ms for animate + cache + fog at 743 star voxels; frame 17.1ms).

### REMAINING WORK
1. USER FEEL-TUNE (next session): walk each biome (desert/grass/ocean/
   swamp/lava) with the four sliders; adjust per-theme values in
   THEME_TREATMENT (mute, fog participation) and THEME_SKY_COLORS to taste.
   Current defaults: biomes 0.7 presence, extras 1.0; stars fog 0.0,
   comet 0.15, clouds 0.5, ptero 0.6, fireflies/butterflies 0.8,
   terrestrial 1.0.
2. Step 3 below (twinkle spike taming) — DEFERRED, stars approved as-is.
3. If a biome still competes after tuning: consider per-theme brightness
   floor (the 0.6 in the presence dim `(0.6 + 0.4*m)`) or stronger
   desaturation curve.
4. Slider persistence across sessions (currently reset to defaults each
   boot) — only if the user asks.

Research done 2026-07-08. Problem statement (user): background themes look
cool but their voxels are DISTRACTING next to the beetle voxels — want a
shader-like treatment / distance fading so far things read as far, and a
general look-better + run-better pass.

## THE CORE FINDING: we can fake fog perfectly, and it solves the distraction

Taichi GGUI (1.7.4) has **no shaders, no fog API, no alpha blending, no
post-processing** — the pipeline is fixed and opaque. BUT:

1. Every background voxel's final color is computed per-frame in ONE kernel we
   own: `update_bg_cache()` (simulation.py:3965-4014). Beetles/arena/debris do
   NOT pass through it — it is a background-only color choke point.
2. The sky is a FLAT clear color `(0.04, 0.04, 0.06)` (no skybox, no gradient).
   **Lerping a voxel's color toward the clear color is visually identical to
   alpha-fading it against the sky.** Fade-to-sky IS our alpha channel.
3. `scene.particles` supports `per_vertex_radius` (already used) — far voxels
   can also SHRINK, reinforcing depth.

So: distance fog = per-voxel `mix(color, sky_color, fog(dist_to_camera))` baked
in `update_bg_cache`, plus a radius taper. ~8000 iterations of cheap math in a
kernel that already runs every frame. No new draw calls, zero upload cost
change, beetles untouched.

## WHY THE BACKGROUND FIGHTS THE BEETLES TODAY (art analysis)

- **Same visual identity**: bg voxels are the SAME lit spheres in the SAME
  `scene.particles` call as beetles — same material, same lighting, same
  crispness. The eye gets no layer separation.
- **No atmospheric perspective**: a comet voxel at 220 units renders with the
  same contrast/saturation as a beetle voxel at 40. Every art tradition
  (and every 3D game) mutes distant things toward the sky color; we do nothing.
- **Contrast inversion**: sky is near-black, so bright bg voxels (stars at 1.0
  brightness, shooting stars at 2.2x overdrive, white cloud tops 0.88-1.0) have
  MORE contrast against their backdrop than beetles do against the warm floor.
  The background literally pops harder than the subject.
- **Motion magnetism**: peripheral vision is motion-sensitive. Twinkle flashes
  with size "starburst" boosts (simulation.py:3986-4001), shooting-star
  overdrives, wave/fish/squid splashes — high-frequency brightness+size spikes
  at full contrast constantly yank the eye off the fight.
- **Scale confusion**: beetle voxels are the SMALLEST radius in the scene
  (0.407) while clouds go to 2.8 and crowd grubs 1.3. Without depth cues, a
  big far sphere and a near small one are the same screen blob.

## USER CLARIFICATION (2026-07-08): the default look (STARS) is GREAT — don't touch it.
The competition problem is the BIOMES: swamp / desert / ocean / lava / grass.
Diagnosis: stars accidentally follow good art rules (tiny voxels, 55-220 units
away, gentle motion, dark sky) — biomes break them: ground-level decor starts
just past the arena edge (~35-55 units, nearly beetle distance), with BIG
voxels (lava crust 2.5-3.0 vs beetle 0.407), full saturation, and constant
eye-level splash/sway animation. **Distance fog alone will not fix biomes —
they're too close for fog. They need a per-theme PRESENCE cut (saturation /
brightness / animation amplitude), while fog handles their farther shells.**

Tuning contract: all defaults chosen so the STARS+COMET look is pixel-
identical (or imperceptibly changed); biome mute defaults ~0.7 presence.
Mechanism: add a per-voxel `bg_mute` f32 field (shape 8000) written at
generation time per theme (theme_start_idx/theme_count maps already exist in
simulation.py); `update_bg_cache` applies it to saturation + brightness +
anim spike amplitude, scaled by a global BIOME MUTE slider. Extras
(stars/comet/fireflies/butterflies/palms/clouds/ptero) default mute 1.0.

## THE PLAN (ordered; each step independently shippable)

### 1. Distance fog + radius taper in `update_bg_cache` — THE BIG ONE
Add args (floats — wrap with float(), Taichi recompile gotcha): camera pos
(cam_x/y/z), sky color (sky_r/g/b), FOG_START, FOG_END, FOG_MAX.
Per visible voxel:
```
d   = distance(world_pos, cam)
f   = smoothstep(FOG_START, FOG_END, d) * FOG_MAX
col = mix(col, sky, f)
rad = rad * (1.0 - 0.35 * f)
if f > 0.97: skip voxel entirely (free culling — fewer spheres drawn)
```
Camera pos is available in the main loop where update_bg_cache is called
(beetle_physics.py:22031-22045); renderer.Camera has pos_x/y/z. Default camera
sits ~107 units from arena center; bg voxels live at ~55-220. Starting values:
FOG_START 90, FOG_END 230, FOG_MAX 0.85 (leave a whisper of the far stuff;
1.0 = vanish). **Expose all three as sliders in the BACKGROUND GUI panel** —
this game tunes by feel.

### 2. Palette separation — per-theme presence (THE BIOME FIX)
Same kernel, before the fog mix, driven by the per-voxel `bg_mute` field
(see clarification above) times a global BIOME MUTE slider:
```
m    = bg_mute[i] * BIOME_MUTE                   # 1.0 for extras, ~0.7 biomes
luma = dot(col, (0.299, 0.587, 0.114))
col  = mix(vec3(luma), col, m)                   # desaturate muted themes
col *= mix(BG_BRIGHTNESS_FLOOR, 1.0, m)          # dim them (floor ~0.6)
```
Rule of art direction: the brightest, most saturated things on screen should
be beetle stripes/horn tips and score moments — never scenery. Biome decor is
stage dressing, not cast. Also multiply animation OFFSET amplitude by m for
the near-arena splash/sway anims (waves, mud splash, sway) so muted themes
move less violently — motion draws the eye more than color does.
Optionally scale saturation DOWN with distance too (near bg keeps color, far
goes hazy-gray) — classic aerial perspective, one multiply.

### 3. Tame the attention spikes
- Twinkle starburst size boost + flash brightness: scale the spike component
  by (1 - f) so near-ish stars still sparkle but far ones shimmer subtly.
- Shooting star 2.2x overdrive: keep (it's a designed set piece) but let fog
  apply — it'll read as a distant meteor instead of a flare in your face.
- Splash-type anims (wave/fish/squid/mud): they live at fixed radii; fog
  handles them once step 1 ships.

### 4. Per-theme sky + fog color (sells each biome, ~free)
Theme toggle sets a recommended `window.background_color` (user's color picker
still overrides): stars = near-black blue (current), desert = warm dusty
`(0.10,0.07,0.05)`, swamp = murky green-black, lava = deep red-black, ocean =
deep blue. Fog color = sky color automatically (that's what makes the fade
read as air). One small dict + assignment at toggle time.

### 5. Perf pass (measure FIRST — Nov 2025 lesson: no blind trimming)
- Add a perf-log bucket around `animate_background` + `update_bg_cache` +
  extract PHASE 6 (renderer.py:1431-1439). Costs currently UNMEASURED.
- If the giant `animate_background` switch kernel shows up: BG_ANIM_FREQUENCY
  already exists (simulation.py:103, =1). Ambient anim at every-2nd-frame is
  visually free and halves the cost — but measure before/after.
- Upload cost is CONSTANT (GGUI uploads full 16000-capacity buffer regardless
  of count — see taichi-ggui-buffer-cost memory), so voxel-count trims do NOT
  help upload; they help extract time + fragment cost. The fog cull (step 1)
  is the honest win: fully-fogged voxels never enter the render buffer.
- Static decor (pyramids, cacti, lava crust, BG_ANIM_NONE) is re-cached every
  frame; only bother splitting static/dynamic if the bucket says it matters.

### 6. NOT recommended
- Re-enabling the gradient background: removed for a reason (~15 FPS canvas
  cost). Per-theme flat sky + fog gives more depth for free.
- True transparency: GGUI VBO carries alpha only if you feed vec4 colors, and
  the shader ignores it (opaque pipeline). Dead end until the Vulkan rewrite
  (GPU_CPU_OPT_PLAN.md).
- Moving bg to a second particles call for separate treatment: costs a draw
  call and buys nothing per-vertex color can't do.

## KEY CODE MAP (from research agents)
- BG fields (8000 cap `MAX_BACKGROUND_VOXELS`): bg_positions/colors/size/
  anim_type/brightness/offset_* + bg_cache_* (simulation.py; docs at
  beetle_physics.py:231-298)
- `animate_background(t)` giant per-type switch: simulation.py:1343+
- `update_bg_cache()` (THE HOOK): simulation.py:3965-4014
- Main-loop call site (has camera + frame gate BG_ANIM_FREQUENCY):
  beetle_physics.py:22031-22045
- Render: extract_particles PHASE 6 renderer.py:1431-1439 → single
  scene.particles renderer.py:1642-1648 (per_vertex_color + per_vertex_radius)
- Sky color: window.background_color (0.04,0.04,0.06), set every frame
  beetle_physics.py:22029; user picker beetle_physics.py:24099-24121
- GUI BACKGROUND panel (add sliders here): beetle_physics.py:24131-24199
- Themes: 16 (STARS default-on at boot beetle_physics.py:17250; biomes
  DESERT/GRASS/OCEAN/SWAMP/LAVA mutually exclusive; extras stackable;
  WATER/JELLYFISH/STADIUM/RAIN exist but hidden from GUI)
- Crowd = STADIUM theme grubs at r=70 w/ excitement system simulation.py:4736+
- Lights: 2-3 point lights + ambient renderer.py:1603-1631 (each extra
  point_light ~15 FPS on iGPUs — don't add lights for mood, use color)

## GOTCHAS FOR THE IMPLEMENTER
- Wrap ALL new kernel args with float() at call sites (JIT recompile spike).
- Declare kernel branch vars before conditionals (Taichi scoping).
- Don't grow any render/bg field capacities — GGUI uploads full capacity
  every frame (~2.5ms/MB).
- Verify with --local4 --perfauto canary + eyeball each theme; sliders make
  fog tuning a live activity with the user.
