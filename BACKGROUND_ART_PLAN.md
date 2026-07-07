# BACKGROUND EFFECTS — Art & Optimization Plan

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

### 2. Palette separation — reserve saturation for gameplay
Same kernel, before the fog mix:
```
luma = dot(col, (0.299, 0.587, 0.114))
col  = mix(vec3(luma), col, BG_SATURATION)      # slider, start ~0.65
col *= min(1.0, BG_BRIGHTNESS_CAP / max_channel) # cap peaks, start ~0.85
```
Rule of art direction: the brightest, most saturated things on screen should
be beetle stripes/horn tips and score moments — never scenery. Optionally
scale saturation DOWN with distance (near bg keeps color, far goes hazy-gray)
— that's classic aerial perspective and one multiply.

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
