# Beetle-vs-Beetle Combat Consistency Plan

Written 2026-07-15, prompted by the atlas static-decoy find (pronotum
chords stole strike credit from the moving cephalic horn — the ball-side
fix also repaired the beetle-side shaft path). Goal: find every place two
beetles in the SAME physical situation get DIFFERENT outcomes because of
representation gaps, not because of intended type stats.
PARKED BEHIND NOTHING — this runs next; ball_floor_beetle_consistency.md
(floor/rim phases) is queued AFTER this per user call.

## What counts as an inconsistency (the atlas test)

INTENDED differences: per-type stats (turn/yaw/tilt speeds), TIP_FACTOR
weights, horn shapes themselves. NOT bugs.
INCONSISTENCY: same maneuver, different physics because a type's geometry
or motion is missing/stale/misattributed in some layer — e.g. atlas
pitch-strike reading dead because a static chord won the contact.

## Audit dimensions x known state

### D1. Motion-credit parity (who transfers momentum when moving)
- [x] Shaft path (horn-vs-body): pitch/yaw via credit helper (static-decoy
      fixed for ball AND beetle intruders); channels bombardier/scorpion/
      spider all credited.
- [x] SVS (horn-vs-horn crossings): pitch/yaw + scorpion tail + spider
      abdomen credited.
- [ ] CONFIRMED GAP: bombardier AIM credit is missing in SVS — aim-tilting
      the head into a horn crossing transfers nothing (works in the shaft
      path). Small block, mirrors the scorpion/spider loop entries.
- [ ] VERIFY: main impulse's artic_tip_speed credit (redirection) is
      pitch/yaw-only and reach-15 hardcoded — channels (tail/aim) get no
      credit there; horn_reach 15 is wrong for short-horned types.

### D2. Contact-damping parity (what slows a weapon grinding through you)
- [x] Horn pitch/yaw: horn damping via calculate_horn_damping.
- [x] Bombardier aim: burial damping (step 6 of the abnormal-body plan).
- [ ] CONFIRMED GAP: scorpion TAIL has no contact damping — V-strike
      grinds through an opponent's horn/body at full 50 deg/s while every
      other weapon slows under burial. Mirror the bombardier aim-damping
      pattern (horn_burial scale on tail rotation speed).
- [ ] CONFIRMED GAP: spider ABDOMEN aim likewise undamped (lower stakes —
      the abdomen aims up/away from opponents; verify in play first).

### D3. Predictive-layer staleness (the tip-vs-tip pre-push)
- [ ] CONFIRMED: the predictive check (beetle_collision head) uses the
      OLD single-tip functions — rhino midline tip for multi-arm types,
      bombardier fixed (10,5) that ignores aim, scorpion the RETIRED
      center-chord tip that points at the GAP between the claws. The
      response layers moved to multi-arm skeletons; the predictive layer
      never did. Options: (a) predictive check per segment-tip pair from
      horn_collision_segments (costlier: NxM tips), (b) accept and
      document (it is only a gentle pre-push; the shaft/svs layers do the
      real work). AUDIT: measure how often it fires per type (add a
      per-type counter to collision_stats for one canary) before deciding.

### D4. Per-slot parity (P3/P4 vs blue/red)
- [ ] CONFIRMED (memory, unfixed): the thin-horn XZ NEIGHBOR RESCUE in
      _column_pair_contact hardcodes blue/red voxel id lists — DEAD for
      slots 2/3, so P3/P4 thin horns clip through gaps blue/red would
      catch. Fix: ownership via simulation.beetle_owner like the main
      column test (also un-hardcodes future palettes).
- [ ] VERIFY: any other blue/red-only code on the combat path (grep
      BEETLE_BLUE|BEETLE_RED literals inside collision functions; the
      old giraffe color==string class of bug).

### D5. Geometry parity in bvb layers
- [x] Segments: all 8 types have multi-arm/channel-tracked skeletons
      (this month's work): stag/hercules/rhino arms, atlas cephalic+
      pronotum, giraffe chain, scorpion claws+tail, bombardier chord+head
      (aim-tracked), spider chord+abdomen (aim-tracked).
- [ ] VERIFY: calculate_horn_damping and the yaw-lift block — do they use
      contact position only (fine) or single-tip geometry (stale for
      multi-arm)? Trace and note.
- [ ] KNOWN ACCEPTED (re-confirm still acceptable): svs tip-band dead
      zone (s/t > 0.7 with tips touching); fast turn+yaw tunneling
      (Phase-3 revert, 2026-07-12).
- [ ] VERIFY: scorpion fully-raised tail tip exits the +22 grounded
      column scan (~row 23) — bvb tail-tip contact at full raise may be
      undetected. Tiny band; check with the tail-strike damping work
      since both touch the tail.

## Execution order

1. AUDIT PASS (no code): trace D1-verify, D4-verify, D5-verify items;
   one canary WITH per-type predictive counters (D3) if cheap to add.
   Output: update this doc's checkboxes to CONFIRMED/CLEAR.
2. FIX 1 — SVS bombardier aim credit (D1). GATE: aim-tilt the head into
   a locked horn cross -> opponent's horn gets pushed; canary optional
   (svs credit only, metrics stable).
3. FIX 2 — scorpion tail contact damping (D2): horn_burial-scaled tail
   speed, same constants as the aim damping (1.5 free / 5.5 stop).
   GATE: V-strike INTO an opponent's horn -> tail visibly slows and
   shoves instead of ghost-grinding; free-air strikes unchanged; ball
   smash unchanged (burial only rises on beetle contact).
4. FIX 3 — per-slot neighbor rescue (D4): ownership-based, all slots.
   CANARY REQUIRED (this changes detection for everyone — deep_clip
   comparable, horn_cross rebaselines... detection tweaks rebaseline
   BOTH; record fresh baselines). GATE: 4P bot match, watch P3/P4 thin
   horn (giraffe) clipping vs blue/red.
5. FIX 4+ — whatever the audit pass confirms from the VERIFY items,
   ordered by user-visible impact; D3 decision (per-segment predictive
   vs document-and-accept) taken on the counter data.

Each fix = its own commit on the user's "commit". User playtests every
gate; canaries only where marked, on request.
