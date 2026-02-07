# Beetle Battle Development Journal

## 2026-01-08 - Steam Setup & Networking Fixes

### What We Did

**Steam Distribution Setup**
- Set up SteamCMD for uploading builds
- Created `BUILD_AND_UPLOAD.bat` - builds game + uploads to Steam
- Created `UPLOAD_ONLY.bat` - quick upload without rebuild
- Created `UPLOAD_PLAYTEST.bat` - uploads to Playtest app
- App ID: 3998620, Depot ID: 3998621
- Playtest App ID: 4318480, Depot ID: 4318481
- Updated `steam_appid.txt` to use real App ID (was 480/Spacewar)

**Build Uploads**
- Successfully uploaded builds to both main app and Playtest app
- Builds set live on default branches
- Configured launch options (BeetleBattle.exe)

**Performance Fixes**
- Changed vsync to OFF by default (was causing 20-25 FPS lock on some systems)
- Added `--vsync` flag to enable if needed
- Added GPU diagnostic info at startup (detects NVIDIA Optimus issues)

**Networking Improvements**
- Added Steam friend auto-join support (`+connect_lobby` command line arg)
- Changed lobby type to "friends" so Steam friends can see "Join Game"
- Simplified disconnect handling:
  - Detects when opponent stops responding (3 seconds)
  - Shows "OPPONENT NOT RESPONDING" message
  - "Return to Menu" button to cleanly exit and rejoin
  - No complex reconnection - just return to menu and host/join again

**Bug Fixes**
- Fixed `dt` → `frame_dt` variable name error in disconnect code
- Disabled complex reconnection code that was causing crashes
- Fixed orphaned code blocks after commenting out reconnect system

---

### Still TODO

**Steam Playtest Setup**
- [ ] Upload capsule images to Playtest app (Library Capsule, Community Capsule)
- [ ] Submit Playtest for lightweight review
- [ ] Once approved, "Playtest Settings" should appear to enable signups
- [ ] Alternative: Request Release State Override Keys for Playtest app (up to 50k keys)

**Steam Store Page**
- [ ] Add basic store presence info (name, description, images)
- [ ] Verify "public default branch includes game.exe" checkbox gets checked
- [ ] Set up store page for eventual release

**Networking Polish (Future)**
- [ ] Re-implement proper disconnect/reconnect system (currently simplified)
- [ ] Test Steam friend "Join Game" functionality
- [ ] Investigate why some players get low FPS (CPU single-thread bottleneck)

**Performance**
- [ ] Friend with RTX 3070 Ti still getting ~25-30 FPS - likely CPU bottleneck
- [ ] scene_particles taking 8-50ms depending on system
- [ ] Consider optimization if needed for low-end systems

---

### Files Changed Today
- `beetle_physics.py` - vsync default, disconnect handling, Steam auto-join
- `simulation.py` - GPU diagnostic info
- `steam_appid.txt` - Changed to real App ID 3998620
- `steamcmd/app_build_3998620.vdf` - Main app upload config
- `steamcmd/app_build_playtest.vdf` - Playtest upload config
- `BUILD_AND_UPLOAD.bat` - Build + upload script
- `UPLOAD_ONLY.bat` - Quick upload script
- `UPLOAD_PLAYTEST.bat` - Playtest upload script
- `STEAM_SETUP.md` - Setup instructions

---

### Quick Reference

**Upload new build:**
```
BUILD_AND_UPLOAD.bat   # Full rebuild + upload
UPLOAD_ONLY.bat        # Just upload existing build
UPLOAD_PLAYTEST.bat    # Upload to Playtest app
```

**Test with friends (no Steam):**
- Send `dist/BeetleBattle_Easy.zip`
- They run INSTALL.bat, then PLAY.bat

**Resolution flags:**
```
--res 720    # 720p (best performance)
--res 1080   # 1080p (default)
--vsync      # Enable vsync (off by default now)
```
