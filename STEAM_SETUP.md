# Steam Upload Setup

## One-Time Setup

### Step 1: Download SteamCMD
1. Go to: https://developer.valvesoftware.com/wiki/SteamCMD#Downloading_SteamCMD
2. Download the Windows version (steamcmd.zip)
3. Extract `steamcmd.exe` to the `steamcmd` folder in this project

### Step 2: First Run of SteamCMD
1. Double-click `steamcmd\steamcmd.exe`
2. It will auto-update itself (takes a minute)
3. When you see `Steam>` prompt, type `quit` and press Enter
4. Done - SteamCMD is ready

### Step 3: Set Your Steam Username (Optional)
For convenience, set environment variable:
```
setx STEAM_USERNAME "your_steam_username"
```
Or you'll be prompted each time.

---

## Uploading a Build

### Option A: Build + Upload (Full)
```
BUILD_AND_UPLOAD.bat
```
- Runs PyInstaller to create fresh build
- Uploads to Steam
- Takes ~30 seconds

### Option B: Upload Only (Quick)
```
UPLOAD_ONLY.bat
```
- Uploads existing build in `dist\BeetleBattle`
- Use when you already built manually
- Takes ~10 seconds

---

## After Upload

1. Go to https://partner.steamgames.com
2. Open your app (3998620)
3. Go to **SteamPipe** → **Builds**
4. Find your new build
5. Click **Set Live** on the `default` branch

Now anyone with access can download the update!

---

## Letting Testers Access

### Option 1: Steam Playtest (Recommended)
1. In Steamworks, go to **Store Presence** → **Playtest**
2. Enable Playtest
3. Anyone can request access via your store page

### Option 2: Beta Keys
1. Go to **Marketing & Visibility** → **Beta Testing**
2. Generate keys
3. Send to testers

### Option 3: Add Specific Users
1. Go to **Users & Permissions**
2. Add their Steam IDs with playtest access

---

## App Info
- App ID: 3998620
- Depot ID: 3998621
