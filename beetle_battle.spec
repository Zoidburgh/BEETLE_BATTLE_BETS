# -*- mode: python ; coding: utf-8 -*-
# PyInstaller spec file for Beetle Battle

import sys
from PyInstaller.utils.hooks import collect_all, collect_submodules

block_cipher = None

# Collect all Taichi data files and binaries
taichi_datas, taichi_binaries, taichi_hiddenimports = collect_all('taichi')

# Collect py_steam_net
steam_datas, steam_binaries, steam_hiddenimports = collect_all('py_steam_net')

# Additional hidden imports for Taichi
hiddenimports = [
    'taichi',
    'taichi.lang',
    'taichi._lib',
    'taichi._lib.core',
    'taichi.ui',
    'taichi.ui.gui',
    'numpy',
    'py_steam_net',
] + taichi_hiddenimports + steam_hiddenimports

a = Analysis(
    ['beetle_physics.py'],
    pathex=[],
    binaries=taichi_binaries + steam_binaries,
    datas=[
        ('steam_api64.dll', '.'),
        ('steam_appid.txt', '.'),
        ('network.py', '.'),
        ('renderer.py', '.'),
        ('simulation.py', '.'),
    ] + taichi_datas + steam_datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='BeetleBattle',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=True,  # Keep console for debugging, can set to False later
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='BeetleBattle',
)
