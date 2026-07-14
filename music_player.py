"""Standalone music player subprocess for Beetle Battle.

The game process runs with SDL_AUDIODRIVER=dummy because a real audio device
open in the same process as Taichi causes memory corruption on audio
device-change events (window focus changes, volume keys, Bluetooth
renegotiation) — silent 0xc0000005 crashes faulting in taichi_python.
Reproduced 2026-07-13 via focus-steal testing: real drivers (WASAPI and
DirectSound both) died, dummy survived.

This windowless process owns the real audio device instead. It receives no
window-focus events, and if the audio stack ever does take it down, the game
keeps running (worst case: the music stops).

Usage: python music_player.py <track_path> <volume 0..1>
Exits on its own if the parent dies (stdin closes) or the track path is gone.
"""
import sys
import time

def main():
    if len(sys.argv) < 2:
        return
    track = sys.argv[1]
    volume = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    import pygame
    pygame.mixer.pre_init(44100, -16, 2, 2048)
    pygame.mixer.init()
    pygame.mixer.music.load(track)
    pygame.mixer.music.set_volume(volume)
    pygame.mixer.music.play(-1)  # Loop forever
    # Park until the parent game process exits: the game holds our stdin pipe,
    # so EOF on stdin = parent is gone -> stop
    try:
        sys.stdin.read()
    except Exception:
        pass

if __name__ == "__main__":
    main()
