# Controller Support Implementation Plan

## Overview

Add controller support for Steam Deck and standard gamepads while maintaining existing keyboard controls.

## Controller Mapping (Per Player)

| Action | Controller Input | Notes |
|--------|------------------|-------|
| Forward | Left Stick Y ↑ | Push up to move forward |
| Backward | Left Stick Y ↓ | Push down to move backward |
| Turn Left | Right Stick X ← | Push left to turn left |
| Turn Right | Right Stick X → | Push right to turn right |
| Horn Up | RT (Right Trigger) | Pitch up |
| Horn Down | LT (Left Trigger) | Pitch down |
| Horn Left | LB (Left Bumper) | Yaw left |
| Horn Right | RB (Right Bumper) | Yaw right |

**Design rationale:**
- Thumbs stay on sticks at all times
- Triggers handle vertical (pitch) - spatial: pull down
- Bumpers handle horizontal (yaw) - spatial: left/right sides
- Tank controls: left stick = throttle, right stick = steering

## Input Scenarios

| Mode | Controllers | Blue Beetle | Red Beetle |
|------|-------------|-------------|------------|
| Local testing | 0 | Keyboard (TFGH) | Keyboard (IJKL) |
| Local testing | 1 | Controller 1 | Keyboard (IJKL) |
| Local 2-player | 2 | Controller 1 | Controller 2 |
| Network mode | 0 | Keyboard (WASD+Arrows) | Remote player |
| Network mode | 1 | Controller 1 | Remote player |

**Rules:**
- 1 controller = 1 beetle (never control both)
- Controller + keyboard inputs are combined (OR'd together)
- Keyboard always works as fallback
- Network mode: only local beetle checks for controller

## Technical Implementation

### Library
Use `pygame._sdl2.controller` (built into pygame 2.0+):
- Handles Steam Deck natively
- Automatic button mapping for Xbox/PS/generic controllers
- No additional dependencies

### Critical: Import Order and SDL Setup
```python
# MUST be set BEFORE importing pygame - at very top of file after standard imports
import os
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')  # No window, controller-only

import pygame
try:
    import pygame._sdl2.controller as sdl_controller
    CONTROLLER_SUPPORT = True
except ImportError:
    CONTROLLER_SUPPORT = False
```

**Why dummy driver?** Game uses Taichi GGUI for rendering. pygame is only for controller input. Dummy driver prevents pygame from trying to create a display.

### Controller Detection (startup only, event-based hot-plug)
```python
# Global state
controllers = []  # List of connected Controller objects

def init_controllers():
    """Call once at startup."""
    if not CONTROLLER_SUPPORT:
        return
    pygame.init()
    sdl_controller.init()
    _refresh_controllers()

def _refresh_controllers():
    """Rebuild controller list. Called on startup and device events."""
    global controllers
    # Close old controllers
    for ctrl in controllers:
        try:
            ctrl.quit()
        except:
            pass
    controllers = []
    for i in range(sdl_controller.get_count()):
        try:
            ctrl = sdl_controller.Controller(i)
            controllers.append(ctrl)
            print(f"[Controller] Found: {ctrl.name} (player {len(controllers)})")
        except:
            pass
```

### Performance: Event Pump (once per frame, minimal overhead)
```python
def pump_controller_events():
    """Call once per frame in main loop. Handles hot-plug via events."""
    if not CONTROLLER_SUPPORT:
        return

    for event in pygame.event.get():
        # Hot-plug: rebuild controller list only when devices change
        if event.type == pygame.CONTROLLERDEVICEADDED:
            print("[Controller] Device connected")
            _refresh_controllers()
        elif event.type == pygame.CONTROLLERDEVICEREMOVED:
            print("[Controller] Device disconnected")
            _refresh_controllers()
    # Note: pygame.event.get() is fast (~0.01ms) when no events queued
```

**Performance note:** `pygame.event.get()` just drains a queue - no polling or scanning. Hot-plug detection only triggers `_refresh_controllers()` on actual device change events, not every frame.

### Controller Input Function
```python
STICK_DEADZONE = 0.2
TRIGGER_THRESHOLD = 0.3

def get_controller_inputs(ctrl):
    """Read controller state and return input bitmask (same format as keyboard)."""
    if ctrl is None:
        return 0

    try:
        inputs = 0

        # Left stick Y-axis: Forward/Backward
        # Range is -32768 to 32767, divide by 32767.0 for proper -1.0 to +1.0
        left_y = ctrl.get_axis(sdl_controller.CONTROLLER_AXIS_LEFTY) / 32767.0
        if left_y < -STICK_DEADZONE:  # Up = negative
            inputs |= INPUT_FORWARD
        elif left_y > STICK_DEADZONE:  # Down = positive
            inputs |= INPUT_BACKWARD

        # Right stick X-axis: Turn Left/Right
        right_x = ctrl.get_axis(sdl_controller.CONTROLLER_AXIS_RIGHTX) / 32767.0
        if right_x < -STICK_DEADZONE:
            inputs |= INPUT_LEFT
        elif right_x > STICK_DEADZONE:
            inputs |= INPUT_RIGHT

        # Triggers: Horn Up/Down (axis values 0 to 32767)
        left_trigger = ctrl.get_axis(sdl_controller.CONTROLLER_AXIS_TRIGGERLEFT) / 32767.0
        right_trigger = ctrl.get_axis(sdl_controller.CONTROLLER_AXIS_TRIGGERRIGHT) / 32767.0
        if left_trigger > TRIGGER_THRESHOLD:
            inputs |= INPUT_HORN_DOWN
        if right_trigger > TRIGGER_THRESHOLD:
            inputs |= INPUT_HORN_UP

        # Bumpers: Horn Left/Right
        if ctrl.get_button(sdl_controller.CONTROLLER_BUTTON_LEFTSHOULDER):
            inputs |= INPUT_HORN_LEFT
        if ctrl.get_button(sdl_controller.CONTROLLER_BUTTON_RIGHTSHOULDER):
            inputs |= INPUT_HORN_RIGHT

        return inputs
    except:
        # Controller disconnected mid-read
        return 0
```

### Modified get_local_inputs()
```python
def get_local_inputs(window, player='blue', network_mode=False, horn_type_id=0):
    """Get inputs from controller (if available) combined with keyboard."""

    # Get keyboard inputs first (always works)
    keyboard_inputs = _get_keyboard_inputs(window, player, network_mode, horn_type_id)

    # Add controller inputs if available
    controller_inputs = 0
    if CONTROLLER_SUPPORT and controllers:
        if network_mode:
            # Network mode: first controller controls local beetle
            controller_inputs = get_controller_inputs(controllers[0])
        elif player == 'blue':
            controller_inputs = get_controller_inputs(controllers[0])
        elif player == 'red' and len(controllers) >= 2:
            controller_inputs = get_controller_inputs(controllers[1])

    # Combine both input sources (allows hybrid play)
    return keyboard_inputs | controller_inputs
```

### Main Loop Integration
```python
# In main game loop, call once per frame BEFORE reading inputs:
pump_controller_events()

# Then get inputs as normal:
blue_inputs = get_local_inputs(window, 'blue', network_mode, blue_horn_type)
red_inputs = get_local_inputs(window, 'red', network_mode, red_horn_type)
```

## File Changes Required

1. **beetle_physics.py**
   - Add pygame/SDL imports at TOP of file (before any other code)
   - Add `CONTROLLER_SUPPORT` flag
   - Add controller globals and init function
   - Add `pump_controller_events()` function
   - Add `get_controller_inputs()` function
   - Rename current `get_local_inputs()` to `_get_keyboard_inputs()`
   - Add new `get_local_inputs()` that combines both
   - Call `init_controllers()` at startup
   - Call `pump_controller_events()` once per frame in main loop

2. **No other files need changes**
   - Input bitmask format stays the same
   - Network code unchanged
   - Physics code unchanged

## Performance Impact

| Operation | Frequency | Cost | Notes |
|-----------|-----------|------|-------|
| `pygame.event.get()` | Once/frame | ~0.01ms | Just drains event queue |
| `get_controller_inputs()` | 1-2x/frame | ~0.02ms | Direct memory reads |
| `_refresh_controllers()` | On device change | ~1ms | Only on plug/unplug |
| SDL dummy driver | Startup | 0 | No display overhead |

**Total overhead: ~0.03ms per frame** (negligible vs 16.6ms frame budget at 60fps)

## Potential Issues & Mitigations

| Issue | Mitigation |
|-------|------------|
| pygame imported elsewhere first | Put SDL_VIDEODRIVER at very top of beetle_physics.py |
| Controller disconnected mid-game | try/except in get_controller_inputs(), event-based refresh |
| Index shifting on hot-plug | Rebuild entire controller list on device events |
| Axis never quite reaches 1.0 | Divide by 32767.0 not 32768.0 |
| Event queue fills up | pygame.event.get() drains it every frame |

## Testing Checklist

- [ ] Controller detected on startup
- [ ] Blue beetle moves with controller 1, left stick
- [ ] Blue beetle turns with controller 1, right stick
- [ ] Horn controls work with triggers/bumpers
- [ ] Red beetle uses controller 2 when connected
- [ ] Red beetle falls back to keyboard when only 1 controller
- [ ] Network mode uses controller for local beetle
- [ ] Keyboard still works when controller connected
- [ ] Hot-plug: controller works when plugged in mid-game
- [ ] Hot-unplug: game doesn't crash when controller removed
- [ ] Steam Deck built-in controls work
- [ ] No FPS drop from controller code

## Steam Deck Notes

- Steam Deck's controls register as standard Xbox-style controller
- pygame._sdl2.controller handles this automatically
- No special code needed for Steam Deck vs other controllers
- Test in Desktop Mode first, then Gaming Mode

## Future Enhancements (Not in initial implementation)

- [ ] Button remapping UI
- [ ] Vibration/haptic feedback on collisions
- [ ] Show controller button prompts in UI
- [ ] Support for gyro aiming (Steam Deck)
- [ ] Per-player sensitivity settings
- [ ] Analog stick sensitivity (proportional speed instead of binary)
