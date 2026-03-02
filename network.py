"""
Beetle Battle Networking Module

Proper Steam networking using py_steam_net:
- Steam lobby creation and discovery
- P2P messaging through Steam (NAT traversal handled automatically)
- No IP addresses needed - players connect via Steam

Requires:
- Python 3.12
- py_steam_net (pip install from GitHub releases)
- steam_api64.dll in game folder
- Steam client running
- steam_appid.txt with app ID (3998620)
"""

import os
import struct
import time
import threading

# Add current directory for DLL loading
os.add_dll_directory(os.getcwd())

# Try to import py_steam_net
try:
    from py_steam_net import PySteamClient
    STEAM_AVAILABLE = True
except ImportError as e:
    STEAM_AVAILABLE = False
    print(f"[Network] py_steam_net import failed: {e}")
    print("[Network] Make sure py_steam_net is installed and steam_api64.dll exists")
except Exception as e:
    STEAM_AVAILABLE = False
    print(f"[Network] Steam error: {e}")


# Network message types
MSG_INPUT = 0x01        # Frame input data
MSG_READY = 0x02        # Player ready signal
MSG_START = 0x03        # Match start (from host)
MSG_HORN_SELECT = 0x04  # Horn type selection
MSG_REMATCH = 0x05      # Rematch request
MSG_PING = 0x06         # Ping for latency measurement
MSG_PONG = 0x07         # Ping response
MSG_STATE_SYNC = 0x08   # Host sends authoritative game state
MSG_SYNC_READY = 0x09   # Guest confirms ready to start (countdown sync)
MSG_GO = 0x0A           # Host tells everyone to start simulating
MSG_FRAME_SYNC = 0x0B   # Host sends frame counter for sync
MSG_BEETLE_CONFIG = 0x0C  # Beetle customization (horn type + sizes)
MSG_SCORE = 0x0D          # Host sends authoritative score event (death/goal)
MSG_GAME_OPTIONS = 0x0E   # Game options (referee enabled, ball active)
MSG_RECONNECT_REQUEST = 0x0F  # Guest requests full state after reconnecting
MSG_RECONNECT_STATE = 0x10    # Host sends full game state snapshot
MSG_DISCONNECT = 0x11         # Player intentionally leaving (graceful exit)
MSG_BALL_EXPLODE = 0x12       # Host tells guest ball has exploded (with position)

# Steam message send flags
SEND_RELIABLE = 2       # Reliable delivery (like TCP)
SEND_UNRELIABLE = 0     # Unreliable (like UDP, faster)

# Default channel for game messages
GAME_CHANNEL = 0

# Steam lobby types (k_ELobbyType enum values)
LOBBY_TYPE_PRIVATE = 0       # Only visible to friends of members
LOBBY_TYPE_FRIENDS_ONLY = 1  # Visible to friends only
LOBBY_TYPE_PUBLIC = 2        # Visible to everyone
LOBBY_TYPE_INVISIBLE = 3     # Returned in search only

# String to int mapping for convenience
LOBBY_TYPES = {
    "private": LOBBY_TYPE_PRIVATE,
    "friends": LOBBY_TYPE_FRIENDS_ONLY,
    "friends_only": LOBBY_TYPE_FRIENDS_ONLY,
    "public": LOBBY_TYPE_PUBLIC,
    "invisible": LOBBY_TYPE_INVISIBLE,
}


class NetworkManager:
    """
    Manages Steam networking for Beetle Battle multiplayer.

    Uses Steam lobbies for matchmaking and P2P for gameplay.
    No IP addresses needed - Steam handles NAT traversal.

    Usage:
        network = NetworkManager()
        if network.init():
            network.create_lobby()  # Host
            # or
            network.join_lobby(lobby_id)  # Join

            # In game loop:
            network.send_input(frame, input_bits)
            network.poll_messages(input_buffer)
    """

    def __init__(self):
        self.client = None
        self.initialized = False
        self.is_host = False
        self.in_lobby = False
        self.connected = False  # True when opponent is in lobby

        # Steam IDs
        self.my_steam_id = None
        self.peer_steam_id = None
        self.lobby_id = None

        # Player info
        self.my_name = "Player"
        self.peer_name = "Opponent"

        # Match state
        self.local_ready = False
        self.remote_ready = False
        self.match_started = False
        self.start_frame = 0
        self.random_seed = 0

        # Horn selection
        self.local_horn = "rhino"
        self.remote_horn = "rhino"

        # Latency tracking
        self.ping_sent_time = 0
        self.ping_ms = 0

        # Debug
        self.poll_count = 0
        self.inputs_received = 0
        self.inputs_sent = 0

        # Message buffer (filled by callbacks, processed by poll_messages)
        self.message_queue = []
        self.message_lock = threading.Lock()

        # Callbacks (set by game code)
        self.on_peer_joined = None
        self.on_peer_left = None
        self.on_match_start = None
        self.on_horn_selected = None

        # State sync (guest receives from host)
        self.pending_state_sync = None

        # Countdown sync state (ensures both players start at same frame)
        self.sync_state = "idle"  # idle -> waiting_for_guest -> go (host) / idle -> received_start -> go (guest)
        self.guest_sync_ready = False  # Host: has guest sent SYNC_READY?
        self.received_go = False  # Guest: has host sent GO?
        self.go_sent_time = 0  # Host: when GO was sent (to delay start by one-way latency)

        # Frame sync (keeps guest frame counter aligned with host)
        self.target_frame = None  # Guest: frame counter we should be at (from host)

        # Beetle customization sync
        self.remote_beetle_config = None  # Opponent's beetle settings (horn_type_id, sizes)

        # Score sync (host-authoritative death/goal detection)
        self.pending_score = None  # Guest: pending score event from host (0=blue scores, 1=red scores)

        # Game options sync (referee, ball, etc.)
        self.pending_game_options = None  # Guest: pending options from host

        # Reconnection sync
        self.pending_reconnect_request = False  # Host: guest requesting full state
        self.pending_reconnect_state = None  # Guest: full state from host
        self.pending_disconnect = False  # Opponent gracefully leaving

        # Ball explosion sync (host-authoritative)
        self.pending_ball_explode = None  # Guest: ball explosion event from host

    def init(self, app_id=3998620):
        """
        Initialize Steam networking.

        Args:
            app_id: Steam App ID (3998620 for Beetle Battle Bros)

        Returns:
            True if successful
        """
        if not STEAM_AVAILABLE:
            print("[Network] Steam library not available")
            return False

        try:
            self.client = PySteamClient()
            self.client.init(app_id)

            self.my_steam_id = self.client.own_steam_id()
            self.my_name = f"Player_{self.my_steam_id % 10000}"  # TODO: Get actual Steam name

            # Set up callbacks
            self.client.set_lobby_changed_callback(self._on_lobby_changed)
            self.client.set_message_recv_callback(self._on_message_received)
            self.client.set_connection_failed_callback(self._on_connection_failed)

            print(f"[Network] Steam initialized - ID: {self.my_steam_id}")
            self.initialized = True
            return True

        except Exception as e:
            print(f"[Network] Steam init error: {e}")
            return False

    def shutdown(self):
        """Clean up Steam resources."""
        if self.in_lobby:
            self.leave_lobby()
        if self.client:
            try:
                self.client.deinit()
            except:
                pass
        self.initialized = False
        self.connected = False
        print("[Network] Shutdown complete")

    # =========================================================================
    # LOBBY MANAGEMENT
    # =========================================================================

    def create_lobby(self, lobby_type="public", max_players=2):
        """
        Create a new lobby and wait for opponent.

        Args:
            lobby_type: "public", "private", "friends", or "invisible"
            max_players: Maximum players (2-4)

        Returns:
            True if lobby creation started
        """
        if not self.initialized:
            print("[Network] Not initialized")
            return False

        try:
            # Convert string lobby type to integer enum
            if isinstance(lobby_type, str):
                lobby_type_int = LOBBY_TYPES.get(lobby_type.lower(), LOBBY_TYPE_PUBLIC)
            else:
                lobby_type_int = lobby_type

            self.client.create_lobby(lobby_type_int, max_players, self._on_lobby_created)
            self.is_host = True
            print(f"[Network] Creating lobby (type={lobby_type_int})...")
            return True

        except Exception as e:
            print(f"[Network] Failed to create lobby: {e}")
            return False

    def _on_lobby_created(self, *args):
        """Callback when lobby is created."""
        print(f"[Network] _on_lobby_created callback fired with args: {args}")

        # Handle different possible callback signatures
        lobby_id = args[0] if len(args) >= 1 else None

        if lobby_id:
            self.lobby_id = lobby_id
            self.in_lobby = True
            print(f"[Network] Lobby created! ID: {lobby_id}")
        else:
            print(f"[Network] Lobby creation failed - no lobby_id in args")
            self.is_host = False

    def join_lobby(self, lobby_id):
        """
        Join an existing lobby by ID.

        Args:
            lobby_id: Steam lobby ID
        """
        if not self.initialized:
            print("[Network] Not initialized")
            return False

        try:
            print(f"[Network] Attempting to join lobby {lobby_id} (type: {type(lobby_id)})")
            self.client.join_lobby(lobby_id, self._on_lobby_joined)
            self.is_host = False
            print(f"[Network] join_lobby called, waiting for callback...")
            return True

        except Exception as e:
            print(f"[Network] Failed to join lobby: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _on_lobby_joined(self, *args):
        """Callback when joined a lobby."""
        print(f"[Network] _on_lobby_joined callback fired with args: {args}")

        # Handle different possible callback signatures
        if len(args) >= 1:
            lobby_id = args[0]
        else:
            lobby_id = None

        if lobby_id:
            self.lobby_id = lobby_id
            self.in_lobby = True
            print(f"[Network] Joined lobby! ID: {lobby_id}")
            # Check for existing members
            members = self.get_lobby_members()
            print(f"[Network] Lobby members: {members}")
            for member in members:
                if member != self.my_steam_id:
                    self.peer_steam_id = member
                    self.connected = True
                    print(f"[Network] Found opponent: {member}")
                    if self.on_peer_joined:
                        self.on_peer_joined()
                    # Guest sends MSG_READY immediately to establish bidirectional P2P channel
                    if not self.is_host:
                        self._send_packet(struct.pack('>B', MSG_READY), reliable=True)
                        print(f"[Network] Sent MSG_READY to establish P2P channel with host")
        else:
            print(f"[Network] Failed to join lobby - no lobby_id returned")

    def leave_lobby(self):
        """Leave current lobby."""
        if self.in_lobby:
            try:
                self.client.leave_lobby()
            except:
                pass

        self._reset_state()
        print("[Network] Left lobby")

    def _reset_state(self):
        """Reset all connection state."""
        self.in_lobby = False
        self.lobby_id = None
        self.connected = False
        self.peer_steam_id = None
        self.match_started = False
        self.local_ready = False
        self.remote_ready = False

    def get_lobby_members(self):
        """Get list of Steam IDs in current lobby."""
        if not self.in_lobby or not self.lobby_id:
            return []
        try:
            return self.client.get_lobby_members(self.lobby_id)
        except:
            return []

    # =========================================================================
    # STEAM CALLBACKS
    # =========================================================================

    def _on_lobby_changed(self, *args):
        """
        Called when lobby membership changes.
        """
        print(f"[Network] _on_lobby_changed callback fired with args: {args}")

        # Try to parse args - format may vary
        lobby_id = args[0] if len(args) > 0 else None
        member_id = args[1] if len(args) > 1 else None
        change_type = args[2] if len(args) > 2 else "unknown"

        print(f"[Network] Lobby changed: {change_type} - member {member_id}")

        if change_type in ["joined", "entered", 1]:  # 1 might be enum for joined
            if member_id and member_id != self.my_steam_id:
                self.peer_steam_id = member_id
                self.connected = True
                if lobby_id:
                    self.lobby_id = lobby_id
                print(f"[Network] Opponent joined: {member_id}")

                # Send ping immediately to establish bidirectional P2P channel
                if self.is_host:
                    print("[Network] Host sending ping to establish reverse P2P channel...")
                    self.send_ping()

                if self.on_peer_joined:
                    self.on_peer_joined()

        elif change_type in ["left", "disconnected", "kicked", "banned", 2, 3, 4]:
            if member_id == self.peer_steam_id:
                self.peer_steam_id = None
                self.connected = False
                print(f"[Network] Opponent left")
                if self.on_peer_left:
                    self.on_peer_left()

    def _on_message_received(self, *args):
        """Called when a P2P message is received."""
        # Handle different callback signatures
        if len(args) == 2:
            sender_id, data = args
            channel = 0
        elif len(args) == 3:
            sender_id, channel, data = args
        else:
            print(f"[Network] Unexpected callback args: {args}")
            return

        # Only log non-INPUT messages
        if data and data[0] != 0x01:
            msg_names = {0x02: "READY", 0x03: "START", 0x04: "HORN", 0x05: "REMATCH", 0x06: "PING", 0x07: "PONG"}
            msg_name = msg_names.get(data[0], f"0x{data[0]:02x}")
            print(f"[Network] Received {msg_name} from peer")

        with self.message_lock:
            self.message_queue.append((sender_id, channel, bytes(data)))

    def _on_connection_failed(self, steam_id, reason):
        """Called when P2P connection fails."""
        print(f"[Network] Connection failed to {steam_id}: {reason}")
        if steam_id == self.peer_steam_id:
            self.connected = False
            if self.on_peer_left:
                self.on_peer_left()

    # =========================================================================
    # MESSAGE SENDING
    # =========================================================================

    def send_input(self, frame, inputs):
        """
        Send input for a frame to opponent.
        Packet format: [MSG_INPUT (1)] [frame (4)] [inputs (1)] = 6 bytes
        """
        if not self.connected or not self.peer_steam_id:
            return False

        self.inputs_sent += 1
        data = struct.pack('>BIB', MSG_INPUT, frame, inputs)
        return self._send_packet(data, reliable=False)  # Inputs can be unreliable for speed

    def send_ready(self):
        """Signal that local player is ready to start."""
        self.local_ready = True
        self._send_packet(struct.pack('>B', MSG_READY), reliable=True)
        print("[Network] Sent ready signal")

        # If both ready and we're host, start the match
        if self.is_host and self.local_ready and self.remote_ready:
            self._send_start()

    def start_match_now(self):
        """Host immediately starts the match (skip ready handshake for now)."""
        if self.is_host and self.connected:
            self._send_start()

    def send_horn_select(self, horn_type):
        """Send horn type selection to opponent."""
        self.local_horn = horn_type

        if not self.connected:
            return

        horn_ids = {"rhino": 0, "stag": 1, "hercules": 2, "scorpion": 3, "atlas": 4, "bombardier": 5}
        horn_id = horn_ids.get(horn_type, 0)
        self._send_packet(struct.pack('>BB', MSG_HORN_SELECT, horn_id), reliable=True)

    def send_rematch_request(self):
        """Request a rematch."""
        self._send_packet(struct.pack('>B', MSG_REMATCH), reliable=True)

    def send_state_sync(self, frame, blue_x, blue_y, blue_z, blue_rot, red_x, red_y, red_z, red_rot,
                        ball_x=0.0, ball_y=0.0, ball_z=0.0, ball_active=False):
        """
        Host sends authoritative state to guest.
        Packet format: [type:1][frame:4][blue_x:4][blue_y:4][blue_z:4][blue_rot:4]
                       [red_x:4][red_y:4][red_z:4][red_rot:4]
                       [ball_x:4][ball_y:4][ball_z:4][ball_active:1] = 50 bytes
        """
        if not self.is_host or not self.connected:
            return

        data = struct.pack('>BIfffffffffffB',
                           MSG_STATE_SYNC, frame,
                           blue_x, blue_y, blue_z, blue_rot,
                           red_x, red_y, red_z, red_rot,
                           ball_x, ball_y, ball_z, 1 if ball_active else 0)
        self._send_packet(data, reliable=False)  # Unreliable is fine for periodic sync

    def send_sync_ready(self):
        """Guest sends SYNC_READY to host to confirm ready to start."""
        print("[Network] Sending SYNC_READY to host")
        self._send_packet(struct.pack('>B', MSG_SYNC_READY), reliable=True)

    def send_go(self):
        """Host sends GO to start simulation on all clients."""
        if not self.is_host:
            return
        print("[Network] Sending GO - all players start now!")
        # Send multiple times for reliability
        for _ in range(3):
            self._send_packet(struct.pack('>B', MSG_GO), reliable=True)
        # Don't set sync_state = "go" yet - wait for one-way latency
        # so guest receives GO at approximately the same time we start
        self.go_sent_time = time.time()
        self.sync_state = "waiting_for_go_delay"

    def is_ready_to_simulate(self):
        """Check if countdown sync is complete and we can start simulating."""
        if self.sync_state == "go":
            return True
        # Host: wait for one-way latency after sending GO
        if self.is_host and self.sync_state == "waiting_for_go_delay":
            one_way_latency = (self.ping_ms / 1000.0) / 2.0  # Half of round-trip
            if one_way_latency < 0.016:  # Minimum 1 frame
                one_way_latency = 0.016
            elapsed = time.time() - self.go_sent_time
            if elapsed >= one_way_latency:
                print(f"[Network] GO delay complete ({elapsed*1000:.0f}ms), starting simulation!")
                self.sync_state = "go"
                return True
        return False

    def send_frame_sync(self, frame):
        """Host sends current frame counter to keep guest in sync."""
        if not self.is_host:
            return
        data = struct.pack('>BI', MSG_FRAME_SYNC, frame)
        self._send_packet(data, reliable=False)

    def send_beetle_config(self, horn_type_id, shaft, prong, back_body, body_len, body_width, leg_len,
                           body_color, leg_color, leg_tip_color, stripe_color, horn_tip_color):
        """Send beetle customization to opponent including colors."""
        player_id = 0 if self.is_host else 1
        # Pack sizes (9 bytes) + 5 colors as RGB bytes (15 bytes) = 24 bytes total
        # Colors are floats 0.0-1.0, convert to bytes 0-255
        def color_to_bytes(c):
            return (int(c[0] * 255), int(c[1] * 255), int(c[2] * 255))
        bc = color_to_bytes(body_color)
        lc = color_to_bytes(leg_color)
        ltc = color_to_bytes(leg_tip_color)
        sc = color_to_bytes(stripe_color)
        htc = color_to_bytes(horn_tip_color)
        data = struct.pack('>BBBBBBBBB BBBBBBBBBBBBBBB', MSG_BEETLE_CONFIG, player_id,
                           horn_type_id, shaft, prong, back_body, body_len, body_width, leg_len,
                           bc[0], bc[1], bc[2], lc[0], lc[1], lc[2], ltc[0], ltc[1], ltc[2],
                           sc[0], sc[1], sc[2], htc[0], htc[1], htc[2])
        self._send_packet(data, reliable=True)
        print(f"[Network] Sent beetle config: horn={horn_type_id}, sizes={shaft}/{prong}/{back_body}/{body_len}/{body_width}/{leg_len}")

    def send_score(self, scorer, score_type=0, death_x=0.0, death_z=0.0):
        """
        Host sends authoritative score event to guest.

        Args:
            scorer: 0 = blue scores (red died/goal), 1 = red scores (blue died/goal)
            score_type: 0 = beetle death (explode beetle), 1 = ball goal (don't explode beetle)
            death_x, death_z: Position where beetle died (for smooth death animation on guest)
        """
        if not self.is_host or not self.connected:
            return
        data = struct.pack('>BBBff', MSG_SCORE, scorer, score_type, death_x, death_z)
        self._send_packet(data, reliable=True)
        type_str = "death" if score_type == 0 else "ball goal"
        print(f"[Network] Sent score event: {'Blue' if scorer == 0 else 'Red'} scores ({type_str}) at ({death_x:.1f}, {death_z:.1f})")

    def send_game_options(self, referee_enabled, ball_active, donut_mode=False, x_stage_mode=False, barbell_mode=False, yinyang_mode=False, hourglass_mode=False, tornado_mode=False, sandstorm_mode=False, ufo_mode=False, ice_mode=False, figure8_mode=False, squiggle_mode=False, hole_mode=False, comet_mode=False, square_mode=False):
        """
        Host sends game options to guest.
        Packet format: [type:1][referee:1][ball:1][donut:1][x_stage:1][barbell:1][yinyang:1][hourglass:1][tornado:1][sandstorm:1][ufo:1][ice:1][figure8:1][squiggle:1][hole:1][comet:1][square:1] = 17 bytes
        """
        if not self.is_host or not self.connected:
            return
        data = struct.pack('>BBBBBBBBBBBBBBBBB', MSG_GAME_OPTIONS,
                          1 if referee_enabled else 0,
                          1 if ball_active else 0,
                          1 if donut_mode else 0,
                          1 if x_stage_mode else 0,
                          1 if barbell_mode else 0,
                          1 if yinyang_mode else 0,
                          1 if hourglass_mode else 0,
                          1 if tornado_mode else 0,
                          1 if sandstorm_mode else 0,
                          1 if ufo_mode else 0,
                          1 if ice_mode else 0,
                          1 if figure8_mode else 0,
                          1 if squiggle_mode else 0,
                          1 if hole_mode else 0,
                          1 if comet_mode else 0,
                          1 if square_mode else 0)
        self._send_packet(data, reliable=True)

    def send_ball_explode(self, pos_x, pos_y, pos_z):
        """
        Host tells guest the ball has exploded (host-authoritative).
        Packet format: [type:1][x:4][y:4][z:4] = 13 bytes
        """
        if not self.is_host or not self.connected:
            return
        data = struct.pack('>Bfff', MSG_BALL_EXPLODE, pos_x, pos_y, pos_z)
        self._send_packet(data, reliable=True)
        print(f"[Network] Sent ball explode at ({pos_x:.1f}, {pos_y:.1f}, {pos_z:.1f})")

    def send_reconnect_request(self):
        """
        Guest sends request for full state after reconnecting.
        Host will respond with MSG_RECONNECT_STATE containing full game snapshot.
        """
        if self.is_host:
            return  # Only guest sends this
        data = struct.pack('>B', MSG_RECONNECT_REQUEST)
        self._send_packet(data, reliable=True)
        print("[Network] Sent reconnect request")

    def send_reconnect_state(self, frame, blue_state, red_state, ball_state, scores, timers):
        """
        Host sends full game state snapshot after opponent reconnects.

        Args:
            frame: Current physics frame number
            blue_state: dict with x, y, z, vx, vy, vz, rotation, pitch, roll
            red_state: dict with x, y, z, vx, vy, vz, rotation, pitch, roll
            ball_state: dict with x, y, z, vx, vy, vz, active
            scores: tuple (blue_score, red_score)
            timers: tuple (blue_respawn_timer, red_respawn_timer)
        """
        if not self.is_host or not self.connected:
            return
        data = struct.pack('>BI fffffffff fffffffff ffffff B BB ff',
            MSG_RECONNECT_STATE,
            frame,
            # Blue beetle (9 floats = 36 bytes)
            blue_state['x'], blue_state['y'], blue_state['z'],
            blue_state['vx'], blue_state['vy'], blue_state['vz'],
            blue_state['rotation'], blue_state['pitch'], blue_state['roll'],
            # Red beetle (9 floats = 36 bytes)
            red_state['x'], red_state['y'], red_state['z'],
            red_state['vx'], red_state['vy'], red_state['vz'],
            red_state['rotation'], red_state['pitch'], red_state['roll'],
            # Ball (6 floats = 24 bytes)
            ball_state['x'], ball_state['y'], ball_state['z'],
            ball_state['vx'], ball_state['vy'], ball_state['vz'],
            ball_state['active'],  # 1 byte
            # Scores (2 bytes)
            scores[0], scores[1],
            # Timers (8 bytes)
            timers[0], timers[1]
        )
        self._send_packet(data, reliable=True)
        print(f"[Network] Sent reconnect state: frame={frame}, scores={scores}")

    def send_disconnect(self):
        """
        Send graceful disconnect message before leaving.
        """
        data = struct.pack('>B', MSG_DISCONNECT)
        # Send multiple times for reliability
        for _ in range(3):
            self._send_packet(data, reliable=True)
        print("[Network] Sent disconnect message")

    def send_ping(self):
        """Send ping to measure latency."""
        # Use lower 32 bits of milliseconds to fit in uint32
        self.ping_sent_time = int(time.time() * 1000) & 0xFFFFFFFF
        self._send_packet(struct.pack('>BI', MSG_PING, self.ping_sent_time), reliable=False)

    def _send_start(self):
        """Host sends match start signal - waits for guest SYNC_READY before GO."""
        import random
        self.random_seed = random.randint(0, 2**32 - 1)
        self.start_frame = 0

        data = struct.pack('>BII', MSG_START, self.start_frame, self.random_seed)

        # Send multiple times to ensure delivery (P2P channel may still be establishing)
        for i in range(3):
            self._send_packet(data, reliable=True)
            time.sleep(0.05)  # Small delay between sends

        # Don't start yet - wait for guest to confirm ready
        self.sync_state = "waiting_for_guest"
        self.match_started = True  # Match is "started" but not simulating yet
        print(f"[Network] START sent, waiting for guest SYNC_READY... Seed: {self.random_seed}")

    def _send_packet(self, data, reliable=True):
        """Send raw packet to peer via Steam P2P."""
        if not self.peer_steam_id:
            print(f"[Network] Send failed: no peer_steam_id")
            return False

        try:
            send_type = SEND_RELIABLE if reliable else SEND_UNRELIABLE
            # Only log non-INPUT messages (INPUT is too spammy at 60/sec)
            if data and data[0] != 0x01:  # Not INPUT
                msg_names = {0x02: "READY", 0x03: "START", 0x04: "HORN", 0x05: "REMATCH", 0x06: "PING", 0x07: "PONG"}
                msg_name = msg_names.get(data[0], f"0x{data[0]:02x}")
                print(f"[Network] Sending {msg_name}")
            self.client.send_message_to(self.peer_steam_id, send_type, GAME_CHANNEL, data)
            return True
        except Exception as e:
            print(f"[Network] Send error: {e}")
            return False

    # =========================================================================
    # MESSAGE RECEIVING
    # =========================================================================

    def poll_messages(self, input_buffer=None):
        """
        Process incoming network messages.
        Call this every frame.

        Args:
            input_buffer: InputBuffer instance to store received inputs
        """
        if not self.initialized:
            return

        self.poll_count += 1

        # Periodic debug output (every 60 frames = ~1 second)
        if self.poll_count % 60 == 0:
            queue_len = len(self.message_queue)
            print(f"[Network] Poll #{self.poll_count}: in={self.inputs_received} out={self.inputs_sent} host={self.is_host}")

        # Run Steam callbacks (don't skip based on is_ready - always try)
        try:
            self.client.run_callbacks()
        except Exception as e:
            print(f"[Network] run_callbacks error: {e}")

        # FALLBACK: If we're in a lobby but haven't detected opponent, check member list directly
        # This handles cases where the lobby_changed callback doesn't fire
        if self.in_lobby and not self.connected and self.lobby_id:
            try:
                members = self.client.get_lobby_members(self.lobby_id)
                for member in members:
                    if member != self.my_steam_id:
                        self.peer_steam_id = member
                        self.connected = True
                        print(f"[Network] Found opponent via polling: {member}")

                        # Send ping immediately to establish bidirectional P2P channel
                        if self.is_host:
                            print("[Network] Host sending ping to establish reverse P2P channel...")
                            self.send_ping()

                        if self.on_peer_joined:
                            self.on_peer_joined()
                        break
            except:
                pass

        # Explicitly receive messages from Steam
        try:
            result = self.client.receive_messages(GAME_CHANNEL, 100)
            # Check if receive_messages returns messages directly (vs callback)
            if result and isinstance(result, list) and len(result) > 0:
                for msg in result:
                    if hasattr(msg, 'data'):
                        self._handle_packet(msg.data, getattr(msg, 'sender', None), input_buffer)
                    elif isinstance(msg, tuple) and len(msg) >= 2:
                        sender_id, data = msg[0], msg[-1]
                        self._handle_packet(bytes(data) if not isinstance(data, bytes) else data, sender_id, input_buffer)
                    elif isinstance(msg, bytes):
                        self._handle_packet(msg, None, input_buffer)
        except Exception as e:
            print(f"[Network] receive_messages error: {e}")

        # Process queued messages (from callback, if it works)
        messages = []
        with self.message_lock:
            messages = self.message_queue[:]
            self.message_queue.clear()

        for sender_id, channel, data in messages:
            try:
                self._handle_packet(data, sender_id, input_buffer)
            except Exception as e:
                print(f"[Network] Error handling packet: {e}")

    def _handle_packet(self, data, sender_id, input_buffer):
        """Process a received network packet."""
        if len(data) < 1:
            return

        msg_type = data[0]

        if msg_type == MSG_INPUT:
            # Input packet: [type (1)] [frame (4)] [inputs (1)]
            if len(data) >= 6:
                _, frame, inputs = struct.unpack('>BIB', data[:6])
                self.inputs_received += 1
                if input_buffer:
                    # Debug: log frame mismatch periodically
                    if self.inputs_received % 60 == 1:
                        print(f"[Network] Frame check: received={frame}, local={input_buffer.current_frame}, diff={input_buffer.current_frame - frame}")
                    input_buffer.add_remote(frame, inputs)

        elif msg_type == MSG_READY:
            self.remote_ready = True
            print(f"[Network] Opponent is ready")

            # If both ready and we're host, start the match
            if self.is_host and self.local_ready and self.remote_ready:
                self._send_start()

        elif msg_type == MSG_START:
            # Start packet: [type (1)] [frame (4)] [seed (4)]
            if len(data) >= 9:
                _, self.start_frame, self.random_seed = struct.unpack('>BII', data[:9])
                self.match_started = True
                print(f"[Network] Match starting! Frame: {self.start_frame}, Seed: {self.random_seed}")

                if self.on_match_start:
                    self.on_match_start(self.start_frame, self.random_seed)

        elif msg_type == MSG_HORN_SELECT:
            # Horn select: [type (1)] [horn_id (1)]
            if len(data) >= 2:
                horn_id = data[1]
                horn_names = {0: "rhino", 1: "stag", 2: "hercules", 3: "scorpion", 4: "atlas", 5: "bombardier"}
                self.remote_horn = horn_names.get(horn_id, "rhino")
                print(f"[Network] Opponent selected {self.remote_horn}")

                if self.on_horn_selected:
                    self.on_horn_selected(self.remote_horn)

        elif msg_type == MSG_REMATCH:
            print(f"[Network] Opponent wants a rematch")

        elif msg_type == MSG_PING:
            # Respond to ping with pong
            if len(data) >= 5:
                timestamp = struct.unpack('>I', data[1:5])[0]
                self._send_packet(struct.pack('>BI', MSG_PONG, timestamp), reliable=False)

        elif msg_type == MSG_PONG:
            # Calculate ping from pong response
            if len(data) >= 5:
                sent_time = struct.unpack('>I', data[1:5])[0]
                now = int(time.time() * 1000) & 0xFFFFFFFF
                # Handle wraparound
                if now >= sent_time:
                    self.ping_ms = now - sent_time
                else:
                    self.ping_ms = (0xFFFFFFFF - sent_time) + now

        elif msg_type == MSG_STATE_SYNC:
            # Host state sync: [type:1][frame:4][beetles:32][ball:13] = 50 bytes (new format with Y)
            if len(data) >= 50 and not self.is_host:
                _, frame, blue_x, blue_y, blue_z, blue_rot, red_x, red_y, red_z, red_rot, ball_x, ball_y, ball_z, ball_active = struct.unpack('>BIfffffffffffB', data[:50])
                # Store for guest to apply
                self.pending_state_sync = {
                    'frame': frame,
                    'blue_x': blue_x, 'blue_y': blue_y, 'blue_z': blue_z, 'blue_rot': blue_rot,
                    'red_x': red_x, 'red_y': red_y, 'red_z': red_z, 'red_rot': red_rot,
                    'ball_x': ball_x, 'ball_y': ball_y, 'ball_z': ball_z, 'ball_active': ball_active == 1
                }
            elif len(data) >= 42 and not self.is_host:
                # Backwards compatibility with old 42-byte format (no beetle Y)
                _, frame, blue_x, blue_z, blue_rot, red_x, red_z, red_rot, ball_x, ball_y, ball_z, ball_active = struct.unpack('>BIfffffffffB', data[:42])
                self.pending_state_sync = {
                    'frame': frame,
                    'blue_x': blue_x, 'blue_z': blue_z, 'blue_rot': blue_rot,
                    'red_x': red_x, 'red_z': red_z, 'red_rot': red_rot,
                    'ball_x': ball_x, 'ball_y': ball_y, 'ball_z': ball_z, 'ball_active': ball_active == 1
                }
            elif len(data) >= 29 and not self.is_host:
                # Backwards compatibility with old 29-byte format (no ball, no Y)
                _, frame, blue_x, blue_z, blue_rot, red_x, red_z, red_rot = struct.unpack('>BIffffff', data[:29])
                self.pending_state_sync = {
                    'frame': frame,
                    'blue_x': blue_x, 'blue_z': blue_z, 'blue_rot': blue_rot,
                    'red_x': red_x, 'red_z': red_z, 'red_rot': red_rot
                }

        elif msg_type == MSG_SYNC_READY:
            # Guest is ready to start - host can send GO
            if self.is_host:
                print("[Network] Received SYNC_READY from guest")
                self.guest_sync_ready = True
                # Send GO to start simulation
                self.send_go()

        elif msg_type == MSG_GO:
            # Host says GO - start simulating!
            if not self.is_host:
                print("[Network] Received GO from host - starting simulation!")
                self.sync_state = "go"
                self.received_go = True

        elif msg_type == MSG_FRAME_SYNC:
            # Host sends frame counter for sync
            if len(data) >= 5 and not self.is_host:
                _, target_frame = struct.unpack('>BI', data[:5])
                self.target_frame = target_frame

        elif msg_type == MSG_BEETLE_CONFIG:
            # Opponent's beetle customization (24 bytes with colors, 9 bytes legacy)
            if len(data) >= 24:
                # New format with colors
                unpacked = struct.unpack('>BBBBBBBBB BBBBBBBBBBBBBBB', data[:24])
                _, player_id, horn_id, shaft, prong, back_body, body_len, body_width, leg_len = unpacked[:9]
                # Convert color bytes (0-255) to floats (0.0-1.0)
                body_color = (unpacked[9]/255, unpacked[10]/255, unpacked[11]/255)
                leg_color = (unpacked[12]/255, unpacked[13]/255, unpacked[14]/255)
                leg_tip_color = (unpacked[15]/255, unpacked[16]/255, unpacked[17]/255)
                stripe_color = (unpacked[18]/255, unpacked[19]/255, unpacked[20]/255)
                horn_tip_color = (unpacked[21]/255, unpacked[22]/255, unpacked[23]/255)
                self.remote_beetle_config = {
                    'player_id': player_id,
                    'horn_type_id': horn_id,
                    'shaft': shaft,
                    'prong': prong,
                    'back_body': back_body,
                    'body_len': body_len,
                    'body_width': body_width,
                    'leg_len': leg_len,
                    'body_color': body_color,
                    'leg_color': leg_color,
                    'leg_tip_color': leg_tip_color,
                    'stripe_color': stripe_color,
                    'horn_tip_color': horn_tip_color
                }
                print(f"[Network] Received beetle config from player {player_id}: horn={horn_id}, sizes={shaft}/{prong}/{back_body}/{body_len}/{body_width}/{leg_len}")
            elif len(data) >= 9:
                # Legacy format without colors
                _, player_id, horn_id, shaft, prong, back_body, body_len, body_width, leg_len = struct.unpack('>BBBBBBBBB', data[:9])
                self.remote_beetle_config = {
                    'player_id': player_id,
                    'horn_type_id': horn_id,
                    'shaft': shaft,
                    'prong': prong,
                    'back_body': back_body,
                    'body_len': body_len,
                    'body_width': body_width,
                    'leg_len': leg_len
                }
                print(f"[Network] Received beetle config from player {player_id}: horn={horn_id}, sizes={shaft}/{prong}/{back_body}/{body_len}/{body_width}/{leg_len}")

        elif msg_type == MSG_SCORE:
            # Host-authoritative score event (guest receives)
            if len(data) >= 11 and not self.is_host:
                # New format with death position
                _, scorer, score_type, death_x, death_z = struct.unpack('>BBBff', data[:11])
                self.pending_score = {'scorer': scorer, 'score_type': score_type, 'death_x': death_x, 'death_z': death_z}
                type_str = "death" if score_type == 0 else "ball goal"
                print(f"[Network] Received score event: {'Blue' if scorer == 0 else 'Red'} scores ({type_str}) at ({death_x:.1f}, {death_z:.1f})")
            elif len(data) >= 3 and not self.is_host:
                # Old format without death position (backwards compatibility)
                _, scorer, score_type = struct.unpack('>BBB', data[:3])
                self.pending_score = {'scorer': scorer, 'score_type': score_type, 'death_x': 0.0, 'death_z': 0.0}
                type_str = "death" if score_type == 0 else "ball goal"
                print(f"[Network] Received score event: {'Blue' if scorer == 0 else 'Red'} scores ({type_str})")

        elif msg_type == MSG_GAME_OPTIONS:
            # Host sends game options (guest receives)
            if len(data) >= 17 and not self.is_host:
                # New format with square
                _, referee_enabled, ball_active, donut_mode, x_stage_mode, barbell_mode, yinyang_mode, hourglass_mode, tornado_mode, sandstorm_mode, ufo_mode, ice_mode, figure8_mode, squiggle_mode, hole_mode, comet_mode, square_mode = struct.unpack('>BBBBBBBBBBBBBBBBB', data[:17])
                self.pending_game_options = {
                    'referee_enabled': referee_enabled == 1,
                    'ball_active': ball_active == 1,
                    'donut_mode': donut_mode == 1,
                    'x_stage_mode': x_stage_mode == 1,
                    'barbell_mode': barbell_mode == 1,
                    'yinyang_mode': yinyang_mode == 1,
                    'hourglass_mode': hourglass_mode == 1,
                    'tornado_mode': tornado_mode == 1,
                    'sandstorm_mode': sandstorm_mode == 1,
                    'ufo_mode': ufo_mode == 1,
                    'ice_mode': ice_mode == 1,
                    'figure8_mode': figure8_mode == 1,
                    'squiggle_mode': squiggle_mode == 1,
                    'hole_mode': hole_mode == 1,
                    'comet_mode': comet_mode == 1,
                    'square_mode': square_mode == 1
                }
                print(f"[Network] Received game options: referee={referee_enabled}, ball={ball_active}, donut={donut_mode}, x_stage={x_stage_mode}, barbell={barbell_mode}, yinyang={yinyang_mode}, hourglass={hourglass_mode}, tornado={tornado_mode}, sandstorm={sandstorm_mode}, ufo={ufo_mode}, ice={ice_mode}, figure8={figure8_mode}, squiggle={squiggle_mode}, hole={hole_mode}, comet={comet_mode}, square={square_mode}")
            elif len(data) >= 16 and not self.is_host:
                # Backwards compatibility with 16-byte format (no square)
                _, referee_enabled, ball_active, donut_mode, x_stage_mode, barbell_mode, yinyang_mode, hourglass_mode, tornado_mode, sandstorm_mode, ufo_mode, ice_mode, figure8_mode, squiggle_mode, hole_mode, comet_mode = struct.unpack('>BBBBBBBBBBBBBBBB', data[:16])
                self.pending_game_options = {
                    'referee_enabled': referee_enabled == 1,
                    'ball_active': ball_active == 1,
                    'donut_mode': donut_mode == 1,
                    'x_stage_mode': x_stage_mode == 1,
                    'barbell_mode': barbell_mode == 1,
                    'yinyang_mode': yinyang_mode == 1,
                    'hourglass_mode': hourglass_mode == 1,
                    'tornado_mode': tornado_mode == 1,
                    'sandstorm_mode': sandstorm_mode == 1,
                    'ufo_mode': ufo_mode == 1,
                    'ice_mode': ice_mode == 1,
                    'figure8_mode': figure8_mode == 1,
                    'squiggle_mode': squiggle_mode == 1,
                    'hole_mode': hole_mode == 1,
                    'comet_mode': comet_mode == 1,
                    'square_mode': False
                }
                print(f"[Network] Received game options: referee={referee_enabled}, ball={ball_active}, donut={donut_mode}, x_stage={x_stage_mode}, barbell={barbell_mode}, yinyang={yinyang_mode}, hourglass={hourglass_mode}, tornado={tornado_mode}, sandstorm={sandstorm_mode}, ufo={ufo_mode}, ice={ice_mode}, figure8={figure8_mode}, squiggle={squiggle_mode}, hole={hole_mode}, comet={comet_mode}")
            elif len(data) >= 15 and not self.is_host:
                # Backwards compatibility with 15-byte format (no comet)
                _, referee_enabled, ball_active, donut_mode, x_stage_mode, barbell_mode, yinyang_mode, hourglass_mode, tornado_mode, sandstorm_mode, ufo_mode, ice_mode, figure8_mode, squiggle_mode, hole_mode = struct.unpack('>BBBBBBBBBBBBBBB', data[:15])
                self.pending_game_options = {
                    'referee_enabled': referee_enabled == 1,
                    'ball_active': ball_active == 1,
                    'donut_mode': donut_mode == 1,
                    'x_stage_mode': x_stage_mode == 1,
                    'barbell_mode': barbell_mode == 1,
                    'yinyang_mode': yinyang_mode == 1,
                    'hourglass_mode': hourglass_mode == 1,
                    'tornado_mode': tornado_mode == 1,
                    'sandstorm_mode': sandstorm_mode == 1,
                    'ufo_mode': ufo_mode == 1,
                    'ice_mode': ice_mode == 1,
                    'figure8_mode': figure8_mode == 1,
                    'squiggle_mode': squiggle_mode == 1,
                    'hole_mode': hole_mode == 1,
                    'comet_mode': False,
                    'square_mode': False
                }
                print(f"[Network] Received game options: referee={referee_enabled}, ball={ball_active}, donut={donut_mode}, x_stage={x_stage_mode}, barbell={barbell_mode}, yinyang={yinyang_mode}, hourglass={hourglass_mode}, tornado={tornado_mode}, sandstorm={sandstorm_mode}, ufo={ufo_mode}, ice={ice_mode}, figure8={figure8_mode}, squiggle={squiggle_mode}, hole={hole_mode}")
            elif len(data) >= 14 and not self.is_host:
                # Backwards compatibility with 14-byte format (no hole)
                _, referee_enabled, ball_active, donut_mode, x_stage_mode, barbell_mode, yinyang_mode, hourglass_mode, tornado_mode, sandstorm_mode, ufo_mode, ice_mode, figure8_mode, squiggle_mode = struct.unpack('>BBBBBBBBBBBBBB', data[:14])
                self.pending_game_options = {
                    'referee_enabled': referee_enabled == 1,
                    'ball_active': ball_active == 1,
                    'donut_mode': donut_mode == 1,
                    'x_stage_mode': x_stage_mode == 1,
                    'barbell_mode': barbell_mode == 1,
                    'yinyang_mode': yinyang_mode == 1,
                    'hourglass_mode': hourglass_mode == 1,
                    'tornado_mode': tornado_mode == 1,
                    'sandstorm_mode': sandstorm_mode == 1,
                    'ufo_mode': ufo_mode == 1,
                    'ice_mode': ice_mode == 1,
                    'figure8_mode': figure8_mode == 1,
                    'squiggle_mode': squiggle_mode == 1,
                    'hole_mode': False,
                    'comet_mode': False,
                    'square_mode': False
                }
                print(f"[Network] Received game options: referee={referee_enabled}, ball={ball_active}, donut={donut_mode}, x_stage={x_stage_mode}, barbell={barbell_mode}, yinyang={yinyang_mode}, hourglass={hourglass_mode}, tornado={tornado_mode}, sandstorm={sandstorm_mode}, ufo={ufo_mode}, ice={ice_mode}, figure8={figure8_mode}, squiggle={squiggle_mode}")
            elif len(data) >= 13 and not self.is_host:
                # Backwards compatibility with 13-byte format (no squiggle)
                _, referee_enabled, ball_active, donut_mode, x_stage_mode, barbell_mode, yinyang_mode, hourglass_mode, tornado_mode, sandstorm_mode, ufo_mode, ice_mode, figure8_mode = struct.unpack('>BBBBBBBBBBBBB', data[:13])
                self.pending_game_options = {
                    'referee_enabled': referee_enabled == 1,
                    'ball_active': ball_active == 1,
                    'donut_mode': donut_mode == 1,
                    'x_stage_mode': x_stage_mode == 1,
                    'barbell_mode': barbell_mode == 1,
                    'yinyang_mode': yinyang_mode == 1,
                    'hourglass_mode': hourglass_mode == 1,
                    'tornado_mode': tornado_mode == 1,
                    'sandstorm_mode': sandstorm_mode == 1,
                    'ufo_mode': ufo_mode == 1,
                    'ice_mode': ice_mode == 1,
                    'figure8_mode': figure8_mode == 1,
                    'squiggle_mode': False,
                    'hole_mode': False,
                    'comet_mode': False,
                    'square_mode': False
                }
                print(f"[Network] Received game options: referee={referee_enabled}, ball={ball_active}, donut={donut_mode}, x_stage={x_stage_mode}, barbell={barbell_mode}, yinyang={yinyang_mode}, hourglass={hourglass_mode}, tornado={tornado_mode}, sandstorm={sandstorm_mode}, ufo={ufo_mode}, ice={ice_mode}, figure8={figure8_mode}")
            elif len(data) >= 12 and not self.is_host:
                # Backwards compatibility with 12-byte format (no figure8)
                _, referee_enabled, ball_active, donut_mode, x_stage_mode, barbell_mode, yinyang_mode, hourglass_mode, tornado_mode, sandstorm_mode, ufo_mode, ice_mode = struct.unpack('>BBBBBBBBBBBB', data[:12])
                self.pending_game_options = {
                    'referee_enabled': referee_enabled == 1,
                    'ball_active': ball_active == 1,
                    'donut_mode': donut_mode == 1,
                    'x_stage_mode': x_stage_mode == 1,
                    'barbell_mode': barbell_mode == 1,
                    'yinyang_mode': yinyang_mode == 1,
                    'hourglass_mode': hourglass_mode == 1,
                    'tornado_mode': tornado_mode == 1,
                    'sandstorm_mode': sandstorm_mode == 1,
                    'ufo_mode': ufo_mode == 1,
                    'ice_mode': ice_mode == 1,
                    'figure8_mode': False,
                    'squiggle_mode': False,
                    'hole_mode': False,
                    'comet_mode': False,
                    'square_mode': False
                }
                print(f"[Network] Received game options: referee={referee_enabled}, ball={ball_active}, donut={donut_mode}, x_stage={x_stage_mode}, barbell={barbell_mode}, yinyang={yinyang_mode}, hourglass={hourglass_mode}, tornado={tornado_mode}, sandstorm={sandstorm_mode}, ufo={ufo_mode}, ice={ice_mode}")
            elif len(data) >= 11 and not self.is_host:
                # Backwards compatibility with 11-byte format (no ice)
                _, referee_enabled, ball_active, donut_mode, x_stage_mode, barbell_mode, yinyang_mode, hourglass_mode, tornado_mode, sandstorm_mode, ufo_mode = struct.unpack('>BBBBBBBBBBB', data[:11])
                self.pending_game_options = {
                    'referee_enabled': referee_enabled == 1,
                    'ball_active': ball_active == 1,
                    'donut_mode': donut_mode == 1,
                    'x_stage_mode': x_stage_mode == 1,
                    'barbell_mode': barbell_mode == 1,
                    'yinyang_mode': yinyang_mode == 1,
                    'hourglass_mode': hourglass_mode == 1,
                    'tornado_mode': tornado_mode == 1,
                    'sandstorm_mode': sandstorm_mode == 1,
                    'ufo_mode': ufo_mode == 1,
                    'ice_mode': False,
                    'figure8_mode': False,
                    'squiggle_mode': False,
                    'hole_mode': False,
                    'comet_mode': False,
                    'square_mode': False
                }
                print(f"[Network] Received game options: referee={referee_enabled}, ball={ball_active}, donut={donut_mode}, x_stage={x_stage_mode}, barbell={barbell_mode}, yinyang={yinyang_mode}, hourglass={hourglass_mode}, tornado={tornado_mode}, sandstorm={sandstorm_mode}, ufo={ufo_mode}")
            elif len(data) >= 10 and not self.is_host:
                # Backwards compatibility with 10-byte format (no ufo)
                _, referee_enabled, ball_active, donut_mode, x_stage_mode, barbell_mode, yinyang_mode, hourglass_mode, tornado_mode, sandstorm_mode = struct.unpack('>BBBBBBBBBB', data[:10])
                self.pending_game_options = {
                    'referee_enabled': referee_enabled == 1,
                    'ball_active': ball_active == 1,
                    'donut_mode': donut_mode == 1,
                    'x_stage_mode': x_stage_mode == 1,
                    'barbell_mode': barbell_mode == 1,
                    'yinyang_mode': yinyang_mode == 1,
                    'hourglass_mode': hourglass_mode == 1,
                    'tornado_mode': tornado_mode == 1,
                    'sandstorm_mode': sandstorm_mode == 1,
                    'ufo_mode': False,
                    'figure8_mode': False,
                    'squiggle_mode': False,
                    'hole_mode': False,
                    'comet_mode': False,
                    'square_mode': False
                }
                print(f"[Network] Received game options: referee={referee_enabled}, ball={ball_active}, donut={donut_mode}, x_stage={x_stage_mode}, barbell={barbell_mode}, yinyang={yinyang_mode}, hourglass={hourglass_mode}, tornado={tornado_mode}, sandstorm={sandstorm_mode}")
            elif len(data) >= 9 and not self.is_host:
                # Backwards compatibility with 9-byte format (no sandstorm)
                _, referee_enabled, ball_active, donut_mode, x_stage_mode, barbell_mode, yinyang_mode, hourglass_mode, tornado_mode = struct.unpack('>BBBBBBBBB', data[:9])
                self.pending_game_options = {
                    'referee_enabled': referee_enabled == 1,
                    'ball_active': ball_active == 1,
                    'donut_mode': donut_mode == 1,
                    'x_stage_mode': x_stage_mode == 1,
                    'barbell_mode': barbell_mode == 1,
                    'yinyang_mode': yinyang_mode == 1,
                    'hourglass_mode': hourglass_mode == 1,
                    'tornado_mode': tornado_mode == 1,
                    'sandstorm_mode': False,
                    'ufo_mode': False,
                    'figure8_mode': False,
                    'squiggle_mode': False,
                    'hole_mode': False,
                    'comet_mode': False,
                    'square_mode': False
                }
                print(f"[Network] Received game options: referee={referee_enabled}, ball={ball_active}, donut={donut_mode}, x_stage={x_stage_mode}, barbell={barbell_mode}, yinyang={yinyang_mode}, hourglass={hourglass_mode}, tornado={tornado_mode}")
            elif len(data) >= 8 and not self.is_host:
                # Backwards compatibility with 8-byte format (no tornado)
                _, referee_enabled, ball_active, donut_mode, x_stage_mode, barbell_mode, yinyang_mode, hourglass_mode = struct.unpack('>BBBBBBBB', data[:8])
                self.pending_game_options = {
                    'referee_enabled': referee_enabled == 1,
                    'ball_active': ball_active == 1,
                    'donut_mode': donut_mode == 1,
                    'x_stage_mode': x_stage_mode == 1,
                    'barbell_mode': barbell_mode == 1,
                    'yinyang_mode': yinyang_mode == 1,
                    'hourglass_mode': hourglass_mode == 1,
                    'tornado_mode': False,
                    'sandstorm_mode': False,
                    'ufo_mode': False,
                    'figure8_mode': False,
                    'squiggle_mode': False,
                    'hole_mode': False,
                    'comet_mode': False,
                    'square_mode': False
                }
                print(f"[Network] Received game options: referee={referee_enabled}, ball={ball_active}, donut={donut_mode}, x_stage={x_stage_mode}, barbell={barbell_mode}, yinyang={yinyang_mode}, hourglass={hourglass_mode}")
            elif len(data) >= 7 and not self.is_host:
                # Backwards compatibility with 7-byte format (no hourglass)
                _, referee_enabled, ball_active, donut_mode, x_stage_mode, barbell_mode, yinyang_mode = struct.unpack('>BBBBBBB', data[:7])
                self.pending_game_options = {
                    'referee_enabled': referee_enabled == 1,
                    'ball_active': ball_active == 1,
                    'donut_mode': donut_mode == 1,
                    'x_stage_mode': x_stage_mode == 1,
                    'barbell_mode': barbell_mode == 1,
                    'yinyang_mode': yinyang_mode == 1,
                    'hourglass_mode': False,
                    'tornado_mode': False,
                    'sandstorm_mode': False,
                    'ufo_mode': False,
                    'figure8_mode': False,
                    'squiggle_mode': False,
                    'hole_mode': False,
                    'comet_mode': False,
                    'square_mode': False
                }
                print(f"[Network] Received game options: referee={referee_enabled}, ball={ball_active}, donut={donut_mode}, x_stage={x_stage_mode}, barbell={barbell_mode}, yinyang={yinyang_mode}")
            elif len(data) >= 6 and not self.is_host:
                # Backwards compatibility with 6-byte format (no yinyang or hourglass)
                _, referee_enabled, ball_active, donut_mode, x_stage_mode, barbell_mode = struct.unpack('>BBBBBB', data[:6])
                self.pending_game_options = {
                    'referee_enabled': referee_enabled == 1,
                    'ball_active': ball_active == 1,
                    'donut_mode': donut_mode == 1,
                    'x_stage_mode': x_stage_mode == 1,
                    'barbell_mode': barbell_mode == 1,
                    'yinyang_mode': False,
                    'hourglass_mode': False,
                    'tornado_mode': False,
                    'sandstorm_mode': False,
                    'ufo_mode': False,
                    'figure8_mode': False,
                    'squiggle_mode': False,
                    'hole_mode': False,
                    'comet_mode': False,
                    'square_mode': False
                }
                print(f"[Network] Received game options: referee={referee_enabled}, ball={ball_active}, donut={donut_mode}, x_stage={x_stage_mode}, barbell={barbell_mode}")
            elif len(data) >= 5 and not self.is_host:
                # Backwards compatibility with 5-byte format (no barbell, yinyang, or hourglass)
                _, referee_enabled, ball_active, donut_mode, x_stage_mode = struct.unpack('>BBBBB', data[:5])
                self.pending_game_options = {
                    'referee_enabled': referee_enabled == 1,
                    'ball_active': ball_active == 1,
                    'donut_mode': donut_mode == 1,
                    'x_stage_mode': x_stage_mode == 1,
                    'barbell_mode': False,
                    'yinyang_mode': False,
                    'hourglass_mode': False,
                    'tornado_mode': False,
                    'sandstorm_mode': False,
                    'ufo_mode': False,
                    'figure8_mode': False,
                    'squiggle_mode': False,
                    'hole_mode': False,
                    'comet_mode': False,
                    'square_mode': False
                }
                print(f"[Network] Received game options: referee={referee_enabled}, ball={ball_active}, donut={donut_mode}, x_stage={x_stage_mode}")
            elif len(data) >= 4 and not self.is_host:
                # Backwards compatibility with old 4-byte format (no x_stage, barbell, yinyang, or hourglass)
                _, referee_enabled, ball_active, donut_mode = struct.unpack('>BBBB', data[:4])
                self.pending_game_options = {
                    'referee_enabled': referee_enabled == 1,
                    'ball_active': ball_active == 1,
                    'donut_mode': donut_mode == 1,
                    'x_stage_mode': False,
                    'barbell_mode': False,
                    'yinyang_mode': False,
                    'hourglass_mode': False,
                    'tornado_mode': False,
                    'sandstorm_mode': False,
                    'ufo_mode': False,
                    'figure8_mode': False,
                    'squiggle_mode': False,
                    'hole_mode': False,
                    'comet_mode': False,
                    'square_mode': False
                }
                print(f"[Network] Received game options (legacy): referee={referee_enabled}, ball={ball_active}, donut={donut_mode}")

        elif msg_type == MSG_RECONNECT_REQUEST:
            # Guest is reconnecting and requesting full state (host receives)
            if self.is_host:
                self.pending_reconnect_request = True
                print("[Network] Received reconnect request from guest")

        elif msg_type == MSG_RECONNECT_STATE:
            # Host sent full state snapshot (guest receives)
            if not self.is_host and len(data) >= 83:  # 1 + 4 + 36 + 36 + 24 + 1 + 2 + 8 = 112 bytes min
                unpacked = struct.unpack('>BI fffffffff fffffffff ffffff B BB ff', data[:112])
                self.pending_reconnect_state = {
                    'frame': unpacked[1],
                    'blue': {
                        'x': unpacked[2], 'y': unpacked[3], 'z': unpacked[4],
                        'vx': unpacked[5], 'vy': unpacked[6], 'vz': unpacked[7],
                        'rotation': unpacked[8], 'pitch': unpacked[9], 'roll': unpacked[10]
                    },
                    'red': {
                        'x': unpacked[11], 'y': unpacked[12], 'z': unpacked[13],
                        'vx': unpacked[14], 'vy': unpacked[15], 'vz': unpacked[16],
                        'rotation': unpacked[17], 'pitch': unpacked[18], 'roll': unpacked[19]
                    },
                    'ball': {
                        'x': unpacked[20], 'y': unpacked[21], 'z': unpacked[22],
                        'vx': unpacked[23], 'vy': unpacked[24], 'vz': unpacked[25],
                        'active': unpacked[26]
                    },
                    'blue_score': unpacked[27],
                    'red_score': unpacked[28],
                    'blue_respawn_timer': unpacked[29],
                    'red_respawn_timer': unpacked[30]
                }
                print(f"[Network] Received reconnect state: frame={unpacked[1]}")

        elif msg_type == MSG_DISCONNECT:
            # Opponent is leaving gracefully
            self.pending_disconnect = True
            print("[Network] Received disconnect message - opponent leaving")

        elif msg_type == MSG_BALL_EXPLODE:
            # Host says ball exploded (guest receives)
            if len(data) >= 13 and not self.is_host:
                _, pos_x, pos_y, pos_z = struct.unpack('>Bfff', data[:13])
                self.pending_ball_explode = {'x': pos_x, 'y': pos_y, 'z': pos_z}
                print(f"[Network] Received ball explode at ({pos_x:.1f}, {pos_y:.1f}, {pos_z:.1f})")

    # =========================================================================
    # CONNECTION STATE
    # =========================================================================

    def update(self):
        """Call every frame to process Steam callbacks."""
        if self.initialized:
            try:
                self.client.run_callbacks()
            except:
                pass

    def is_connected(self):
        """Check if we have a connected opponent."""
        return self.connected and self.peer_steam_id is not None

    def get_ping(self):
        """Get current ping to opponent in milliseconds."""
        return self.ping_ms

    def get_status(self):
        """Get connection status string for UI."""
        if not self.initialized:
            return "Steam not initialized"
        if not self.in_lobby:
            return "Not in lobby"
        if not self.connected:
            return "Waiting for opponent..."
        if self.match_started:
            return f"Playing ({self.ping_ms}ms)"
        return f"Connected ({self.ping_ms}ms)"


# =============================================================================
# STANDALONE TESTING
# =============================================================================

if __name__ == "__main__":
    import sys

    print("Beetle Battle Steam Network Test")
    print("=" * 50)

    if not STEAM_AVAILABLE:
        print("ERROR: py_steam_net not available")
        print("Make sure:")
        print("  - py_steam_net is installed (Python 3.12)")
        print("  - steam_api64.dll is in this folder")
        print("  - Steam is running")
        sys.exit(1)

    network = NetworkManager()

    if not network.init():
        print("Failed to initialize Steam")
        sys.exit(1)

    print(f"Steam ID: {network.my_steam_id}")
    print()

    # Create or join based on command line
    if len(sys.argv) > 1:
        # Join mode - argument is lobby ID
        lobby_id = int(sys.argv[1])
        print(f"Joining lobby {lobby_id}...")
        network.join_lobby(lobby_id)
    else:
        # Host mode
        print("Creating public lobby...")
        network.create_lobby("public", 2)
        print("Waiting for opponent to join...")
        print("(Share your lobby ID with your friend)")

    print()
    print("Press Ctrl+C to exit")

    try:
        last_status = ""
        while True:
            network.poll_messages()

            status = network.get_status()
            if status != last_status:
                print(f"Status: {status}")
                last_status = status

            # Show lobby members
            members = network.get_lobby_members()
            if members and network.lobby_id:
                pass  # Could print member list periodically

            # If connected, send pings
            if network.connected:
                network.send_ping()

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nShutting down...")

    network.shutdown()
    print("Done!")
