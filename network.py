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
- steam_appid.txt with app ID (480 for testing)
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

        # Message buffer (filled by callbacks, processed by poll_messages)
        self.message_queue = []
        self.message_lock = threading.Lock()

        # Callbacks (set by game code)
        self.on_peer_joined = None
        self.on_peer_left = None
        self.on_match_start = None
        self.on_horn_selected = None

    def init(self, app_id=480):
        """
        Initialize Steam networking.

        Args:
            app_id: Steam App ID (480 = Spacewar for testing)

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

    def _on_lobby_created(self, lobby_id):
        """Callback when lobby is created."""
        if lobby_id:
            self.lobby_id = lobby_id
            self.in_lobby = True
            print(f"[Network] Lobby created! ID: {lobby_id}")
        else:
            print(f"[Network] Lobby creation failed")
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

    def _on_lobby_changed(self, lobby_id, member_id, change_type):
        """
        Called when lobby membership changes.

        change_type: "joined", "left", "disconnected", etc.
        """
        print(f"[Network] Lobby changed: {change_type} - member {member_id}")

        if change_type == "joined" or change_type == "entered":
            if member_id != self.my_steam_id:
                self.peer_steam_id = member_id
                self.connected = True
                self.lobby_id = lobby_id
                print(f"[Network] Opponent joined: {member_id}")
                if self.on_peer_joined:
                    self.on_peer_joined()

        elif change_type in ["left", "disconnected", "kicked", "banned"]:
            if member_id == self.peer_steam_id:
                self.peer_steam_id = None
                self.connected = False
                print(f"[Network] Opponent left")
                if self.on_peer_left:
                    self.on_peer_left()

    def _on_message_received(self, sender_id, channel, data):
        """Called when a P2P message is received."""
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

    def send_ping(self):
        """Send ping to measure latency."""
        self.ping_sent_time = int(time.time() * 1000)
        self._send_packet(struct.pack('>BI', MSG_PING, self.ping_sent_time), reliable=False)

    def _send_start(self):
        """Host sends match start signal with frame sync and random seed."""
        import random
        self.random_seed = random.randint(0, 2**32 - 1)
        self.start_frame = 0

        data = struct.pack('>BII', MSG_START, self.start_frame, self.random_seed)
        self._send_packet(data, reliable=True)

        self.match_started = True
        print(f"[Network] Match starting! Seed: {self.random_seed}")
        if self.on_match_start:
            self.on_match_start(self.start_frame, self.random_seed)

    def _send_packet(self, data, reliable=True):
        """Send raw packet to peer via Steam P2P."""
        if not self.peer_steam_id:
            return False

        try:
            send_type = SEND_RELIABLE if reliable else SEND_UNRELIABLE
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

        # Run Steam callbacks
        try:
            self.client.run_callbacks()
        except:
            pass

        # Process queued messages
        messages = []
        with self.message_lock:
            messages = self.message_queue[:]
            self.message_queue.clear()

        for sender_id, channel, data in messages:
            self._handle_packet(data, sender_id, input_buffer)

    def _handle_packet(self, data, sender_id, input_buffer):
        """Process a received network packet."""
        if len(data) < 1:
            return

        msg_type = data[0]

        if msg_type == MSG_INPUT:
            # Input packet: [type (1)] [frame (4)] [inputs (1)]
            if len(data) >= 6:
                _, frame, inputs = struct.unpack('>BIB', data[:6])
                if input_buffer:
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
                self.ping_ms = int(time.time() * 1000) - sent_time

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
