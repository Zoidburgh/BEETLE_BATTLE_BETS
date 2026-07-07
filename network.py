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
import sys
import struct
import time
import random
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


# Protocol version - bump whenever a packet format changes incompatibly.
# Carried in MSG_READY (guest->host) and MSG_START (host->guest); a mismatch
# refuses the match with a clear message instead of desyncing silently.
PROTOCOL_VERSION = 5  # v5: 4-player lobby; game options carry game_mode +
                      # team_of_slot + lives_per_player (reserved for 2v2 and
                      # lives modes - parsed and stored, only mode 0 ships)
                      # v4: N-player - state sync player_count, MSG_INPUTS_ALL, score victim

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
MSG_SLOT_ASSIGN = 0x13        # Host sends player slot roster (N-player support)
MSG_INPUTS_ALL = 0x14         # Host broadcasts all players' inputs for a frame

MAX_PLAYERS = 4

# Messages sent on the unreliable channel (subject to real packet loss,
# and the only ones dropped by the --simloss debug simulator)
UNRELIABLE_MSG_TYPES = {MSG_INPUT, MSG_INPUTS_ALL, MSG_PING, MSG_PONG, MSG_STATE_SYNC, MSG_FRAME_SYNC}

# Short names for the debug HUD / logging
MSG_NAMES = {
    MSG_INPUT: "INPUT", MSG_READY: "READY", MSG_START: "START",
    MSG_HORN_SELECT: "HORN", MSG_REMATCH: "REMATCH", MSG_PING: "PING",
    MSG_PONG: "PONG", MSG_STATE_SYNC: "SYNC", MSG_SYNC_READY: "SYNCRDY",
    MSG_GO: "GO", MSG_FRAME_SYNC: "FRAME", MSG_BEETLE_CONFIG: "CONFIG",
    MSG_SCORE: "SCORE", MSG_GAME_OPTIONS: "OPTIONS",
    MSG_RECONNECT_REQUEST: "RECONREQ", MSG_RECONNECT_STATE: "RECONST",
    MSG_DISCONNECT: "DISCON", MSG_BALL_EXPLODE: "BALLEXP",
    MSG_SLOT_ASSIGN: "SLOT", MSG_INPUTS_ALL: "INPUTSALL",
}

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
        self.peer_steam_id = None  # 2P transitional: the single opponent (host<->guest)
        self.lobby_id = None

        # N-player peer tracking (slots: 0 = host, 1..3 = guests in join order)
        self.peers = {}         # {steam_id: slot} - remote players only
        self.slot_to_steam = {} # {slot: steam_id} - inverse mapping
        self.my_slot = 0        # Our own slot (0 until host assigns otherwise)
        self.player_count = 1   # Total players including us
        self.bot_peers = set()  # Fake steam ids of host-side bots (--bots N);
                                # counted in the roster but never sent to
        self.pending_peer_departures = []  # Slots of guests who left mid-match
                                # (consumed by the game loop; see A8)

        # Match config from v5 game options (only mode 0 = FFA-score exists
        # today; fields reserved so 2v2/lives modes need no protocol bump)
        self.game_mode = 0
        self.team_of_slot = (0, 0, 0, 0)
        self.lives_per_player = 0

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
        self.guest_sync_ready = False  # Host: have ALL real guests sent SYNC_READY?
        self.sync_ready_peers = set()  # Host: which peer ids confirmed SYNC_READY
        self.received_go = False  # Guest: has host sent GO?
        self.go_sent_time = 0  # Host: when GO was sent (to delay start by one-way latency)

        # Frame sync (keeps guest frame counter aligned with host)
        self.target_frame = None  # Guest: frame counter we should be at (from host)

        # Beetle customization sync
        self.remote_beetle_config = None  # Legacy single-opponent slot (unused; kept for safety)
        self.remote_beetle_configs = {}   # {slot: config dict} - all remote players' builds

        # Score sync (host-authoritative death/goal detection)
        # Guest: QUEUE of score events from host. Must be a list - two events
        # can arrive in one poll (simultaneous deaths) and a single slot drops one
        self.pending_scores = []

        # Game options sync (referee, ball, etc.)
        self.pending_game_options = None  # Guest: pending options from host

        # Reconnection sync
        self.pending_reconnect_request = False  # Host: guest requesting full state
        self.pending_reconnect_state = None  # Guest: full state from host
        self.pending_disconnect = False  # Opponent gracefully leaving

        # Ball explosion sync (host-authoritative)
        self.pending_ball_explode = None  # Guest: ball explosion event from host

        # Protocol version check (set when peer runs an incompatible build)
        self.version_mismatch = False

        # Wall-clock disconnect detection (works without lockstep waits)
        self.last_packet_received_time = time.time()

        # Packet statistics for the net debug HUD (N key while online)
        self.pkt_sent = {}       # msg_type -> total count
        self.pkt_recv = {}       # msg_type -> total count
        self.pkt_sent_bytes = 0
        self.pkt_recv_bytes = 0
        self.pkt_rates = {'in_pps': 0.0, 'out_pps': 0.0, 'in_bps': 0.0, 'out_bps': 0.0}
        self._rate_window_start = time.time()
        self._rate_snapshot = (0, 0, 0, 0)  # (sent_count, recv_count, sent_bytes, recv_bytes)

        # Network condition simulator (debug): --simlag <ms> --simloss <pct>
        # Lag delays ALL received packets; loss drops only unreliable ones
        # (reliable messages survive real packet loss via retransmission).
        self.sim_lag_ms = 0.0
        self.sim_loss_pct = 0.0
        self._sim_queue = []  # [(deliver_time, data, sender_id), ...]
        for i, arg in enumerate(sys.argv):
            if arg == '--simlag' and i + 1 < len(sys.argv):
                try:
                    self.sim_lag_ms = max(0.0, float(sys.argv[i + 1]))
                except ValueError:
                    pass
            elif arg == '--simloss' and i + 1 < len(sys.argv):
                try:
                    self.sim_loss_pct = min(100.0, max(0.0, float(sys.argv[i + 1])))
                except ValueError:
                    pass
        if self.sim_lag_ms > 0 or self.sim_loss_pct > 0:
            print(f"[Network] SIMULATING degraded network: +{self.sim_lag_ms:.0f}ms lag, {self.sim_loss_pct:.0f}% loss on unreliable msgs")

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
                    self._register_peer(member)
                    self.connected = True
                    print(f"[Network] Found opponent: {member}")
                    if self.on_peer_joined:
                        self.on_peer_joined()
                    # Guest sends MSG_READY immediately to establish bidirectional P2P channel
                    # (carries our protocol version so the host can detect old builds)
                    if not self.is_host:
                        self._send_packet(struct.pack('>BB', MSG_READY, PROTOCOL_VERSION), reliable=True)
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
        self.peers = {}
        self.slot_to_steam = {}
        self.my_slot = 0
        self.player_count = 1
        self.bot_peers = set()
        self.sync_ready_peers = set()
        self.remote_beetle_configs = {}
        self.pending_peer_departures = []  # Slots of guests who left mid-match
        self.match_started = False
        self.local_ready = False
        self.remote_ready = False

    # =========================================================================
    # PEER / SLOT TRACKING (N-player support; slots: 0 = host, 1-3 = guests)
    # =========================================================================

    def _register_peer(self, steam_id):
        """Track a remote player; the host assigns the next free slot."""
        if steam_id in self.peers or steam_id == self.my_steam_id:
            return
        if self.is_host:
            used = set(self.peers.values()) | {0}
            free = [s for s in range(1, MAX_PLAYERS) if s not in used]
            if not free:
                print(f"[Network] Lobby full - ignoring peer {steam_id}")
                return
            slot = free[0]
        else:
            slot = 0  # From a guest's view the direct peer is the host
        self.peers[steam_id] = slot
        self.slot_to_steam[slot] = steam_id
        self.player_count = len(self.peers) + 1
        if self.peer_steam_id is None:
            self.peer_steam_id = steam_id  # 2P transitional single-opponent field
        print(f"[Network] Registered peer {steam_id} as slot {slot} ({self.player_count} players)")
        if self.is_host:
            self.send_slot_assign()

    def _unregister_peer(self, steam_id):
        """Remove a remote player (left/kicked/disconnected)."""
        if steam_id not in self.peers:
            return
        slot = self.peers.pop(steam_id)
        if self.slot_to_steam.get(slot) == steam_id:
            del self.slot_to_steam[slot]
        self.player_count = len(self.peers) + 1
        if self.peer_steam_id == steam_id:
            self.peer_steam_id = next(iter(self.peers), None)
        print(f"[Network] Unregistered peer {steam_id} (slot {slot}, {self.player_count} players left)")
        if self.is_host:
            self.send_slot_assign()

    def slot_for_sender(self, sender_id):
        """Resolve a received packet's sender to a player slot (None if unknown)."""
        if sender_id is not None and sender_id in self.peers:
            return self.peers[sender_id]
        if not self.is_host:
            return 0  # Guests only ever hear from the host
        if len(self.peers) == 1:
            return next(iter(self.peers.values()))  # Sole guest (sender_id missing)
        return None

    def send_slot_assign(self):
        """Host sends the full roster to every guest (each told its own slot).
        Packet: [type:1][your_slot:1][player_count:1] + [slot:1][steam64:8] per player."""
        if not self.is_host:
            return
        real_guests = 0
        for steam_id, slot in self.peers.items():
            if steam_id in self.bot_peers:
                continue  # Roster CONTENT includes bots; never send TO them
            data = struct.pack('>BBB', MSG_SLOT_ASSIGN, slot, self.player_count)
            data += struct.pack('>BQ', 0, self.my_steam_id)  # Host is always slot 0
            for other_id, other_slot in self.peers.items():
                data += struct.pack('>BQ', other_slot, other_id)
            try:
                self.client.send_message_to(steam_id, SEND_RELIABLE, GAME_CHANNEL, data)
                self.pkt_sent[MSG_SLOT_ASSIGN] = self.pkt_sent.get(MSG_SLOT_ASSIGN, 0) + 1
                self.pkt_sent_bytes += len(data)
                real_guests += 1
            except Exception as e:
                print(f"[Network] slot assign send error: {e}")
        print(f"[Network] Sent slot roster to {real_guests} guest(s) ({self.player_count} players incl. bots)")

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
                self._register_peer(member_id)
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
            if member_id in self.peers:
                _left_slot = self.peers.get(member_id)
                self._unregister_peer(member_id)
                self.connected = bool(self.peers)
                print(f"[Network] Peer left (slot {_left_slot})")
                if _left_slot is not None and _left_slot != 0:
                    self.pending_peer_departures.append(_left_slot)
                # Match-over escalation only when it actually is over: for a
                # guest that means the HOST left; for the host it means no
                # real guests (and no bots) remain. A guest leaving a 4P
                # match must NOT end it for the others.
                if self.is_host:
                    match_over = not (self.has_real_guests() or self.bot_peers)
                else:
                    match_over = (_left_slot == 0)
                if match_over and self.on_peer_left:
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

    def send_inputs_all(self, frame, slot_bits):
        """
        Host broadcasts every player's input for a frame to all guests, so
        guests predict ALL beetles with the same last-known-input model.
        Packet: [type:1][frame:4][count:1] + [slot:1][bits:1] per player (~11B at 4P)
        """
        if not self.is_host or not self.peers:
            return
        parts = [struct.pack('>BIB', MSG_INPUTS_ALL, frame, len(slot_bits))]
        for slot, bits in slot_bits:
            parts.append(struct.pack('>BB', slot, bits))
        self._broadcast(b''.join(parts), reliable=False)

    def send_ready(self):
        """Signal that local player is ready to start."""
        self.local_ready = True
        self._send_packet(struct.pack('>BB', MSG_READY, PROTOCOL_VERSION), reliable=True)
        print("[Network] Sent ready signal")

        # If both ready and we're host, start the match
        if self.is_host and self.local_ready and self.remote_ready:
            self._send_start()

    # Fake steam id for --phantom-peer broadcast testing
    PHANTOM_PEER_ID = 0xDEADBEEF

    # Reserved fake steam id range for host-side network bots (--bots N).
    # Must never collide with real ids; excluded from all sends and from
    # SYNC_READY waits, but counted in the roster real guests receive.
    BOT_PEER_ID_BASE = 0xB0700001

    def register_bot_peers(self):
        """Host registers --bots N fake guests at match start (AFTER real
        guests, so humans keep low slots; lobby roster stays humans-only
        until now). Their inputs are injected through _handle_packet each
        frame by the main loop, so slot routing, last-known fallback and
        MSG_INPUTS_ALL aggregation all exercise production code."""
        if not self.is_host:
            return
        bots = 0
        for i, arg in enumerate(sys.argv):
            if arg == '--bots' and i + 1 < len(sys.argv):
                try:
                    bots = max(0, min(3, int(sys.argv[i + 1])))
                except ValueError:
                    bots = 0
        for b in range(bots):
            fake_id = self.BOT_PEER_ID_BASE + b
            if fake_id in self.peers:
                continue
            if self.player_count >= MAX_PLAYERS:
                print(f"[Network] BOTS: lobby full, skipping bot {b + 1}")
                break
            self.bot_peers.add(fake_id)  # Mark BEFORE registering so the
            # roster send triggered inside _register_peer skips this id
            print(f"[Network] BOT PEER: registering bot {b + 1} (--bots {bots})")
            self._register_peer(fake_id)

    def has_real_guests(self):
        """Any connected peers that are not host-side bots?"""
        return any(pid not in self.bot_peers for pid in self.peers)

    def start_match_now(self):
        """Host immediately starts the match (skip ready handshake for now).
        With --bots and no real guest connected this starts a SOLO BOT MATCH:
        the full online code path (roster, input packets, state sync attempts)
        runs on one machine - the single-machine test rig for 4P plumbing."""
        solo_bots = ('--bots' in sys.argv) and not self.connected
        if self.is_host and (self.connected or solo_bots):
            if '--phantom-peer' in sys.argv and self.PHANTOM_PEER_ID not in self.peers:
                # Debug: a fake guest that receives everything and answers
                # nothing - exercises multi-peer broadcast/roster paths with
                # only one real guest connected
                print("[Network] PHANTOM PEER: registering fake guest (--phantom-peer)")
                self._register_peer(self.PHANTOM_PEER_ID)
            self.register_bot_peers()
            self._send_start()
            if not self.has_real_guests():
                # Nobody real to wait for - skip the SYNC_READY handshake
                print("[Network] Solo bot match: no real guests, sending GO immediately")
                self.send_go()

    def send_horn_select(self, horn_type):
        """Send horn type selection to opponent."""
        self.local_horn = horn_type

        if not self.connected:
            return

        horn_ids = {"rhino": 0, "stag": 1, "hercules": 2, "scorpion": 3, "atlas": 4, "bombardier": 5, "spider": 6, "giraffe": 7}
        horn_id = horn_ids.get(horn_type, 0)
        self._send_packet(struct.pack('>BB', MSG_HORN_SELECT, horn_id), reliable=True)

    def send_rematch_request(self):
        """Request a rematch."""
        self._send_packet(struct.pack('>B', MSG_REMATCH), reliable=True)

    def send_state_sync(self, frame, beetle_states, ball_state):
        """
        Host sends authoritative state to guest (protocol v3).

        Args:
            frame: physics frame number
            beetle_states: list of per-beetle dicts (blue first, then red) with
                x, y, z, rot, pitch, roll, vx, vy, vz,
                active (bool), is_falling (bool)
            ball_state: dict with x, y, z, vx, vy, vz, active (bool)

        Packet (v4): [type:1][frame:4][player_count:1]
                per beetle: [x,y,z,rot,pitch,roll,vx,vy,vz (9*f32)][flags:1] = 37 bytes
                            flags bit0 = active, bit1 = is_falling
                ball: [x,y,z,vx,vy,vz (6*f32)][active:1] = 25 bytes
        Total: 6 + 37*N + 25 (= 105 bytes for 2 players)
        """
        if not self.is_host or not self.connected:
            return

        parts = [struct.pack('>BIB', MSG_STATE_SYNC, frame, len(beetle_states))]
        for b in beetle_states:
            flags = (1 if b['active'] else 0) | (2 if b['is_falling'] else 0)
            parts.append(struct.pack('>fffffffffB',
                                     b['x'], b['y'], b['z'],
                                     b['rot'], b['pitch'], b['roll'],
                                     b['vx'], b['vy'], b['vz'], flags))
        parts.append(struct.pack('>ffffffB',
                                 ball_state['x'], ball_state['y'], ball_state['z'],
                                 ball_state['vx'], ball_state['vy'], ball_state['vz'],
                                 1 if ball_state['active'] else 0))
        self._broadcast(b''.join(parts), reliable=False)  # Unreliable is fine for periodic sync

    def send_sync_ready(self):
        """Guest sends SYNC_READY to host to confirm ready to start."""
        print("[Network] Sending SYNC_READY to host")
        self._send_packet(struct.pack('>B', MSG_SYNC_READY), reliable=True)

    def send_go(self):
        """Host sends GO to start simulation on all clients."""
        if not self.is_host:
            return
        if self.sync_state in ("waiting_for_go_delay", "go"):
            return  # Already going (late SYNC_READY duplicates are fine)
        print("[Network] Sending GO - all players start now!")
        # Send multiple times for reliability
        for _ in range(3):
            self._broadcast(struct.pack('>B', MSG_GO), reliable=True)
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
        self._broadcast(data, reliable=False)

    def send_beetle_config(self, horn_type_id, shaft, prong, back_body, body_len, body_width, leg_len,
                           body_color, leg_color, leg_tip_color, stripe_color, horn_tip_color):
        """Send our beetle customization to all peers, tagged with our slot.
        The host relays received guest configs to the other guests (see the
        MSG_BEETLE_CONFIG handler), so every player sees every build."""
        player_id = self.my_slot
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
        self._broadcast(data, reliable=True)
        print(f"[Network] Sent beetle config (slot {player_id}): horn={horn_type_id}, sizes={shaft}/{prong}/{back_body}/{body_len}/{body_width}/{leg_len}")

    def send_score(self, scorer, score_type=0, death_x=0.0, death_z=0.0, victim=None):
        """
        Host sends authoritative score event to guests.

        Args:
            scorer: player slot that scored (0-3)
            score_type: 0 = beetle death (explode beetle), 1 = ball goal (don't explode beetle)
            death_x, death_z: Position where beetle died (for smooth death animation on guest)
            victim: player slot that died / was scored on (defaults to the
                other player at 2P; 255 = unknown/none)
        """
        if not self.is_host or not self.connected:
            return
        if victim is None:
            victim = 1 - scorer if scorer in (0, 1) else 255
        data = struct.pack('>BBBBff', MSG_SCORE, scorer, victim, score_type, death_x, death_z)
        self._broadcast(data, reliable=True)
        type_str = "death" if score_type == 0 else "ball goal"
        print(f"[Network] Sent score event: slot {scorer} scores ({type_str}), victim slot {victim} at ({death_x:.1f}, {death_z:.1f})")

    def send_game_options(self, referee_enabled, ball_active, donut_mode=False, x_stage_mode=False, barbell_mode=False, yinyang_mode=False, hourglass_mode=False, tornado_mode=False, sandstorm_mode=False, ufo_mode=False, ice_mode=False, figure8_mode=False, squiggle_mode=False, hole_mode=False, comet_mode=False, square_mode=False, cut_square_mode=False, board_break_mode=False, star_mode=False,
                          game_mode=0, team_of_slot=(0, 0, 0, 0), lives_per_player=0):
        """
        Host sends game options to guest.
        Packet (v5, 23 bytes): [type:1] + 19 mode flags + [game_mode:1]
        [team_of_slot packed 2 bits/slot:1] [lives_per_player:1]
        game_mode: 0=FFA-score (reserved: 1=FFA-lives, 2=2v2-score, 3=2v2-ball)
        The trailing 3 fields are parsed/stored but only mode 0 ships today —
        they exist so 2v2/lives modes are handler work, not a protocol bump.
        """
        if not self.is_host or not self.connected:
            return
        # 3-4P matches are FFA-LIVES (mode 1) today; auto-derive so existing
        # call sites (which don't pass the mode kwargs) stay correct
        if game_mode == 0 and self.player_count > 2:
            game_mode = 1
            if lives_per_player == 0:
                lives_per_player = 3
        teams_packed = ((team_of_slot[0] & 3) | ((team_of_slot[1] & 3) << 2) |
                        ((team_of_slot[2] & 3) << 4) | ((team_of_slot[3] & 3) << 6))
        data = struct.pack('>BBBBBBBBBBBBBBBBBBBBBBB', MSG_GAME_OPTIONS,
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
                          1 if square_mode else 0,
                          1 if cut_square_mode else 0,
                          1 if board_break_mode else 0,
                          1 if star_mode else 0,
                          game_mode & 0xFF,
                          teams_packed,
                          lives_per_player & 0xFF)
        self._broadcast(data, reliable=True)

    def send_ball_explode(self, pos_x, pos_y, pos_z):
        """
        Host tells guest the ball has exploded (host-authoritative).
        Packet format: [type:1][x:4][y:4][z:4] = 13 bytes
        """
        if not self.is_host or not self.connected:
            return
        data = struct.pack('>Bfff', MSG_BALL_EXPLODE, pos_x, pos_y, pos_z)
        self._broadcast(data, reliable=True)
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
        # Pump Steam callbacks briefly so the message actually transmits
        # before the caller tears the connection down (otherwise the peer
        # never learns we left gracefully and has to hit the 3s timeout)
        for _ in range(6):
            try:
                self.client.run_callbacks()
            except Exception:
                pass
            time.sleep(0.03)
        print("[Network] Sent disconnect message")

    def send_ping(self):
        """Send ping to measure latency."""
        # Use lower 32 bits of milliseconds to fit in uint32
        self.ping_sent_time = int(time.time() * 1000) & 0xFFFFFFFF
        self._send_packet(struct.pack('>BI', MSG_PING, self.ping_sent_time), reliable=False)

    def _send_start(self):
        """Host sends match start signal - waits for guest SYNC_READY before GO."""
        self.random_seed = random.randint(0, 2**32 - 1)
        self.start_frame = 0

        # [type:1][start_frame:4][seed:4][protocol_version:1]
        data = struct.pack('>BIIB', MSG_START, self.start_frame, self.random_seed, PROTOCOL_VERSION)

        # Send multiple times to ensure delivery (P2P channel may still be establishing)
        for i in range(3):
            self._broadcast(data, reliable=True)
            time.sleep(0.05)  # Small delay between sends

        # Don't start yet - wait for all real guests to confirm ready
        self.sync_ready_peers = set()
        self.guest_sync_ready = False
        self.sync_state = "waiting_for_guest"
        self.match_started = True  # Match is "started" but not simulating yet
        print(f"[Network] START sent, waiting for guest SYNC_READY... Seed: {self.random_seed}")

    def _broadcast(self, data, reliable=True):
        """Send raw packet to ALL connected peers (host -> every guest).
        Bot peers are skipped (fake ids; sends to them would error)."""
        if not self.peers:
            return False
        send_type = SEND_RELIABLE if reliable else SEND_UNRELIABLE
        ok = True
        real_sends = 0
        for steam_id in self.peers:
            if steam_id in self.bot_peers:
                continue
            try:
                self.client.send_message_to(steam_id, send_type, GAME_CHANNEL, data)
                real_sends += 1
            except Exception as e:
                print(f"[Network] Broadcast error to {steam_id}: {e}")
                ok = False
        if data and real_sends:
            self.pkt_sent[data[0]] = self.pkt_sent.get(data[0], 0) + real_sends
            self.pkt_sent_bytes += len(data) * real_sends
        return ok

    def _send_packet(self, data, reliable=True):
        """Send raw packet to peer via Steam P2P."""
        if not self.peer_steam_id:
            print(f"[Network] Send failed: no peer_steam_id")
            return False

        try:
            send_type = SEND_RELIABLE if reliable else SEND_UNRELIABLE
            # Only log non-INPUT messages (INPUT is too spammy at 60/sec)
            if data and data[0] != 0x01:  # Not INPUT
                msg_name = MSG_NAMES.get(data[0], f"0x{data[0]:02x}")
                print(f"[Network] Sending {msg_name}")
            self.client.send_message_to(self.peer_steam_id, send_type, GAME_CHANNEL, data)
            if data:
                self.pkt_sent[data[0]] = self.pkt_sent.get(data[0], 0) + 1
                self.pkt_sent_bytes += len(data)
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

        # Update packet in/out rates once per second (for debug HUD)
        now = time.time()
        if now - self._rate_window_start >= 1.0:
            elapsed = now - self._rate_window_start
            sent_count = sum(self.pkt_sent.values())
            recv_count = sum(self.pkt_recv.values())
            s0, r0, sb0, rb0 = self._rate_snapshot
            self.pkt_rates = {
                'out_pps': (sent_count - s0) / elapsed,
                'in_pps': (recv_count - r0) / elapsed,
                'out_bps': (self.pkt_sent_bytes - sb0) / elapsed,
                'in_bps': (self.pkt_recv_bytes - rb0) / elapsed,
            }
            self._rate_snapshot = (sent_count, recv_count, self.pkt_sent_bytes, self.pkt_recv_bytes)
            self._rate_window_start = now

        # Deliver simulated-lag packets whose time has come (--simlag debug)
        if self._sim_queue:
            due = [m for m in self._sim_queue if m[0] <= now]
            if due:
                self._sim_queue = [m for m in self._sim_queue if m[0] > now]
                for _, data, sender_id in due:
                    try:
                        self._handle_packet(data, sender_id, input_buffer)
                    except Exception as e:
                        print(f"[Network] Error handling delayed packet: {e}")

        # Run Steam callbacks (don't skip based on is_ready - always try)
        try:
            self.client.run_callbacks()
        except Exception as e:
            print(f"[Network] run_callbacks error: {e}")

        # FALLBACK: If we're in a lobby but haven't detected opponent, check member list directly
        # This handles cases where the lobby_changed callback doesn't fire.
        # The host also rescans every ~2s so late joiners (players 3/4) are caught.
        if self.in_lobby and self.lobby_id and (
                not self.connected or (self.is_host and self.poll_count % 120 == 0)):
            try:
                members = self.client.get_lobby_members(self.lobby_id)
                for member in members:
                    if member != self.my_steam_id and member not in self.peers:
                        self._register_peer(member)
                        self.connected = True
                        print(f"[Network] Found opponent via polling: {member}")

                        # Send ping immediately to establish bidirectional P2P channel
                        if self.is_host:
                            print("[Network] Host sending ping to establish reverse P2P channel...")
                            self.send_ping()

                        if self.on_peer_joined:
                            self.on_peer_joined()
            except:
                pass

        # Explicitly receive messages from Steam
        try:
            result = self.client.receive_messages(GAME_CHANNEL, 100)
            # Check if receive_messages returns messages directly (vs callback)
            if result and isinstance(result, list) and len(result) > 0:
                for msg in result:
                    if hasattr(msg, 'data'):
                        self._receive_packet(msg.data, getattr(msg, 'sender', None), input_buffer)
                    elif isinstance(msg, tuple) and len(msg) >= 2:
                        sender_id, data = msg[0], msg[-1]
                        self._receive_packet(bytes(data) if not isinstance(data, bytes) else data, sender_id, input_buffer)
                    elif isinstance(msg, bytes):
                        self._receive_packet(msg, None, input_buffer)
        except Exception as e:
            print(f"[Network] receive_messages error: {e}")

        # Process queued messages (from callback, if it works)
        messages = []
        with self.message_lock:
            messages = self.message_queue[:]
            self.message_queue.clear()

        for sender_id, channel, data in messages:
            try:
                self._receive_packet(data, sender_id, input_buffer)
            except Exception as e:
                print(f"[Network] Error handling packet: {e}")

    def _receive_packet(self, data, sender_id, input_buffer):
        """
        Entry point for all received packets.
        Applies the debug network-condition simulator (--simlag/--simloss),
        then hands off to _handle_packet.
        """
        if not data:
            return
        if self.sim_loss_pct > 0 and data[0] in UNRELIABLE_MSG_TYPES:
            if random.random() * 100.0 < self.sim_loss_pct:
                return  # Simulated packet loss
        if self.sim_lag_ms > 0:
            self._sim_queue.append((time.time() + self.sim_lag_ms / 1000.0, data, sender_id))
            return
        self._handle_packet(data, sender_id, input_buffer)

    def _handle_packet(self, data, sender_id, input_buffer):
        """Process a received network packet."""
        if len(data) < 1:
            return

        msg_type = data[0]
        self.last_packet_received_time = time.time()
        self.pkt_recv[msg_type] = self.pkt_recv.get(msg_type, 0) + 1
        self.pkt_recv_bytes += len(data)

        if msg_type == MSG_INPUT:
            # Input packet: [type (1)] [frame (4)] [inputs (1)]
            # Sender's player slot is resolved from its Steam ID (authoritative,
            # unspoofable - the packet itself carries no slot byte)
            if len(data) >= 6:
                _, frame, inputs = struct.unpack('>BIB', data[:6])
                self.inputs_received += 1
                if input_buffer:
                    # Debug: log frame mismatch periodically
                    if self.inputs_received % 60 == 1:
                        print(f"[Network] Frame check: received={frame}, local={input_buffer.current_frame}, diff={input_buffer.current_frame - frame}")
                    slot = self.slot_for_sender(sender_id)
                    if slot is not None:
                        input_buffer.add_remote(slot, frame, inputs)

        elif msg_type == MSG_INPUTS_ALL:
            # Host broadcast of all players' inputs: [type][frame:4][count] + [slot][bits]*
            if len(data) >= 6 and not self.is_host and input_buffer:
                _, frame, count = struct.unpack('>BIB', data[:6])
                offset = 6
                for _i in range(count):
                    if offset + 2 > len(data):
                        break
                    slot, bits = data[offset], data[offset + 1]
                    offset += 2
                    if slot != self.my_slot:
                        input_buffer.add_remote(slot, frame, bits)

        elif msg_type == MSG_READY:
            # [type:1][protocol_version:1] - old builds send just [type:1]
            peer_version = data[1] if len(data) >= 2 else 0
            if peer_version != PROTOCOL_VERSION:
                self.version_mismatch = True
                print(f"[Network] PROTOCOL VERSION MISMATCH: ours={PROTOCOL_VERSION}, peer={peer_version} - update both builds!")
            self.remote_ready = True
            print(f"[Network] Opponent is ready")

            # If both ready and we're host, start the match
            if self.is_host and self.local_ready and self.remote_ready and not self.version_mismatch:
                self._send_start()

        elif msg_type == MSG_START:
            # Start packet: [type:1][frame:4][seed:4][protocol_version:1]
            # (old builds send 9 bytes without the version byte)
            if len(data) >= 9:
                host_version = data[9] if len(data) >= 10 else 0
                if host_version != PROTOCOL_VERSION:
                    self.version_mismatch = True
                    print(f"[Network] PROTOCOL VERSION MISMATCH: ours={PROTOCOL_VERSION}, host={host_version} - refusing match. Update both builds!")
                    return
                _, self.start_frame, self.random_seed = struct.unpack('>BII', data[:9])
                self.match_started = True
                print(f"[Network] Match starting! Frame: {self.start_frame}, Seed: {self.random_seed}")

                if self.on_match_start:
                    self.on_match_start(self.start_frame, self.random_seed)

        elif msg_type == MSG_HORN_SELECT:
            # Horn select: [type (1)] [horn_id (1)]
            if len(data) >= 2:
                horn_id = data[1]
                horn_names = {0: "rhino", 1: "stag", 2: "hercules", 3: "scorpion", 4: "atlas", 5: "bombardier", 6: "spider", 7: "giraffe"}
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
            # Protocol v4: [type:1][frame:4][player_count:1] + Nx beetle(37B) + ball(25B)
            if len(data) >= 6 and not self.is_host:
                _, frame, sync_count = struct.unpack('>BIB', data[:6])
                if len(data) < 6 + 37 * sync_count + 25:
                    return  # Truncated packet
                offset = 6
                beetles = []
                for _i in range(sync_count):
                    bx, by, bz, brot, bpitch, broll, bvx, bvy, bvz, flags = struct.unpack('>fffffffffB', data[offset:offset + 37])
                    beetles.append({
                        'x': bx, 'y': by, 'z': bz,
                        'rot': brot, 'pitch': bpitch, 'roll': broll,
                        'vx': bvx, 'vy': bvy, 'vz': bvz,
                        'active': bool(flags & 1), 'is_falling': bool(flags & 2),
                    })
                    offset += 37
                ball_x, ball_y, ball_z, ball_vx, ball_vy, ball_vz, ball_active = struct.unpack('>ffffffB', data[offset:offset + 25])
                self.pending_state_sync = {
                    'frame': frame,
                    'beetles': beetles,
                    'ball': {'x': ball_x, 'y': ball_y, 'z': ball_z,
                             'vx': ball_vx, 'vy': ball_vy, 'vz': ball_vz,
                             'active': ball_active == 1},
                }

        elif msg_type == MSG_SYNC_READY:
            # Guest is ready to start - host sends GO once ALL real guests
            # have confirmed (bots never send SYNC_READY and don't count)
            if self.is_host:
                real_guests = [pid for pid in self.peers if pid not in self.bot_peers]
                if sender_id is None and len(real_guests) == 1:
                    sender_id = real_guests[0]  # sole-guest packets may lack sender id
                if sender_id is not None:
                    self.sync_ready_peers.add(sender_id)
                ready = sum(1 for pid in real_guests if pid in self.sync_ready_peers)
                print(f"[Network] SYNC_READY received ({ready}/{len(real_guests)} real guests ready)")
                self.guest_sync_ready = ready >= len(real_guests)
                if self.guest_sync_ready:
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
                self.remote_beetle_configs[player_id] = {
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
                self.remote_beetle_configs[player_id] = {
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
            # Host relays guest configs to the other real guests so every
            # player sees every build (bots excluded; sender excluded)
            if self.is_host and len(data) >= 9:
                for _pid in self.peers:
                    if _pid == sender_id or _pid in self.bot_peers:
                        continue
                    try:
                        self.client.send_message_to(_pid, SEND_RELIABLE, GAME_CHANNEL, data)
                    except Exception as e:
                        print(f"[Network] Config relay error to {_pid}: {e}")

        elif msg_type == MSG_SCORE:
            # Host-authoritative score event (guest receives)
            # v4: [type][scorer:1][victim:1][score_type:1][death_x:4][death_z:4]
            if len(data) >= 12 and not self.is_host:
                _, scorer, victim, score_type, death_x, death_z = struct.unpack('>BBBBff', data[:12])
                self.pending_scores.append({'scorer': scorer, 'victim': victim, 'score_type': score_type,
                                            'death_x': death_x, 'death_z': death_z})
                type_str = "death" if score_type == 0 else "ball goal"
                print(f"[Network] Received score event: slot {scorer} scores ({type_str}), victim slot {victim} at ({death_x:.1f}, {death_z:.1f})")

        elif msg_type == MSG_GAME_OPTIONS:
            # Host sends game options (guest receives). v5: 23 bytes with
            # trailing game_mode / packed teams / lives (stored for future
            # 2v2 and lives modes; only mode 0 exists today)
            if len(data) >= 23 and not self.is_host:
                _, referee_enabled, ball_active, donut_mode, x_stage_mode, barbell_mode, yinyang_mode, hourglass_mode, tornado_mode, sandstorm_mode, ufo_mode, ice_mode, figure8_mode, squiggle_mode, hole_mode, comet_mode, square_mode, cut_square_mode, board_break_mode, star_mode = struct.unpack('>BBBBBBBBBBBBBBBBBBBB', data[:20])
                game_mode = data[20]
                teams_packed = data[21]
                lives_per_player = data[22]
                self.game_mode = game_mode
                self.team_of_slot = tuple((teams_packed >> (2 * s)) & 3 for s in range(4))
                self.lives_per_player = lives_per_player
                self.pending_game_options = {
                    'game_mode': game_mode,
                    'team_of_slot': self.team_of_slot,
                    'lives_per_player': lives_per_player,
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
                    'square_mode': square_mode == 1,
                    'cut_square_mode': cut_square_mode == 1,
                    'board_break_mode': board_break_mode == 1,
                    'star_mode': star_mode == 1
                }
                print(f"[Network] Received game options: referee={referee_enabled}, ball={ball_active}, donut={donut_mode}, x_stage={x_stage_mode}, barbell={barbell_mode}, yinyang={yinyang_mode}, hourglass={hourglass_mode}, tornado={tornado_mode}, sandstorm={sandstorm_mode}, ufo={ufo_mode}, ice={ice_mode}, figure8={figure8_mode}, squiggle={squiggle_mode}, hole={hole_mode}, comet={comet_mode}, square={square_mode}, cut_square={cut_square_mode}, board_break={board_break_mode}, star={star_mode}")

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
            # A peer is leaving gracefully. Resolve WHO: for the host a guest
            # leaving a 4P match must not end it; for a guest any disconnect
            # is from the host (match over)
            _dc_slot = self.slot_for_sender(sender_id)
            if self.is_host and _dc_slot is not None and _dc_slot != 0:
                if sender_id in self.peers:
                    self._unregister_peer(sender_id)
                self.connected = bool(self.peers)
                self.pending_peer_departures.append(_dc_slot)
                if self.has_real_guests() or self.bot_peers:
                    print(f"[Network] Guest slot {_dc_slot} left gracefully - match continues")
                else:
                    self.pending_disconnect = True
                    print("[Network] Received disconnect message - opponent leaving")
            else:
                self.pending_disconnect = True
                print("[Network] Received disconnect message - opponent leaving")

        elif msg_type == MSG_BALL_EXPLODE:
            # Host says ball exploded (guest receives)
            if len(data) >= 13 and not self.is_host:
                _, pos_x, pos_y, pos_z = struct.unpack('>Bfff', data[:13])
                self.pending_ball_explode = {'x': pos_x, 'y': pos_y, 'z': pos_z}
                print(f"[Network] Received ball explode at ({pos_x:.1f}, {pos_y:.1f}, {pos_z:.1f})")

        elif msg_type == MSG_SLOT_ASSIGN:
            # Host-assigned roster: [type][your_slot][player_count] + [slot][steam64] per player
            if not self.is_host and len(data) >= 3:
                self.my_slot = data[1]
                self.player_count = data[2]
                offset = 3
                self.slot_to_steam = {}
                roster_peers = {}
                while offset + 9 <= len(data):
                    slot, steam_id = struct.unpack('>BQ', data[offset:offset + 9])
                    offset += 9
                    self.slot_to_steam[slot] = steam_id
                    if steam_id != self.my_steam_id:
                        roster_peers[steam_id] = slot
                self.peers = roster_peers
                # Guests talk to the host directly (slot 0)
                self.peer_steam_id = self.slot_to_steam.get(0, self.peer_steam_id)
                print(f"[Network] Slot assigned: we are slot {self.my_slot} of {self.player_count} players")

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
        if self.version_mismatch:
            return "VERSION MISMATCH - update both builds!"
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
