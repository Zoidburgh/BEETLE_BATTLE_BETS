# Beetle Battle RL Training Guide

## Table of Contents
1. [Introduction - What is Reinforcement Learning?](#1-introduction---what-is-reinforcement-learning)
2. [How RL Works for Beetle Battles](#2-how-rl-works-for-beetle-battles)
3. [The Training Loop - Step by Step](#3-the-training-loop---step-by-step)
4. [PPO Algorithm Explained](#4-ppo-algorithm-explained)
5. [Integration Architecture](#5-integration-architecture)
6. [Implementation Options](#6-implementation-options)
7. [Technical Specifications](#7-technical-specifications)
8. [Code Examples](#8-code-examples)
9. [Training Progression](#9-training-progression)
10. [Player Experience Design](#10-player-experience-design)

---

## 1. Introduction - What is Reinforcement Learning?

### The Core Concept

**Reinforcement Learning (RL)** is teaching an AI agent to make decisions through trial and error, just like training a pet:

- **Good behavior** → Reward (treat for the dog)
- **Bad behavior** → Punishment (no treat)
- **Repeat** → Agent learns what actions lead to rewards

In beetle battles:
- **Good behavior** = Pushing opponent toward edge, staying in center, winning fights
- **Bad behavior** = Falling off edge, getting pushed around, losing
- **Repeat** = Beetle gets better at combat over thousands of practice matches

### Key Terminology

| Term | Definition | Beetle Battle Example |
|------|------------|----------------------|
| **Agent** | The AI being trained | Your beetle's "brain" |
| **Environment** | The world the agent interacts with | The arena, physics, opponent |
| **State/Observation** | What the agent can see | Beetle position, opponent location, velocities |
| **Action** | What the agent can do | Move forward, turn left, tilt horn |
| **Reward** | Feedback on how good an action was | +10 for winning, -1 for falling off |
| **Policy** | The agent's decision-making strategy | Neural network that chooses actions |
| **Episode** | One complete match from start to finish | One beetle battle (30 seconds) |

### Why RL for Beetle Battles?

**Traditional approach (hand-coded AI):**
```python
def beetle_ai(beetle, opponent):
    if distance_to_opponent < 10:
        if horn_angle < 20:
            tilt_horn_up()
        else:
            charge_forward()
    elif distance_to_edge < 5:
        turn_toward_center()
    # ... hundreds of if-else rules
```
❌ Hard to get right
❌ Brittle and predictable
❌ Doesn't adapt to new strategies

**RL approach:**
```python
def beetle_ai(beetle, opponent):
    observation = get_state(beetle, opponent)
    action = neural_network(observation)  # Learned from experience
    return action
```
✅ Learns automatically
✅ Adapts to opponent strategies
✅ Discovers creative tactics
✅ Players can train unique beetles

---

## 2. How RL Works for Beetle Battles

### The Observation-Action-Reward Loop

```
     ┌─────────────────────────────────────────────────────┐
     │                  TRAINING CYCLE                     │
     │                                                     │
     │   ┌──────────────┐                                 │
     │   │  ENVIRONMENT │ (Taichi Physics Simulation)     │
     │   │  • Arena     │                                 │
     │   │  • 2 Beetles │                                 │
     │   │  • Physics   │                                 │
     │   └──────┬───────┘                                 │
     │          │                                          │
     │          │ (1) OBSERVATION                          │
     │          │ "What do I see?"                         │
     │          ▼                                          │
     │   ┌──────────────┐                                 │
     │   │  POLICY      │ (Neural Network)                │
     │   │  "Brain"     │ Input: [19 numbers]             │
     │   │              │ Output: [3 numbers]             │
     │   │              │ (thrust, turn, horn)            │
     │   └──────┬───────┘                                 │
     │          │                                          │
     │          │ (2) ACTION                               │
     │          │ "What should I do?"                      │
     │          ▼                                          │
     │   ┌──────────────┐                                 │
     │   │  PHYSICS     │                                 │
     │   │  • Apply     │                                 │
     │   │    forces    │                                 │
     │   │  • Simulate  │                                 │
     │   │    collision │                                 │
     │   └──────┬───────┘                                 │
     │          │                                          │
     │          │ (3) REWARD                               │
     │          │ "How did I do?"                          │
     │          ▼                                          │
     │   ┌──────────────┐                                 │
     │   │  LEARNING    │                                 │
     │   │  "Update the │                                 │
     │   │   neural net"│                                 │
     │   └──────────────┘                                 │
     │          │                                          │
     │          └──────────┐                               │
     │                     │                               │
     │          Repeat 10,000+ times                       │
     │                     │                               │
     │          ┌──────────┘                               │
     │          │                                          │
     │          ▼                                          │
     │   ┌──────────────┐                                 │
     │   │ TRAINED AI   │                                 │
     │   │ (Smart       │                                 │
     │   │  Beetle)     │                                 │
     │   └──────────────┘                                 │
     └─────────────────────────────────────────────────────┘
```

### What the Beetle "Sees" (Observation)

The observation is a list of 19 numbers that describe the current situation:

```python
observation = [
    # Self state (12 numbers)
    beetle.x,           # My X position (-32 to 32)
    beetle.z,           # My Z position (-32 to 32)
    beetle.y,           # My height (0 to 10)
    beetle.vx,          # My X velocity (-7 to 7)
    beetle.vz,          # My Z velocity (-7 to 7)
    beetle.vy,          # My Y velocity (-10 to 10)
    beetle.rotation,    # Which way I'm facing (-π to π)
    beetle.pitch,       # Forward/back tilt (-π/4 to π/4)
    beetle.roll,        # Left/right tilt (-π/4 to π/4)
    beetle.angular_vel, # How fast I'm turning (-8 to 8)
    beetle.pitch_vel,   # Tilt change speed
    beetle.roll_vel,    # Roll change speed

    # Horn state (1 number)
    beetle.horn_pitch,  # Horn angle (-10° to 30°)

    # Opponent info (4 numbers)
    dist_to_opponent,   # How far away (0 to 64)
    angle_to_opponent,  # Which direction (-π to π)
    opponent.vx - beetle.vx,  # Relative velocity X
    opponent.vz - beetle.vz,  # Relative velocity Z

    # Arena info (1 number)
    dist_to_edge,       # How close to falling (0 to 32)
]
```

**Think of it like the beetle's senses:**
- Position/velocity = "Where am I and how fast am I moving?"
- Opponent info = "Where is my enemy and what are they doing?"
- Arena info = "Am I close to danger?"

### What the Beetle Does (Action)

The policy (neural network) outputs 3 continuous numbers:

```python
action = [
    thrust,    # -1.0 to +1.0 (backward to forward)
    turn,      # -1.0 to +1.0 (left to right)
    horn_tilt, # -1.0 to +1.0 (down to up)
]
```

These get converted to physics forces:

```python
# Example: action = [0.8, -0.3, 0.5]
# Means: "Move mostly forward, turn slightly left, tilt horn up a bit"

# Applied to physics:
forward_force = 0.8 * MOVE_FORCE  # 96 Newtons forward
turning_speed = -0.3 * ROTATION_SPEED  # -0.9 rad/s left turn
horn_change = 0.5 * HORN_TILT_SPEED  # 1.0 rad/s upward tilt
```

### How the Beetle Learns (Reward)

After each action, the environment gives a reward signal:

```python
def calculate_reward(beetle, opponent):
    reward = 0.0

    # Survive each frame
    reward += 0.01

    # Stay in center (safe)
    dist_from_center = sqrt(beetle.x**2 + beetle.z**2)
    if dist_from_center < ARENA_RADIUS * 0.5:
        reward += 0.1  # "Good, you're safe"
    elif dist_from_center > ARENA_RADIUS * 0.8:
        reward -= 0.5  # "Danger! Get back!"

    # Push opponent toward edge (offensive)
    opp_dist_from_center = sqrt(opponent.x**2 + opponent.z**2)
    if opp_dist_from_center > ARENA_RADIUS * 0.8:
        reward += 0.3  # "Nice! They're in danger"

    # Engage with opponent (don't run away)
    dist_to_opponent = sqrt((beetle.x - opponent.x)**2 +
                           (beetle.z - opponent.z)**2)
    if dist_to_opponent < 20.0:
        reward += 0.2  # "Good, you're fighting"

    # Win/loss (terminal)
    if not beetle.active:
        reward -= 10.0  # "You lost!"
    if not opponent.active:
        reward += 10.0  # "You won!"

    return reward
```

**The beetle learns:** "Actions that led to high total reward are good, repeat them!"

---

## 3. The Training Loop - Step by Step

### Single Episode Walkthrough

```
EPISODE START (t=0)
├─ Reset arena
│  └─ Blue beetle at (-20, 0), facing right
│  └─ Red beetle at (+20, 0), facing left
│
├─ STEP 1 (t=1/60 sec)
│  ├─ Observation: [beetle at -20, opponent at +20, ...]
│  ├─ Policy decides: action = [0.9, 0.0, 0.2] (charge forward)
│  ├─ Physics simulates 1/60 second
│  ├─ New state: beetle at (-19.8, 0), moving forward
│  └─ Reward: +0.11 (survival + center + engagement)
│
├─ STEP 2 (t=2/60 sec)
│  ├─ Observation: [beetle at -19.8, opponent at +19.5, ...]
│  ├─ Policy decides: action = [1.0, -0.1, 0.3] (charge, slight left)
│  ├─ Physics simulates 1/60 second
│  ├─ New state: beetle at (-19.5, 0.05), moving faster
│  └─ Reward: +0.15 (getting closer to opponent)
│
├─ ... (repeat for 1800 steps = 30 seconds)
│
├─ STEP 1234 (t=20.5 sec)
│  ├─ Observation: [beetle at +15, opponent at +28, ...]
│  ├─ Policy decides: action = [1.0, 0.2, 0.9] (charge with horn up)
│  ├─ Physics: COLLISION! Horn hits opponent, pushes them
│  ├─ New state: opponent at +31 (near edge!)
│  └─ Reward: +0.8 (big bonus for pushing opponent to edge)
│
├─ STEP 1235 (t=20.56 sec)
│  ├─ Observation: [beetle at +16, opponent at +32.5, ...]
│  ├─ Physics: Opponent falls off! (y < -30)
│  ├─ Episode ends: YOU WIN
│  └─ Reward: +10.0 (victory bonus)
│
EPISODE END
├─ Total reward collected: +34.7
└─ Total steps: 1235
```

### What Happens After the Episode?

The episode data is stored:

```python
episode_data = {
    'observations': [obs_1, obs_2, ..., obs_1235],  # 1235 x 19 numbers
    'actions': [act_1, act_2, ..., act_1235],       # 1235 x 3 numbers
    'rewards': [0.11, 0.15, ..., 10.0],             # 1235 numbers
}
```

Then the **learning algorithm** (PPO) analyzes this:

1. **Which actions led to victory?**
   - Step 1234's aggressive charge was great (+10.0 eventual reward)
   - Earlier positioning in center was good (+0.1 each step)
   - Running away from opponent was bad (low engagement rewards)

2. **Update the neural network**
   - Make "charge when opponent is near edge" more likely
   - Make "stay in center" more likely
   - Make "run away" less likely

3. **Run another episode**
   - Policy is now slightly better
   - Repeat 10,000+ times
   - Beetle gets progressively smarter

### Batch Training (Multiple Episodes)

In practice, we collect multiple episodes before updating:

```
TRAINING ITERATION 1
├─ Run 16 episodes in parallel
│  ├─ Episode 1: Lost at step 234 (total reward: -5.3)
│  ├─ Episode 2: Won at step 1456 (total reward: +23.1)
│  ├─ Episode 3: Lost at step 678 (total reward: -2.8)
│  ├─ ... (13 more episodes)
│  └─ Episode 16: Won at step 1123 (total reward: +18.7)
│
├─ Combine all data (16 * ~1000 steps = ~16,000 data points)
│
├─ Compute advantages (how much better each action was than average)
│  └─ "Charging in Episode 2 was +15.3 better than expected"
│  └─ "Running away in Episode 1 was -8.2 worse than expected"
│
├─ Update neural network with PPO algorithm
│  └─ Adjust weights to favor good actions
│
└─ Win rate this iteration: 6/16 = 37.5%

TRAINING ITERATION 2
├─ Run 16 more episodes (policy is now slightly smarter)
├─ Win rate: 8/16 = 50%
└─ Continue...

TRAINING ITERATION 100
├─ Run 16 more episodes (policy is now much smarter)
├─ Win rate: 13/16 = 81.25%
└─ Beetle has learned advanced tactics!
```

---

## 4. PPO Algorithm Explained

### What is PPO?

**PPO (Proximal Policy Optimization)** is the learning algorithm that updates the neural network. It's currently the most popular RL algorithm because it's:
- **Stable** - Doesn't have wild learning swings
- **Sample-efficient** - Learns from data multiple times
- **Simple** - Relatively easy to implement

### The Problem PPO Solves

**Naive approach:**
```python
# Bad idea: update policy too aggressively
if action_was_good:
    make_action_MUCH_more_likely()  # ← Can break the policy!
```

This causes **catastrophic forgetting**: The policy changes so much it forgets everything it learned before.

**PPO's solution:**
```python
# Good idea: limit how much the policy can change
if action_was_good:
    make_action_more_likely()
    but_not_too_much()  # ← Keeps learning stable
```

### PPO in Simple Terms

Imagine teaching a beetle to fight:

**Without PPO:**
- Beetle wins once by luck (random charge worked)
- You say "ALWAYS DO THAT!" and overwrite all its knowledge
- Beetle now only knows how to charge blindly
- Beetle loses to enemies who sidestep
- You lost all progress

**With PPO:**
- Beetle wins once by luck
- You say "That charge was good, do it 10% more often in that situation"
- Beetle retains other behaviors (turning, positioning, etc.)
- Gradually learns when charging is good vs bad
- Becomes well-rounded fighter

### PPO Math (Simplified)

Don't worry about implementing this - JAX libraries handle it. But here's what's happening:

```python
def ppo_update(policy, episode_data):
    # 1. Calculate "advantage" (how good each action was)
    advantages = []
    for step in episode_data:
        # "If I took action A, how much better was the outcome
        #  compared to the average outcome from this state?"
        advantage = actual_return - predicted_value
        advantages.append(advantage)

    # 2. Calculate policy ratio (how much did policy change?)
    old_prob = old_policy.probability(action)
    new_prob = new_policy.probability(action)
    ratio = new_prob / old_prob

    # 3. Clip the ratio (prevent huge changes)
    if advantage > 0:  # Good action
        # Don't make it more than 20% more likely
        clipped_ratio = min(ratio, 1.2)
    else:  # Bad action
        # Don't make it more than 20% less likely
        clipped_ratio = max(ratio, 0.8)

    # 4. Update policy to increase good actions (within limits)
    loss = -min(ratio * advantage, clipped_ratio * advantage)
    policy.update(loss)
```

**Key insight:** The `min()` operation ensures the policy doesn't change too much in one update.

### PPO Hyperparameters

These control how learning works:

| Parameter | What it does | Typical value | Effect if too high | Effect if too low |
|-----------|--------------|---------------|-------------------|------------------|
| **Learning rate** | How big each update is | 3e-4 | Policy changes too fast, unstable | Learning too slow |
| **Clip epsilon** | How much policy can change | 0.2 | No effect (defeats purpose) | Too conservative, slow learning |
| **Gamma (γ)** | How much to value future rewards | 0.99 | Focuses on long-term (sometimes too much) | Focuses on immediate rewards only |
| **GAE lambda (λ)** | Advantage estimation smoothing | 0.95 | High variance (noisy) | High bias (inaccurate) |
| **Epochs** | How many times to learn from data | 4 | Overfitting to old data | Underutilizing data |

For beetle battles, default values work well. Players don't need to touch these.

---

## 5. Integration Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    BEETLE BATTLE RL SYSTEM                  │
└─────────────────────────────────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
        ▼                   ▼                   ▼
┌───────────────┐   ┌───────────────┐   ┌───────────────┐
│   EXISTING    │   │   NEW RL      │   │   PLAYER      │
│   TAICHI      │   │   TRAINING    │   │   INTERFACE   │
│   PHYSICS     │   │   SYSTEM      │   │               │
└───────────────┘   └───────────────┘   └───────────────┘
        │                   │                   │
        │                   │                   │
┌───────┴────────┐  ┌───────┴────────┐  ┌───────┴────────┐
│ beetle_physics │  │ rl/environment │  │ UI/training    │
│ simulation.py  │  │ rl/policy.py   │  │ UI/competition │
│ renderer.py    │  │ rl/trainer.py  │  │ UI/leaderboard │
└────────────────┘  └────────────────┘  └────────────────┘
```

### Data Flow: Training Mode

```
┌────────────────────────────────────────────────────────────┐
│  1. INITIALIZATION                                         │
│     Player clicks "Train Beetle" → Create new policy       │
└──────────────────────────┬─────────────────────────────────┘
                           │
                           ▼
┌────────────────────────────────────────────────────────────┐
│  2. EPISODE LOOP (Python)                                  │
│     for episode in range(10000):                           │
└──────────────────────────┬─────────────────────────────────┘
                           │
        ┌──────────────────┴──────────────────┐
        │                                     │
        ▼                                     ▼
┌─────────────────┐                  ┌─────────────────┐
│  3. TAICHI SIM  │                  │  4. JAX POLICY  │
│  (GPU/CPU)      │                  │  (GPU)          │
│                 │                  │                 │
│  Step physics   │◄────────────────►│  Choose actions │
│  60 Hz          │   NumPy arrays   │  from obs       │
│  1800 steps     │                  │  Return actions │
└─────────────────┘                  └─────────────────┘
        │                                     │
        │                                     │
        ▼                                     ▼
┌────────────────────────────────────────────────────────────┐
│  5. COLLECT DATA                                           │
│     observations: [1800, 19]  # All states                 │
│     actions:      [1800, 3]   # All actions                │
│     rewards:      [1800]      # All rewards                │
└──────────────────────────┬─────────────────────────────────┘
                           │
                           ▼
┌────────────────────────────────────────────────────────────┐
│  6. PPO UPDATE (JAX on GPU)                                │
│     - Compute advantages                                   │
│     - Update policy 4 epochs                               │
│     - Save new weights                                     │
└──────────────────────────┬─────────────────────────────────┘
                           │
                           └──────────┐
                                      │
                           ┌──────────┘
                           │
                           ▼
┌────────────────────────────────────────────────────────────┐
│  7. REPEAT                                                 │
│     Beetle gets smarter each episode                       │
│     Every 100 episodes: Save checkpoint                    │
│     Every 10 episodes: Update UI progress                  │
└────────────────────────────────────────────────────────────┘
```

### Data Flow: Competition Mode

```
┌────────────────────────────────────────────────────────────┐
│  1. LOAD MODELS                                            │
│     player_beetle.pkl → Policy A (neural network)          │
│     opponent_beetle.pkl → Policy B (neural network)        │
└──────────────────────────┬─────────────────────────────────┘
                           │
                           ▼
┌────────────────────────────────────────────────────────────┐
│  2. BATTLE LOOP                                            │
│     for step in range(1800):  # 30 seconds                 │
└──────────────────────────┬─────────────────────────────────┘
                           │
        ┌──────────────────┴──────────────────┐
        │                                     │
        ▼                                     ▼
┌─────────────────┐                  ┌─────────────────┐
│  3. BOTH AIs    │                  │  4. TAICHI SIM  │
│  DECIDE         │                  │                 │
│                 │                  │                 │
│  obs_blue  →    │                  │  Apply both     │
│  Policy A  →    │──── actions ────►│  actions        │
│  action_blue    │                  │  Simulate       │
│                 │                  │  physics        │
│  obs_red   →    │                  │  Check winner   │
│  Policy B  →    │                  │                 │
│  action_red     │                  │                 │
└─────────────────┘                  └─────────────────┘
        │                                     │
        │                                     │
        └──────────────┬──────────────────────┘
                       │
                       ▼
┌────────────────────────────────────────────────────────────┐
│  5. RESULT                                                 │
│     Winner: Policy A (Blue beetle)                         │
│     Update ELO ratings                                     │
│     Display battle replay                                  │
└────────────────────────────────────────────────────────────┘
```

### The Taichi-JAX Bridge

This is the critical technical piece:

```python
# ============================================================
# THE BRIDGE: Converting between Taichi and JAX
# ============================================================

# TAICHI → NUMPY → JAX (observation)
def get_observation_for_jax(beetle, opponent):
    # Extract data from Taichi fields/objects
    obs_list = [
        beetle.x, beetle.z, beetle.y,
        beetle.vx, beetle.vz, beetle.vy,
        # ... (19 total values)
    ]

    # Convert to NumPy (CPU memory)
    obs_numpy = np.array(obs_list, dtype=np.float32)

    # Convert to JAX (can be GPU memory)
    obs_jax = jax.numpy.array(obs_numpy)

    return obs_jax


# JAX → NUMPY → TAICHI (action)
def apply_jax_action_to_taichi(beetle, action_jax):
    # Convert JAX to NumPy (may involve GPU → CPU copy)
    action_numpy = np.array(action_jax)

    # Extract individual actions
    thrust = float(action_numpy[0])
    turn = float(action_numpy[1])
    horn_tilt = float(action_numpy[2])

    # Apply to Taichi physics (existing code)
    forward_x = math.cos(beetle.rotation)
    forward_z = math.sin(beetle.rotation)
    beetle.apply_force(
        forward_x * thrust * MOVE_FORCE,
        forward_z * thrust * MOVE_FORCE,
        dt
    )
    beetle.angular_velocity += turn * ROTATION_SPEED * dt
    beetle.horn_pitch_velocity = horn_tilt * HORN_TILT_SPEED
```

**Performance Notes:**
- CPU↔GPU copies take ~0.5-2ms per transfer
- For training: Totally fine (not real-time)
- For inference: Also fine (2ms << 16ms frame budget)
- Optimization possible later if needed (PyTorch bridge, custom kernels)

---

## 6. Implementation Options

### Option A: Simple Sequential Training

**What it is:** Train one beetle against one opponent, one episode at a time.

```
Training Loop:
  Episode 1 → Collect data → Update policy
  Episode 2 → Collect data → Update policy
  Episode 3 → Collect data → Update policy
  ... (10,000 episodes)
```

**Pros:**
- Easiest to implement (~1 week)
- Simple to debug
- No complex parallelization
- Good starting point

**Cons:**
- Slower (1-2 episodes/second)
- Underutilizes GPU
- Longer training time

**Training Time:**
- 10,000 episodes × 0.5 sec/episode = **1.4 hours**
- Acceptable for initial training

**Code Complexity:** ⭐ (1/5 stars)

---

### Option B: Vectorized Parallel Training

**What it is:** Run 16 episodes simultaneously, update policy on batched data.

```
Training Iteration:
  ┌─ Episode 1  ─┐
  ├─ Episode 2  ─┤
  ├─ Episode 3  ─┤
  ├─ ...        ─┤  → Collect all data → Update policy once
  ├─ Episode 15 ─┤
  └─ Episode 16 ─┘
  (All run in parallel)
```

**Pros:**
- 16x faster data collection
- Better GPU utilization
- Matches research best practices
- More diverse training data

**Cons:**
- Requires modifying Taichi for batching
- More complex state management
- Harder to debug

**Training Time:**
- 10,000 episodes ÷ 16 parallel × 0.5 sec = **5 minutes**
- Great for iterating quickly

**Code Complexity:** ⭐⭐⭐ (3/5 stars)

**Implementation:**
```python
# Allocate 16 separate arena grids
@ti.kernel
def init_batch_simulation(num_envs: int):
    for env_id in range(num_envs):
        # Each env gets its own voxel grid region
        grid_offset_x = (env_id % 4) * n_grid
        grid_offset_z = (env_id // 4) * n_grid
        # Initialize arena at offset location
```

---

### Option C: Self-Play with Policy Pool

**What it is:** Train multiple versions of the policy, have them fight each other.

```
Policy Pool: [V1, V2, V3, V4, V5]

Training:
  V5 (current) vs V3 (older) → V5 learns from past self
  V5 vs V1 (oldest) → V5 learns to beat early strategies
  V5 vs V4 (recent) → V5 learns cutting-edge tactics

  Update V5 → Save as V6
  Add V6 to pool, remove V1
```

**Pros:**
- Opponent diversity (fights different styles)
- Emergent strategies (no scripted behaviors)
- Prevents overfitting to single opponent
- Creates "evolution" of tactics

**Cons:**
- More complex bookkeeping
- Requires checkpoint management
- Slower initial learning (weak opponents at start)

**Training Time:**
- Similar to Option B, but needs more episodes (20,000+)
- **10-15 minutes** for good results

**Code Complexity:** ⭐⭐⭐⭐ (4/5 stars)

---

### Option D: Curriculum Learning (Progressive Difficulty)

**What it is:** Start with easy opponents, gradually increase difficulty.

```
Stage 1 (Episodes 0-1000): Stationary opponent
  → Learn: "Don't fall off, push forward"

Stage 2 (Episodes 1000-3000): Random movement opponent
  → Learn: "Track moving target, time attacks"

Stage 3 (Episodes 3000-7000): Scripted aggressive opponent
  → Learn: "Defense, counter-attacks, positioning"

Stage 4 (Episodes 7000+): Self-play
  → Learn: "Advanced tactics, adaptation"
```

**Pros:**
- Fastest learning overall
- Each stage builds on previous
- Guaranteed progression
- Good for player experience (visible improvement)

**Cons:**
- Must implement multiple opponent types
- Manual stage design
- Need to tune stage thresholds

**Training Time:**
- **7-10 minutes** for full curriculum
- Most efficient learning

**Code Complexity:** ⭐⭐⭐ (3/5 stars)

---

### Recommendation Matrix

| Your Priority | Best Option | Why |
|---------------|-------------|-----|
| **Get working ASAP** | Option A | Simple, 1 week, good enough |
| **Best performance** | Option B + D | Fast training, great results |
| **Most interesting AI** | Option C | Emergent diverse strategies |
| **Player experience** | Option D | Visible progression, satisfying |
| **Research/learning** | Option A → B → C | Progressive complexity |

**My recommendation:** Start with **Option A** (1 week), then upgrade to **Option D** (1-2 more weeks) once basics work.

---

## 7. Technical Specifications

### Neural Network Architecture

```
INPUT LAYER (19 neurons)
  │
  │  (Observation: position, velocity, opponent info, etc.)
  │
  ▼
HIDDEN LAYER 1 (64 neurons)
  │
  │  [Dense layer + ReLU activation]
  │  Learns: Basic feature combinations
  │  Example: "When close to edge AND moving fast → danger"
  │
  ▼
HIDDEN LAYER 2 (64 neurons)
  │
  │  [Dense layer + ReLU activation]
  │  Learns: Complex patterns
  │  Example: "Opponent near edge + I'm behind them → charge opportunity"
  │
  ├──────────────────┬──────────────────┐
  │                  │                  │
  ▼                  ▼                  ▼
ACTOR HEAD      ACTOR HEAD        CRITIC HEAD
(Action Mean)   (Action Std)      (Value)
  │                  │                  │
  │  3 neurons       │  3 params        │  1 neuron
  │  (thrust,        │  (noise          │  (state
  │   turn,          │   level)         │   value)
  │   horn)          │                  │
  │                  │                  │
  ▼                  ▼                  ▼
[Tanh]          [Exp]             [Linear]
-1 to +1        >0 (std dev)      -∞ to +∞
```

**Total Parameters:**
- Input → Hidden1: 19 × 64 = 1,216
- Hidden1 → Hidden2: 64 × 64 = 4,096
- Hidden2 → Actor mean: 64 × 3 = 192
- Hidden2 → Critic: 64 × 1 = 64
- Actor log_std: 3 (learned per-action noise)
- Biases: ~200
- **Total: ~5,771 parameters (~23 KB)**

This is tiny! For comparison:
- GPT-3: 175 billion parameters
- DALL-E: 12 billion parameters
- Beetle AI: **0.000006 billion parameters**

Small network = fast training, fast inference, easy to save/load.

### Observation Space Details

| Index | Feature | Range | Normalization | Why Important? |
|-------|---------|-------|---------------|----------------|
| 0 | Self X | -32 to +32 | ÷ 32 | Avoid edge on X axis |
| 1 | Self Z | -32 to +32 | ÷ 32 | Avoid edge on Z axis |
| 2 | Self Y | 0 to 10 | ÷ 10 | Detect if being lifted |
| 3 | Self VX | -7 to +7 | ÷ 7 | Control momentum X |
| 4 | Self VZ | -7 to +7 | ÷ 7 | Control momentum Z |
| 5 | Self VY | -10 to +10 | ÷ 10 | Detect falling/jumping |
| 6 | Self Rotation | -π to +π | ÷ π | Which direction facing |
| 7 | Self Pitch | -π/4 to +π/4 | ÷ 0.785 | Am I tilted forward? |
| 8 | Self Roll | -π/4 to +π/4 | ÷ 0.785 | Am I tilted sideways? |
| 9 | Self Ang Vel | -8 to +8 | ÷ 8 | How fast turning |
| 10 | Self Pitch Vel | -8 to +8 | ÷ 8 | Tilting speed |
| 11 | Self Roll Vel | -8 to +8 | ÷ 8 | Rolling speed |
| 12 | Horn Pitch | -0.17 to +0.52 | ÷ 0.7 | Horn angle |
| 13 | Opp Distance | 0 to 64 | ÷ 64 | How far to target |
| 14 | Opp Angle | -π to +π | ÷ π | Which direction to target |
| 15 | Opp Rel VX | -14 to +14 | ÷ 14 | Opponent velocity relative to me |
| 16 | Opp Rel VZ | -14 to +14 | ÷ 14 | Opponent velocity relative to me |
| 17 | Edge Distance | 0 to 32 | ÷ 32 | How close to falling off |
| 18 | (Reserved) | - | - | Future use (health, stamina, etc.) |

**Normalization** is critical - neural networks work best with inputs in range [-1, +1].

### Action Space Details

| Index | Action | Range | Effect | Resolution |
|-------|--------|-------|--------|------------|
| 0 | Thrust | -1.0 to +1.0 | Forward/backward force | Continuous |
| 1 | Turn | -1.0 to +1.0 | Rotation speed | Continuous |
| 2 | Horn Tilt | -1.0 to +1.0 | Horn angle velocity | Continuous |

**Why continuous?** Gives beetle fine motor control:
- `thrust = 0.3` → gentle push
- `thrust = 1.0` → full charge
- `turn = -0.15` → slight course correction
- `turn = -1.0` → hard turn

Compare to discrete (8 actions):
- Forward, Backward, Left, Right, Horn Up, Horn Down, Forward+Horn Up, etc.
- Less expressive, harder to learn smooth movement

### Reward Function Options

#### Option 1: Sparse (Win/Loss Only)

```python
def reward_sparse(beetle, opponent):
    if not beetle.active:
        return -1.0  # Lost
    if not opponent.active:
        return +1.0  # Won
    return 0.0      # Still fighting
```

**Learning curve:** Slow (1000+ episodes to see progress)
**Pros:** Unbiased, no reward shaping artifacts
**Cons:** Hard credit assignment (which actions caused win?)

---

#### Option 2: Dense (Recommended)

```python
def reward_dense(beetle, opponent):
    r = 0.0

    # Survival (small constant)
    r += 0.01  # "Every frame alive is good"

    # Arena positioning (safety)
    dist_center = sqrt(beetle.x**2 + beetle.z**2)
    if dist_center < ARENA_RADIUS * 0.5:
        r += 0.1  # Safe zone
    elif dist_center > ARENA_RADIUS * 0.8:
        r -= 0.5  # Danger zone (strong penalty)

    # Opponent positioning (offense)
    opp_dist_center = sqrt(opponent.x**2 + opponent.z**2)
    if opp_dist_center > ARENA_RADIUS * 0.8:
        r += 0.3  # Opponent in danger (reward!)

    # Engagement (don't run away)
    dist_opp = distance(beetle, opponent)
    if dist_opp < 20.0:
        r += 0.2  # Close combat

    # Facing opponent (aim matters)
    angle_to_opp = atan2(opponent.z - beetle.z,
                         opponent.x - beetle.x)
    angle_diff = abs(normalize_angle(angle_to_opp - beetle.rotation))
    if angle_diff < pi/4:  # Within 45° cone
        r += 0.15  # Good aim

    # Terminal rewards (big!)
    if not beetle.active:
        r -= 10.0  # Lost
    if not opponent.active:
        r += 10.0  # Won

    return r
```

**Learning curve:** Fast (100-200 episodes for basic behavior)
**Pros:** Clear feedback, faster learning
**Cons:** Risk of "reward hacking" (exploiting shaped rewards)

**Example reward hacking:** If engagement reward is too high, beetle might just circle opponent without attacking.

---

#### Option 3: Curriculum Rewards

```python
def reward_curriculum(beetle, opponent, stage):
    if stage == 1:  # Survival stage
        return reward_survival_only(beetle)
    elif stage == 2:  # Positioning stage
        return reward_positioning(beetle)
    elif stage == 3:  # Combat stage
        return reward_dense(beetle, opponent)
    elif stage == 4:  # Mastery stage
        return reward_sparse(beetle, opponent)

def reward_survival_only(beetle):
    # Just stay alive and in bounds
    if not beetle.active:
        return -1.0
    dist = sqrt(beetle.x**2 + beetle.z**2)
    return 0.1 if dist < ARENA_RADIUS * 0.8 else -0.1

def reward_positioning(beetle):
    # Survival + face opponent
    r = reward_survival_only(beetle)
    # ... add facing reward
    return r
```

**Learning curve:** Fastest overall (50-100 episodes per stage)
**Pros:** Guaranteed progression, best final quality
**Cons:** More engineering, need to design stages

---

### Training Hyperparameters

| Parameter | Value | Explanation |
|-----------|-------|-------------|
| **Learning rate** | 3e-4 | Standard for PPO; controls update size |
| **Discount (γ)** | 0.99 | Value future rewards at 99% of current |
| **GAE lambda (λ)** | 0.95 | Smoothing for advantage estimation |
| **Clip epsilon** | 0.2 | Max 20% policy change per update |
| **Value coefficient** | 0.5 | Weight for value function loss |
| **Entropy coefficient** | 0.01 | Exploration bonus (randomness) |
| **Max grad norm** | 0.5 | Clip gradients to prevent exploding |
| **Num epochs** | 4 | Update 4 times per batch of data |
| **Num minibatches** | 4 | Split data into 4 chunks for updating |
| **Batch size** | 2048 | Collect 2048 timesteps before updating |

**For players:** These are hidden! No need to expose to UI.

**For developers:** These defaults work well, minimal tuning needed.

---

## 8. Code Examples

### File: `rl/environment.py`

```python
"""
Reinforcement Learning Environment for Beetle Battles

This wraps the Taichi physics simulation in a Gymnax-style interface
so JAX-based RL algorithms can train beetle AI.
"""

import numpy as np
import jax.numpy as jnp
import math
from typing import Tuple, Dict

# Import existing game code
import beetle_physics
import simulation


class BeetleBattleEnv:
    """
    Gym-style environment for beetle combat.

    Observation: 19D vector (position, velocity, opponent info, etc.)
    Action: 3D continuous (thrust, turn, horn_tilt)
    Reward: Shaped reward for combat behavior
    """

    def __init__(self, opponent_policy=None):
        """
        Args:
            opponent_policy: Policy to use for opponent (default: random)
        """
        self.arena_radius = beetle_physics.ARENA_RADIUS
        self.max_episode_steps = 1800  # 30 seconds at 60 Hz
        self.step_count = 0
        self.opponent_policy = opponent_policy or RandomOpponent()

    def reset(self) -> np.ndarray:
        """Reset environment to initial state"""
        # Reset Taichi simulation
        beetle_physics.reset_match()
        self.step_count = 0

        # Extract initial observation
        obs = self._get_observation(beetle_physics.beetle_blue)
        return obs

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict]:
        """
        Execute one environment step (1/60 second)

        Args:
            action: [thrust, turn, horn_tilt] each in [-1, 1]

        Returns:
            observation: 19D state vector
            reward: Scalar reward
            done: True if episode ended
            info: Extra info dict
        """
        # Apply action to blue beetle (player)
        thrust, turn, horn_tilt = action
        self._apply_action(beetle_physics.beetle_blue, thrust, turn, horn_tilt)

        # Apply opponent action
        opp_obs = self._get_observation(beetle_physics.beetle_red)
        opp_action = self.opponent_policy.get_action(opp_obs)
        self._apply_action(beetle_physics.beetle_red, *opp_action)

        # Step Taichi physics (1/60 second)
        beetle_physics.update_physics_fixed_timestep(beetle_physics.PHYSICS_TIMESTEP)

        # Get new observation
        obs = self._get_observation(beetle_physics.beetle_blue)

        # Calculate reward
        reward = self._calculate_reward(
            beetle_physics.beetle_blue,
            beetle_physics.beetle_red
        )

        # Check termination
        self.step_count += 1
        done = (
            not beetle_physics.beetle_blue.active or
            not beetle_physics.beetle_red.active or
            self.step_count >= self.max_episode_steps
        )

        # Info
        info = {
            'winner': beetle_physics.match_winner if done else None,
            'episode_length': self.step_count,
            'blue_pos': (beetle_physics.beetle_blue.x, beetle_physics.beetle_blue.z),
            'red_pos': (beetle_physics.beetle_red.x, beetle_physics.beetle_red.z),
        }

        return obs, reward, done, info

    def _get_observation(self, beetle) -> np.ndarray:
        """
        Extract 19D observation from beetle state

        Returns:
            obs: [self_state(12), horn(1), opponent(4), arena(1), reserved(1)]
        """
        opponent = (beetle_physics.beetle_red if beetle.color == simulation.BEETLE_BLUE
                   else beetle_physics.beetle_blue)

        # Self state (12 values)
        self_state = np.array([
            beetle.x,
            beetle.z,
            beetle.y,
            beetle.vx,
            beetle.vz,
            beetle.vy,
            beetle.rotation,
            beetle.pitch,
            beetle.roll,
            beetle.angular_velocity,
            beetle.pitch_velocity,
            beetle.roll_velocity,
        ], dtype=np.float32)

        # Horn state (1 value)
        horn_state = np.array([beetle.horn_pitch], dtype=np.float32)

        # Opponent relative info (4 values)
        dx = opponent.x - beetle.x
        dz = opponent.z - beetle.z
        dist_to_opponent = math.sqrt(dx**2 + dz**2)
        angle_to_opponent = math.atan2(dz, dx) - beetle.rotation
        # Normalize angle to [-π, π]
        angle_to_opponent = math.atan2(math.sin(angle_to_opponent),
                                       math.cos(angle_to_opponent))

        opponent_state = np.array([
            dist_to_opponent,
            angle_to_opponent,
            opponent.vx - beetle.vx,  # Relative velocity
            opponent.vz - beetle.vz,
        ], dtype=np.float32)

        # Arena info (1 value)
        dist_to_edge = self.arena_radius - math.sqrt(beetle.x**2 + beetle.z**2)
        arena_state = np.array([dist_to_edge], dtype=np.float32)

        # Reserved (1 value) - for future use
        reserved = np.array([0.0], dtype=np.float32)

        # Concatenate (19 total)
        obs = np.concatenate([
            self_state,      # 12
            horn_state,      # 1
            opponent_state,  # 4
            arena_state,     # 1
            reserved         # 1
        ])

        return obs

    def _apply_action(self, beetle, thrust: float, turn: float, horn_tilt: float):
        """Apply RL action to beetle physics"""
        dt = beetle_physics.PHYSICS_TIMESTEP

        # Clip actions to valid range
        thrust = np.clip(thrust, -1.0, 1.0)
        turn = np.clip(turn, -1.0, 1.0)
        horn_tilt = np.clip(horn_tilt, -1.0, 1.0)

        # Apply thrust (forward/backward movement)
        forward_x = math.cos(beetle.rotation)
        forward_z = math.sin(beetle.rotation)
        beetle.apply_force(
            forward_x * thrust * beetle_physics.MOVE_FORCE,
            forward_z * thrust * beetle_physics.MOVE_FORCE,
            dt
        )

        # Apply turning
        beetle.angular_velocity += turn * beetle_physics.ROTATION_SPEED * dt

        # Apply horn control
        beetle.horn_pitch_velocity = horn_tilt * beetle_physics.HORN_TILT_SPEED

    def _calculate_reward(self, beetle, opponent) -> float:
        """
        Dense reward function for combat behavior

        Rewards:
        - Survival (+0.01 per frame)
        - Staying in center (+0.1)
        - Pushing opponent toward edge (+0.3)
        - Engaging opponent (+0.2)
        - Facing opponent (+0.15)
        - Winning (+10.0)
        - Losing (-10.0)
        """
        reward = 0.0

        # Survival reward (small constant)
        if beetle.active:
            reward += 0.01

        # Arena positioning (safety)
        dist_from_center = math.sqrt(beetle.x**2 + beetle.z**2)
        if dist_from_center < self.arena_radius * 0.5:
            reward += 0.1  # Safe zone
        elif dist_from_center > self.arena_radius * 0.8:
            reward -= 0.5  # Danger zone

        # Opponent positioning (offense)
        opp_dist_from_center = math.sqrt(opponent.x**2 + opponent.z**2)
        if opp_dist_from_center > self.arena_radius * 0.8:
            reward += 0.3  # Opponent in danger

        # Engagement (be close to opponent)
        dx = opponent.x - beetle.x
        dz = opponent.z - beetle.z
        dist_to_opp = math.sqrt(dx**2 + dz**2)
        if dist_to_opp < 20.0:
            reward += 0.2  # Close combat

        # Facing opponent (aim)
        angle_to_opp = math.atan2(dz, dx)
        angle_diff = abs(math.atan2(
            math.sin(angle_to_opp - beetle.rotation),
            math.cos(angle_to_opp - beetle.rotation)
        ))
        if angle_diff < math.pi / 4:  # Within 45° cone
            reward += 0.15  # Good aim

        # Terminal outcomes (large rewards)
        if not beetle.active:
            reward -= 10.0  # Lost
        if not opponent.active:
            reward += 10.0  # Won

        return reward


class RandomOpponent:
    """Simple random opponent for initial training"""

    def get_action(self, obs):
        """Random actions"""
        return np.random.uniform(-1.0, 1.0, size=3)


class ScriptedOpponent:
    """Hand-coded opponent for curriculum training"""

    def __init__(self, aggression=0.5):
        self.aggression = aggression  # 0=defensive, 1=aggressive

    def get_action(self, obs):
        """Simple scripted behavior"""
        # Extract key info from observation
        dist_to_opponent = obs[13]
        angle_to_opponent = obs[14]
        dist_to_edge = obs[17]

        # Decide behavior
        if dist_to_edge < 5.0:
            # Too close to edge - retreat
            thrust = -0.8  # Move back
            turn = 0.3 if angle_to_opponent > 0 else -0.3  # Turn away
            horn = -0.5  # Lower horn
        elif dist_to_opponent < 15.0:
            # Close to opponent - attack
            thrust = self.aggression  # Charge
            turn = 0.5 * angle_to_opponent  # Aim at opponent
            horn = 0.8  # Raise horn
        else:
            # Far from opponent - approach
            thrust = 0.6  # Move forward
            turn = 0.8 * angle_to_opponent  # Turn toward opponent
            horn = 0.0  # Neutral horn

        return np.array([thrust, turn, horn], dtype=np.float32)
```

---

### File: `rl/policy.py`

```python
"""
Neural Network Policy for Beetle Control

Uses Flax (JAX neural network library) to define an actor-critic
network for PPO training.
"""

import jax
import jax.numpy as jnp
import flax.linen as nn
from typing import Sequence


class BeetlePolicy(nn.Module):
    """
    Actor-Critic network for beetle control

    Architecture:
        Input (19) → Hidden (64) → Hidden (64) → Actor (3) + Critic (1)

    Actor outputs: Gaussian policy (mean + std for continuous actions)
    Critic outputs: State value estimate
    """
    hidden_dims: Sequence[int] = (64, 64)

    def setup(self):
        # Shared feature extractor
        self.dense1 = nn.Dense(self.hidden_dims[0])
        self.dense2 = nn.Dense(self.hidden_dims[1])

        # Actor head (policy)
        self.actor_mean = nn.Dense(3)  # [thrust, turn, horn]
        # Log std is a learnable parameter (not state-dependent)
        self.actor_logstd = self.param(
            'log_std',
            nn.initializers.zeros,
            (3,)
        )

        # Critic head (value function)
        self.critic = nn.Dense(1)

    @nn.compact
    def __call__(self, obs):
        """
        Forward pass

        Args:
            obs: [batch_size, 19] observation

        Returns:
            action_mean: [batch_size, 3] mean action
            action_logstd: [3] log std deviation (shared across batch)
            value: [batch_size, 1] state value estimate
        """
        # Normalize observations to [-1, 1] range
        obs_norm = obs / jnp.array([
            32, 32, 10,           # position bounds
            7, 7, 10,             # velocity bounds
            3.14, 0.785, 0.785,   # rotation bounds
            8, 8, 8,              # angular velocity bounds
            0.7,                  # horn angle bound
            64, 3.14, 14, 14,     # opponent info bounds
            32,                   # edge distance bound
            1.0                   # reserved
        ])

        # Shared features
        x = nn.relu(self.dense1(obs_norm))
        x = nn.relu(self.dense2(x))

        # Actor (policy)
        action_mean = nn.tanh(self.actor_mean(x))  # Bounded to [-1, 1]
        action_logstd = self.actor_logstd

        # Critic (value)
        value = self.critic(x)

        return action_mean, action_logstd, value

    def get_action(self, params, obs, rng):
        """
        Sample action from policy

        Args:
            params: Network parameters
            obs: Single observation [19]
            rng: JAX random key

        Returns:
            action: [3] sampled action
            log_prob: Log probability of action
            value: State value estimate
        """
        # Forward pass
        action_mean, action_logstd, value = self.apply(params, obs)

        # Sample from Gaussian
        action_std = jnp.exp(action_logstd)
        action = action_mean + action_std * jax.random.normal(
            rng, shape=action_mean.shape
        )

        # Clip to valid range
        action = jnp.clip(action, -1.0, 1.0)

        # Compute log probability
        log_prob = -0.5 * jnp.sum(
            ((action - action_mean) / action_std) ** 2 +
            2 * action_logstd +
            jnp.log(2 * jnp.pi)
        )

        return action, log_prob, value

    def get_value(self, params, obs):
        """Get state value estimate"""
        _, _, value = self.apply(params, obs)
        return value
```

---

### File: `scripts/train_beetle.py`

```python
"""
Simple training script for beetle AI

Usage:
    python scripts/train_beetle.py --episodes 10000 --output models/my_beetle.pkl
"""

import argparse
import time
import pickle
import numpy as np
import jax
import jax.numpy as jnp
import optax
from flax.training.train_state import TrainState

# Import RL components
import sys
sys.path.append('..')
from rl.environment import BeetleBattleEnv, RandomOpponent
from rl.policy import BeetlePolicy


def create_train_state(rng, learning_rate=3e-4):
    """Initialize training state"""
    policy = BeetlePolicy()
    dummy_obs = jnp.zeros(19)
    params = policy.init(rng, dummy_obs)

    tx = optax.chain(
        optax.clip_by_global_norm(0.5),
        optax.adam(learning_rate, eps=1e-5)
    )

    return TrainState.create(
        apply_fn=policy.apply,
        params=params,
        tx=tx
    )


def collect_episode(env, policy, params, rng):
    """Collect one episode of data"""
    obs_list = []
    action_list = []
    reward_list = []
    done_list = []
    value_list = []

    obs = env.reset()

    for step in range(env.max_episode_steps):
        # Convert to JAX
        obs_jax = jnp.array(obs)

        # Get action from policy
        rng, action_rng = jax.random.split(rng)
        action_mean, action_logstd, value = policy.apply(params, obs_jax)
        action_std = jnp.exp(action_logstd)
        action = action_mean + action_std * jax.random.normal(
            action_rng, shape=action_mean.shape
        )
        action = jnp.clip(action, -1.0, 1.0)

        # Convert back to NumPy
        action_np = np.array(action)

        # Step environment
        next_obs, reward, done, info = env.step(action_np)

        # Store
        obs_list.append(obs)
        action_list.append(action_np)
        reward_list.append(reward)
        done_list.append(done)
        value_list.append(float(value))

        obs = next_obs

        if done:
            break

    return {
        'obs': np.array(obs_list),
        'actions': np.array(action_list),
        'rewards': np.array(reward_list),
        'dones': np.array(done_list),
        'values': np.array(value_list),
        'info': info
    }


def compute_gae(rewards, values, dones, gamma=0.99, gae_lambda=0.95):
    """Compute Generalized Advantage Estimation"""
    advantages = np.zeros_like(rewards)
    gae = 0

    for t in reversed(range(len(rewards))):
        if t == len(rewards) - 1:
            next_value = 0
        else:
            next_value = values[t + 1]

        delta = rewards[t] + gamma * next_value * (1 - dones[t]) - values[t]
        gae = delta + gamma * gae_lambda * (1 - dones[t]) * gae
        advantages[t] = gae

    returns = advantages + values
    return advantages, returns


def train(num_episodes=10000, learning_rate=3e-4, output_path='trained_beetle.pkl'):
    """Main training loop"""
    print("Initializing training...")

    # Create environment and policy
    env = BeetleBattleEnv(opponent_policy=RandomOpponent())
    rng = jax.random.PRNGKey(0)
    rng, init_rng = jax.random.split(rng)
    train_state = create_train_state(init_rng, learning_rate)
    policy = BeetlePolicy()

    # Track statistics
    episode_rewards = []
    win_count = 0

    print(f"Training for {num_episodes} episodes...")
    start_time = time.time()

    for episode in range(num_episodes):
        # Collect episode
        rng, episode_rng = jax.random.split(rng)
        episode_data = collect_episode(env, policy, train_state.params, episode_rng)

        # Track stats
        total_reward = sum(episode_data['rewards'])
        episode_rewards.append(total_reward)
        if episode_data['info']['winner'] == 'BLUE':
            win_count += 1

        # Compute advantages
        advantages, returns = compute_gae(
            episode_data['rewards'],
            episode_data['values'],
            episode_data['dones']
        )

        # Normalize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        # TODO: Implement PPO update here
        # (For simplicity, this example just collects data)
        # In practice, you'd use a PPO implementation from PureJaxRL

        # Log progress
        if episode % 100 == 0:
            elapsed = time.time() - start_time
            avg_reward = np.mean(episode_rewards[-100:])
            win_rate = win_count / (episode + 1)
            print(f"Episode {episode}/{num_episodes} | "
                  f"Avg Reward: {avg_reward:.2f} | "
                  f"Win Rate: {win_rate:.2%} | "
                  f"Time: {elapsed:.1f}s")

        # Save checkpoint
        if episode % 1000 == 0 and episode > 0:
            checkpoint_path = output_path.replace('.pkl', f'_ep{episode}.pkl')
            with open(checkpoint_path, 'wb') as f:
                pickle.dump({
                    'params': train_state.params,
                    'episode': episode,
                    'win_rate': win_count / (episode + 1),
                }, f)
            print(f"Saved checkpoint: {checkpoint_path}")

    # Save final model
    with open(output_path, 'wb') as f:
        pickle.dump({
            'params': train_state.params,
            'episode': num_episodes,
            'win_rate': win_count / num_episodes,
        }, f)
    print(f"\nTraining complete! Saved to {output_path}")
    print(f"Final win rate: {win_count / num_episodes:.2%}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--episodes', type=int, default=10000)
    parser.add_argument('--lr', type=float, default=3e-4)
    parser.add_argument('--output', type=str, default='trained_beetle.pkl')
    args = parser.parse_args()

    train(args.episodes, args.lr, args.output)
```

---

## 9. Training Progression

### What to Expect at Different Stages

#### Episodes 0-100: Random Flailing

**Behavior:**
- Beetle moves randomly
- Often falls off edge immediately
- No coherent strategy
- Win rate: ~5% (pure luck)

**Observation metrics:**
- Average episode length: 200-400 steps (3-7 seconds)
- Average reward: -5 to +5 (mostly negative)

**What's happening:** Neural network has random weights, actions are essentially noise.

---

#### Episodes 100-500: Learning to Survive

**Behavior:**
- Beetle stays in arena more often
- Starts avoiding edges
- Still random fighting behavior
- Win rate: ~15%

**Observation metrics:**
- Average episode length: 800-1200 steps (13-20 seconds)
- Average reward: +2 to +8

**What's happening:** Beetle learns "don't fall off = good" from reward signal.

---

#### Episodes 500-2000: Basic Combat

**Behavior:**
- Beetle approaches opponent
- Occasionally lands successful hits
- Learns to face opponent
- Still poor timing
- Win rate: ~35%

**Observation metrics:**
- Average episode length: 1200-1600 steps (20-27 seconds)
- Average reward: +8 to +15

**What's happening:** Engagement and positioning rewards kick in, beetle learns combat basics.

---

#### Episodes 2000-5000: Tactical Emergence

**Behavior:**
- Beetle actively pushes opponent
- Uses horn strategically
- Circles opponent
- Controls center of arena
- Win rate: ~55%

**Observation metrics:**
- Average episode length: 1400-1800 steps (23-30 seconds, full matches)
- Average reward: +15 to +25

**What's happening:** Complex patterns emerge from reward combinations, beetle develops "style".

---

#### Episodes 5000-10000: Mastery

**Behavior:**
- Consistent winning tactics
- Adaptive to opponent strategies
- Feints and baits
- Edge control expertise
- Win rate: ~70% (vs random opponent)

**Observation metrics:**
- Average episode length: 1200-1500 (wins faster)
- Average reward: +20 to +35

**What's happening:** Policy converges to near-optimal for this opponent.

---

#### Episodes 10000+ (Self-Play): Diversity

**Behavior:**
- Multiple distinct strategies emerge
- Aggressive rushers
- Defensive turtles
- Circling flankers
- Counter-punchers
- Win rate: ~50% (self-play equilibrium)

**Observation metrics:**
- High variance in tactics
- More complex action distributions

**What's happening:** Arms race between training iterations creates diverse meta-game.

---

### Visual Training Progress

```
WIN RATE OVER TIME (vs random opponent)

100% ┤                                        ╭─────────
 90% ┤                                   ╭────╯
 80% ┤                              ╭────╯
 70% ┤                         ╭────╯
 60% ┤                    ╭────╯
 50% ┤               ╭────╯
 40% ┤          ╭────╯
 30% ┤     ╭────╯
 20% ┤ ╭───╯
 10% ┤─╯
  0% ┼─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────
     0    1k    2k    3k    4k    5k    6k    7k    8k    9k   10k
                          EPISODES


AVERAGE REWARD OVER TIME

 40 ┤                                              ╭──────
 35 ┤                                         ╭────╯
 30 ┤                                    ╭────╯
 25 ┤                               ╭────╯
 20 ┤                          ╭────╯
 15 ┤                     ╭────╯
 10 ┤                ╭────╯
  5 ┤           ╭────╯
  0 ┤      ╭────╯
 -5 ┤──────╯
-10 ┼─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────
     0    1k    2k    3k    4k    5k    6k    7k    8k    9k   10k
                          EPISODES
```

---

## 10. Player Experience Design

### Training UI Mockup

```
╔════════════════════════════════════════════════════════════╗
║              BEETLE TRAINING CENTER                        ║
╠════════════════════════════════════════════════════════════╣
║                                                            ║
║  Beetle Name: [My Super Beetle_____________]              ║
║                                                            ║
║  Training Duration:                                        ║
║    ◯ Quick (5 min)    ● Medium (15 min)    ◯ Long (30 min)║
║                                                            ║
║  Opponent Difficulty:                                      ║
║    ◯ Easy    ● Medium    ◯ Hard    ◯ Self-Play           ║
║                                                            ║
║  Fighting Style:                                           ║
║    ◯ Aggressive (charge, high risk)                       ║
║    ● Balanced (adaptive)                                   ║
║    ◯ Defensive (patient, counter-attack)                  ║
║                                                            ║
║  ┌──────────────────────────────────────────────────────┐ ║
║  │           [  START TRAINING  ]                       │ ║
║  └──────────────────────────────────────────────────────┘ ║
║                                                            ║
╠════════════════════════════════════════════════════════════╣
║  Previous Training Sessions:                               ║
║    • Beetle_V1 (15 min, 68% win rate) [Load] [Delete]     ║
║    • Beetle_V2 (30 min, 74% win rate) [Load] [Delete]     ║
╚════════════════════════════════════════════════════════════╝


DURING TRAINING:
╔════════════════════════════════════════════════════════════╗
║              TRAINING IN PROGRESS...                       ║
╠════════════════════════════════════════════════════════════╣
║                                                            ║
║  Episode: 3,247 / 10,000                                   ║
║  Progress: [████████░░░░░░░░] 32%                         ║
║                                                            ║
║  Current Performance:                                      ║
║    Win Rate: 54.2%  ↑ (+2.1% from last 100)               ║
║    Avg Fight Duration: 18.3 seconds                        ║
║    Wins: 1,761   Losses: 1,486                            ║
║                                                            ║
║  Estimated Time Remaining: 9 minutes                       ║
║                                                            ║
║  ┌────────────────────────────────────────────────────┐   ║
║  │  Win Rate Over Time:                               │   ║
║  │  100% ┤                    ╭───                     │   ║
║  │   75% ┤               ╭────╯                        │   ║
║  │   50% ┤          ╭────╯                             │   ║
║  │   25% ┤     ╭────╯                                  │   ║
║  │    0% ┼─────╯                                       │   ║
║  │       └─────┬─────┬─────┬─────┬─────┬─────         │   ║
║  │            0    1k   2k   3k   4k   5k              │   ║
║  └────────────────────────────────────────────────────┘   ║
║                                                            ║
║  [  PAUSE  ]  [  STOP TRAINING  ]                         ║
║                                                            ║
╚════════════════════════════════════════════════════════════╝


AFTER TRAINING:
╔════════════════════════════════════════════════════════════╗
║              TRAINING COMPLETE!                            ║
╠════════════════════════════════════════════════════════════╣
║                                                            ║
║  Beetle: My Super Beetle                                   ║
║  Total Episodes: 10,000                                    ║
║  Training Time: 14 minutes 32 seconds                      ║
║                                                            ║
║  ╭────────────────────────────────────────────────────╮   ║
║  │  FINAL STATS:                                      │   ║
║  │                                                     │   ║
║  │  Win Rate:  72.4%  ⭐⭐⭐⭐                         │   ║
║  │  Rating:    1547 ELO (Gold Tier)                   │   ║
║  │                                                     │   ║
║  │  Strengths:                                        │   ║
║  │    • Excellent edge control                        │   ║
║  │    • Aggressive horn usage                         │   ║
║  │    • Quick reactions                               │   ║
║  │                                                     │   ║
║  │  Weaknesses:                                       │   ║
║  │    • Vulnerable when rushed early                  │   ║
║  │    • Sometimes overcommits                         │   ║
║  │                                                     │   ║
║  │  Fighting Style: Aggressive Rusher                 │   ║
║  ╰────────────────────────────────────────────────────╯   ║
║                                                            ║
║  [  SAVE BEETLE  ]  [  TEST IN ARENA  ]  [  TRAIN MORE  ] ║
║                                                            ║
╚════════════════════════════════════════════════════════════╝
```

---

### Competition UI Mockup

```
╔════════════════════════════════════════════════════════════╗
║              BEETLE BATTLE ARENA                           ║
╠════════════════════════════════════════════════════════════╣
║                                                            ║
║  YOUR BEETLE:                   OPPONENT:                  ║
║  ┌────────────────────────┐    ┌────────────────────────┐ ║
║  │ My Super Beetle        │    │ OpponentBeetle_2000    │ ║
║  │ Rating: 1547           │ VS │ Rating: 1523           │ ║
║  │ Style: Aggressive      │    │ Style: Defensive       │ ║
║  │ Record: 72W - 28L      │    │ Record: 65W - 35L      │ ║
║  └────────────────────────┘    └────────────────────────┘ ║
║                                                            ║
║  Match Type:                                               ║
║    ● Quick Match (find random opponent)                    ║
║    ○ Challenge Friend (enter code)                         ║
║    ○ Ranked Match (ELO ladder)                             ║
║    ○ Tournament (bracket system)                           ║
║                                                            ║
║  [  FIND OPPONENT  ]    [  LOAD BEETLE FILE  ]            ║
║                                                            ║
╠════════════════════════════════════════════════════════════╣
║  LEADERBOARD (Top 10):                                     ║
║   #1  UltraBeetle_X        2145 ELO   ⭐⭐⭐⭐⭐          ║
║   #2  DestroyerBot         2098 ELO   ⭐⭐⭐⭐⭐          ║
║   #3  BeetleKing           2034 ELO   ⭐⭐⭐⭐            ║
║   ...                                                      ║
║   #47 My Super Beetle      1547 ELO   ⭐⭐⭐              ║
║   ...                                                      ║
╚════════════════════════════════════════════════════════════╝
```

---

### Simple Player Interface (Recommended)

Players shouldn't need to know RL/ML. Hide complexity:

**What players see:**
- Training time slider (5 min → 30 min)
- Opponent difficulty (Easy/Medium/Hard)
- Style preference (Aggressive/Balanced/Defensive)
- Progress bar + win rate graph
- Final stats (strengths/weaknesses auto-detected)

**What's hidden:**
- Learning rates
- Network architecture
- PPO hyperparameters
- Advantage estimation
- Gradient clipping

**Example implementation:**

```python
class PlayerTrainingInterface:
    """Simple interface for non-technical players"""

    STYLE_REWARDS = {
        'aggressive': {
            'engagement_weight': 0.4,  # High reward for being close
            'safety_weight': 0.05,     # Low penalty for danger
            'damage_weight': 0.5,      # High reward for pushing opponent
        },
        'defensive': {
            'engagement_weight': 0.1,  # Low reward for being close
            'safety_weight': 0.3,      # High penalty for danger
            'damage_weight': 0.3,      # Moderate reward for damage
        },
        'balanced': {
            'engagement_weight': 0.2,
            'safety_weight': 0.1,
            'damage_weight': 0.3,
        },
    }

    def train_beetle(self, name: str, duration_minutes: int,
                     difficulty: str, style: str):
        """
        Train a beetle with player-friendly parameters

        Args:
            name: Beetle name
            duration_minutes: 5, 15, or 30
            difficulty: 'easy', 'medium', 'hard'
            style: 'aggressive', 'defensive', 'balanced'
        """
        # Convert duration to episodes
        episodes = duration_minutes * 40  # ~40 episodes/minute

        # Create opponent based on difficulty
        if difficulty == 'easy':
            opponent = RandomOpponent()
        elif difficulty == 'medium':
            opponent = ScriptedOpponent(aggression=0.5)
        else:  # hard
            opponent = ScriptedOpponent(aggression=0.8)

        # Modify reward function based on style
        reward_weights = self.STYLE_REWARDS[style]

        # Run training (with all ML complexity hidden)
        print(f"Training {name} for {duration_minutes} minutes...")
        trained_model = self._run_training(
            episodes=episodes,
            opponent=opponent,
            reward_weights=reward_weights
        )

        # Detect fighting style automatically
        detected_style = self._analyze_policy(trained_model)

        # Save with metadata
        self._save_beetle(trained_model, {
            'name': name,
            'style': detected_style,
            'win_rate': trained_model.final_win_rate,
        })

    def _analyze_policy(self, model):
        """Auto-detect fighting style from learned behavior"""
        # Run test episodes and measure:
        # - Average distance to opponent (engagement level)
        # - Average distance to edge (risk-taking)
        # - Action variance (aggression)
        # Classify into style categories
        pass
```

---

## 11. Next Steps

### Week 1: Foundation
1. Install JAX dependencies: `pip install jax flax optax gymnax`
2. Create `rl/environment.py` with `BeetleBattleEnv` class
3. Test: Can you reset and step the environment?
4. Verify observations are correct (print and inspect)

### Week 2: Basic Training
1. Create `rl/policy.py` with `BeetlePolicy` network
2. Create simple training loop (collect episodes, no PPO yet)
3. Test: Can policy output actions and step env?
4. Add reward function and verify it's giving reasonable values

### Week 3: PPO Implementation
1. Integrate PureJaxRL or implement simple PPO
2. Train for 1000 episodes against random opponent
3. Verify: Is win rate improving?
4. Add checkpoint saving/loading

### Week 4: Opponents & Tuning
1. Implement scripted opponents (aggressive, defensive)
2. Train for 5000+ episodes
3. Tune reward function if needed
4. Verify: Does beetle show combat behaviors?

### Week 5: Player Interface
1. Create simple training UI
2. Add progress visualization
3. Implement style preferences
4. Test with non-technical person

### Week 6: Competition Mode
1. Implement model loading for vs mode
2. Create ELO rating system
3. Build matchmaking
4. Host first tournament!

---

## 12. FAQ

**Q: Do I need to understand all the math?**
A: No! Use existing libraries (PureJaxRL, Flax) that handle PPO. You mainly need to understand observation/action/reward design.

**Q: How long does training take?**
A: Sequential: 1-2 hours for 10K episodes. Vectorized: 10-15 minutes.

**Q: Can I train on CPU?**
A: Yes, but slower (3-5x). GPU strongly recommended for faster iteration.

**Q: What if training doesn't work?**
A: Common issues:
- Reward function too sparse → Add dense rewards
- Observation not normalized → Scale to [-1, 1]
- Learning rate too high → Lower to 1e-4
- Network too big → Try smaller (32, 32)

**Q: How do I make beetles fight each other?**
A: Save trained policy as `.pkl` file, load both policies, run them in same environment.

**Q: Can beetles train against themselves?**
A: Yes! Self-play: Save checkpoints, load random past versions as opponents.

**Q: Do I need to implement PPO from scratch?**
A: No! Use PureJaxRL or CleanRL's PPO implementation. Just adapt the environment wrapper.

**Q: What makes one beetle better than another?**
A: Training time, opponent diversity, reward function quality, and emergent strategies from self-play.

---

## 13. Resources

### Recommended Reading
- **PPO Paper:** [Proximal Policy Optimization Algorithms](https://arxiv.org/abs/1707.06347)
- **PureJaxRL:** [GitHub](https://github.com/luchris429/purejaxrl)
- **OpenAI Spinning Up:** [RL Introduction](https://spinningup.openai.com/en/latest/)

### Code Examples
- **PureJaxRL PPO:** Clean JAX implementation
- **CleanRL PPO:** PyTorch version with great docs
- **Gymnax:** JAX environment examples

### Similar Projects
- **Neural MMO:** Multi-agent RL combat
- **OpenAI Five (Dota):** Team-based combat RL
- **AlphaStar (StarCraft):** RTS game RL

---

**End of Guide**

This document should give you everything needed to understand and implement RL training for beetle battles. Start simple (Option A), iterate, and have fun!
