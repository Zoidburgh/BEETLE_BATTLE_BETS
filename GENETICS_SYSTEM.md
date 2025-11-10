# Beetle Genetics System

## Core Genetics Model

**Diploid System**: Each beetle has 2 alleles (copies) per gene - one from each parent.

**Phenotype Expression**: The expressed trait value is the **average** of the two alleles.

**Example:**
- Parent A has horn_length alleles [12, 14] → Phenotype: 13 voxels
- Parent B has horn_length alleles [8, 10] → Phenotype: 9 voxels
- Offspring gets one random allele from each parent:
  - Possible outcomes: [12,8]=10, [12,10]=11, [14,8]=11, [14,10]=12
  - Average offspring: ~10-11 voxels (between parents)

---

## 16 Heritable Traits

### VOXEL TRAITS (Physical Geometry)

#### 1. horn_length
- **What it changes:** Number of voxels in horn length
- **Allele type:** Integer (discrete voxels)
- **Current default:** 9 voxels
- **Proposed range:** 6 - 15 voxels
- **Affects:** Reach, leverage point height, contact range

#### 2. horn_thickness
- **What it changes:** Number of voxels in horn width
- **Allele type:** Integer (discrete voxels)
- **Current default:** 2 voxels wide
- **Proposed range:** 1 - 4 voxels
- **Affects:** Contact area, visual bulk, collision detection

#### 3. body_width
- **What it changes:** Width of beetle body in voxels
- **Allele type:** Integer (discrete voxels)
- **Current default:** ~14 voxels
- **Proposed range:** 10 - 18 voxels
- **Affects:** Collision hitbox width, stability, mass

#### 4. body_length
- **What it changes:** Length of beetle body in voxels
- **Allele type:** Integer (discrete voxels)
- **Current default:** ~14 voxels
- **Proposed range:** 10 - 18 voxels
- **Affects:** Collision hitbox length, mass, center of gravity

#### 5. leg_length
- **What it changes:** Number of voxel segments per leg
- **Allele type:** Integer (discrete segments)
- **Current default:** 3 segments
- **Proposed range:** 2 - 5 segments
- **Affects:** Center of gravity height, tip-over threshold, visual appearance

#### 6. leg_thickness
- **What it changes:** Voxel width of each leg segment
- **Allele type:** Integer (discrete voxels)
- **Current default:** 1 voxel thick
- **Proposed range:** 1 - 3 voxels
- **Affects:** Visual bulk, slight mass increase, collision (minor)

#### 7. shell_layers
- **What it changes:** Number of voxel layers forming shell/armor
- **Allele type:** Integer (discrete layers)
- **Current default:** 1 layer
- **Proposed range:** 1 - 3 layers
- **Affects:** Mass significantly, collision volume, visual thickness

---

### INVISIBLE STAT TRAITS (Physics Parameters)

#### 8. mass
- **What it changes:** Mass multiplier for physics
- **Allele type:** Float (continuous)
- **Current default:** 1.0
- **Proposed range:** 0.5 - 2.5
- **Affects:** Momentum, push force, lifting resistance, gravity pull

#### 9. forward_speed
- **What it changes:** MAX_SPEED for forward movement
- **Allele type:** Float (continuous)
- **Current default:** 7.0
- **Proposed range:** 5.0 - 10.0
- **Affects:** Top speed when moving forward

#### 10. backward_speed
- **What it changes:** BACKWARD_MAX_SPEED for backward movement
- **Allele type:** Float (continuous)
- **Current default:** 5.0
- **Proposed range:** 3.0 - 7.0
- **Affects:** Top speed when moving backward (retreat)

#### 11. max_speed (DEPRECATED - merged into forward_speed)
*Note: We'll use forward_speed and backward_speed instead of a single max_speed*

#### 12. rotation_speed
- **What it changes:** ROTATION_SPEED (angular acceleration)
- **Allele type:** Float (continuous)
- **Current default:** 3.0 rad/s
- **Proposed range:** 1.5 - 5.5 rad/s
- **Affects:** How quickly beetle starts turning

#### 13. max_angular_speed
- **What it changes:** MAX_ANGULAR_SPEED (top turning speed)
- **Allele type:** Float (continuous)
- **Current default:** 8.0 rad/s
- **Proposed range:** 5.0 - 12.0 rad/s
- **Affects:** Maximum rotation velocity

#### 14. horn_tilt_speed
- **What it changes:** HORN_TILT_SPEED (horn movement rate)
- **Allele type:** Float (continuous)
- **Current default:** 2.0 rad/s
- **Proposed range:** 0.8 - 3.5 rad/s
- **Affects:** How fast horn can tilt up/down

#### 15. horn_tilt_range
- **What it changes:** Total angular range horn can move
- **Allele type:** Float (continuous, in radians)
- **Current default:** 40° total (HORN_MIN_PITCH to HORN_MAX_PITCH)
- **Proposed range:** 30° - 60° total range
- **Affects:** Horn flexibility, attack angles available

#### 16. restoring_strength
- **What it changes:** RESTORING_STRENGTH (uprighting torque)
- **Allele type:** Float (continuous)
- **Current default:** 15.0
- **Proposed range:** 10.0 - 25.0
- **Affects:** How quickly beetle recovers from being tipped

---

## Revised Trait List (15 Total)

After removing max_speed duplicate:

### VOXEL TRAITS (7):
1. horn_length
2. horn_thickness
3. body_width
4. body_length
5. leg_length
6. leg_thickness
7. shell_layers

### STAT TRAITS (8):
8. mass
9. forward_speed
10. backward_speed
11. rotation_speed
12. max_angular_speed
13. horn_tilt_speed
14. horn_tilt_range
15. restoring_strength

---

## Breeding Mechanics

### Basic Inheritance

```python
def breed(parent_a, parent_b):
    """Sexual reproduction - one random allele from each parent"""
    child = BeetleGenome()

    for trait in all_traits:
        # Get alleles from parents
        allele_from_a = random.choice(parent_a.get_alleles(trait))
        allele_from_b = random.choice(parent_b.get_alleles(trait))

        # Child gets both
        child.set_alleles(trait, [allele_from_a, allele_from_b])

    return child
```

### Phenotype Calculation

```python
def get_phenotype(trait):
    """Express the trait - average of two alleles"""
    alleles = self.get_alleles(trait)
    return sum(alleles) / 2.0
```

For discrete traits (voxels), round to nearest integer:
```python
horn_length_phenotype = round((allele1 + allele2) / 2.0)
```

---

## Mutation System Options

### Option 1: Point Mutations (Simple)

**When:** During breeding, each allele has small mutation chance

**Probability:** 5% per allele per trait

**Effect:** Add random offset to allele value

```python
def apply_point_mutation(allele, trait_type):
    if random.random() < 0.05:  # 5% chance
        if trait_type == 'voxel':
            # Discrete: ±1 voxel
            allele += random.choice([-1, 0, 0, 1])
        else:
            # Continuous: ±10% of current value
            allele *= random.uniform(0.9, 1.1)

    return clamp_to_valid_range(allele)
```

**Example:**
- Parent has horn_length allele = 12
- Mutation: 12 → 13 (grew by 1 voxel)
- Offspring gets this mutated allele

---

### Option 2: Gene Duplication (Rare Jumps)

**When:** Breeding, very rare

**Probability:** 1% per trait

**Effect:** Allele value jumps significantly (breakthrough genetics)

```python
def apply_gene_duplication(allele, trait_type):
    if random.random() < 0.01:  # 1% chance
        if trait_type == 'voxel':
            # Jump by ±2-3 voxels
            allele += random.choice([-3, -2, 2, 3])
        else:
            # Jump by ±20-30%
            allele *= random.uniform(0.7, 1.3)

    return clamp_to_valid_range(allele)
```

**Example:**
- Parent has mass allele = 1.0
- Rare mutation: 1.0 → 1.3 (heavyweight mutation)
- Creates "titan" bloodline suddenly

---

### Option 3: Beneficial Mutations (Random Stat Boost)

**When:** Breeding, very rare

**Probability:** 0.5% per entire genome

**Effect:** Random trait gets permanent +5-10% boost

```python
def apply_beneficial_mutation(genome):
    if random.random() < 0.005:  # 0.5% chance
        # Pick random trait
        trait = random.choice(all_traits)
        allele_idx = random.randint(0, 1)

        # Boost by 5-10%
        genome.alleles[trait][allele_idx] *= random.uniform(1.05, 1.10)

        # Mark as "gifted" in lineage
        genome.mutations.append(f"Beneficial: +{trait}")
```

**Example:**
- Offspring randomly gets +8% horn_tilt_speed
- This becomes inheritable - can pass to future generations

---

### Option 4: Detrimental Mutations (Balancing)

**When:** Breeding

**Probability:** 0.5% per entire genome

**Effect:** Random trait gets -5-10% penalty

```python
def apply_detrimental_mutation(genome):
    if random.random() < 0.005:  # 0.5% chance
        trait = random.choice(all_traits)
        allele_idx = random.randint(0, 1)

        # Penalty by 5-10%
        genome.alleles[trait][allele_idx] *= random.uniform(0.90, 0.95)

        genome.mutations.append(f"Detrimental: -{trait}")
```

**Why include this?** Prevents runaway optimization, adds realism, creates breeding challenge

---

### Option 5: Linked Traits (Advanced)

**Concept:** Some traits are genetically linked (inherit together)

**Example:** `body_width` and `body_length` on same chromosome

```python
# If parent passes body_width allele [0], must also pass body_length allele [0]
# Creates correlation between traits
```

**Effect:**
- Wide beetles tend to also be long (bulk builds)
- Narrow beetles tend to be short (compact builds)
- Can still get mismatches through crossover events (rare)

---

### Option 6: Crossover Events (Chromosomal)

**Concept:** During reproduction, chromosome segments swap

**Probability:** 10% per trait pair

**Effect:** Breaks linked traits, creates new combinations

```python
def chromosomal_crossover(parent_a, parent_b):
    """Swap segments of genetic material"""
    if random.random() < 0.10:
        # Swap alleles for linked traits
        parent_a.body_width, parent_b.body_width = parent_b.body_width, parent_a.body_width
```

**Example:**
- Parent A: Wide body [18] + Short length [10] (unusual mix)
- Parent B: Narrow body [10] + Long length [18] (unusual mix)
- Crossover: Creates normal combos [18,18] or [10,10]

---

### Option 7: Hybrid Vigor / Inbreeding Depression

**Concept:** Genetic diversity affects fitness

**Hybrid Vigor (Outbreeding):**
```python
def calculate_hybrid_vigor(parent_a, parent_b):
    """Bonus if parents are genetically diverse"""
    diversity = calculate_genetic_distance(parent_a, parent_b)

    if diversity > 0.5:  # Very different genetics
        # All stats +3%
        for trait in child.alleles:
            child.alleles[trait] *= 1.03
```

**Inbreeding Depression:**
```python
def calculate_inbreeding_penalty(parent_a, parent_b):
    """Penalty if parents are too similar"""
    diversity = calculate_genetic_distance(parent_a, parent_b)

    if diversity < 0.2:  # Very similar genetics
        # All stats -5%
        for trait in child.alleles:
            child.alleles[trait] *= 0.95
```

**Why?** Encourages diverse breeding, punishes same-lineage breeding

---

### Option 8: Sex-Linked Traits (Optional)

**Concept:** Some traits only pass from mother or father

**Example:** Color pattern only from mother

```python
# Mother always passes color alleles
# Father never passes color alleles
child.color = parent_mother.color
```

**Why?** Adds breeding complexity, makes certain lineages valuable

---

### Option 9: Recessive "Carrier" Traits

**Concept:** Some alleles don't express unless homozygous (both copies same)

**Example:** Gold color is recessive

```python
# Regular allele: Normal color (N)
# Recessive allele: Gold color (g)

# NN = Normal (no gold)
# Ng = Normal (carries gold gene)
# gg = Gold (expresses!)

def get_color_phenotype(alleles):
    if alleles == ['g', 'g']:
        return 'GOLD'
    else:
        return 'NORMAL'
```

**Strategy:** Breed two carriers (Ng × Ng) → 25% chance of gold offspring (gg)

---

## Recommended Mutation Setup (For MVP)

**Start Simple:**
1. **Point Mutations** - 5% per allele (small changes)
2. **Beneficial Mutations** - 0.5% per genome (rare stat boosts)

**Add Later:**
3. **Gene Duplication** - 1% per trait (rare jumps)
4. **Hybrid Vigor** - Bonus for diverse parents

**Optional (Phase 2):**
5. Linked traits
6. Crossover events
7. Inbreeding depression

---

## Breeding Strategy Examples

### Goal: Breed a Fast Attacker

**Target traits:**
- High forward_speed (9.0+)
- High horn_tilt_speed (3.0+)
- Low mass (0.7 or less)

**Strategy:**
1. Find beetles with fast alleles hidden in genome
2. Breed fastest × fastest
3. Hope for lucky combination [9.5, 9.2] → 9.35 phenotype
4. Repeat for 3-4 generations
5. Get pure-bred fast line [10.0, 10.0] → 10.0 (max speed)

---

### Goal: Breed a Tank

**Target traits:**
- High mass (2.0+)
- High shell_layers (3)
- Short leg_length (2 segments)
- High restoring_strength (22+)

**Strategy:**
1. Breed heavy × heavy
2. Breed short-legged × short-legged
3. Cross the two lines
4. Select offspring with best combo
5. Inbreed to lock in traits

---

### Goal: Discover Rare Mutation

**Target:** Gold color beetle with 15 voxel horn

**Strategy:**
1. Breed 100s of beetles (volume increases mutation chances)
2. When mutation appears, immediately breed that beetle
3. Try to get both mutated alleles in one beetle [15,15]
4. Create pure-bred mutant line

---

## Implementation Notes

### Data Structure

```python
class BeetleGenome:
    def __init__(self):
        # Each trait has 2 alleles (diploid)
        self.alleles = {
            'horn_length': [9, 9],
            'horn_thickness': [2, 2],
            'body_width': [14, 14],
            'body_length': [14, 14],
            'leg_length': [3, 3],
            'leg_thickness': [1, 1],
            'shell_layers': [1, 1],
            'mass': [1.0, 1.0],
            'forward_speed': [7.0, 7.0],
            'backward_speed': [5.0, 5.0],
            'rotation_speed': [3.0, 3.0],
            'max_angular_speed': [8.0, 8.0],
            'horn_tilt_speed': [2.0, 2.0],
            'horn_tilt_range': [0.7, 0.7],  # radians (~40°)
            'restoring_strength': [15.0, 15.0],
        }

        # Track mutations for lineage display
        self.mutations = []

    def get_phenotype(self, trait):
        """Calculate expressed value (average of alleles)"""
        avg = sum(self.alleles[trait]) / 2.0

        # Round discrete traits
        if trait in ['horn_length', 'horn_thickness', 'body_width',
                     'body_length', 'leg_length', 'leg_thickness', 'shell_layers']:
            return round(avg)

        return avg
```

---

## Random Beetle Generation (Starting Population)

To create initial genetic diversity:

```python
def create_random_beetle():
    """Generate beetle with randomized starting genes"""
    genome = BeetleGenome()

    # Randomize each allele within starting range (±20% of default)
    genome.alleles['horn_length'] = [
        random.randint(7, 11),
        random.randint(7, 11)
    ]

    genome.alleles['mass'] = [
        random.uniform(0.8, 1.2),
        random.uniform(0.8, 1.2)
    ]

    # ... etc for all 15 traits

    return genome
```

**Starting population:** 10-20 random beetles ensures genetic diversity for players to work with.

---

## Visualization for Players

**Genome View:**
```
Beetle: "Tank Tom"
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
VOXEL TRAITS:
  Horn Length:    [12][10] → 11 voxels
  Body Width:     [16][18] → 17 voxels (bulky)
  Leg Length:     [2][3]   → 2.5 → 2 segments (short)

STAT TRAITS:
  Mass:           [1.8][2.0] → 1.9x (heavy!)
  Forward Speed:  [6.5][7.0] → 6.75
  Restoring:      [18][20]   → 19.0 (fast recovery)

MUTATIONS:
  ✨ Beneficial: +mass (generation 3)

LINEAGE:
  Mother: "Big Bertha" (mass 1.8)
  Father: "Fortress" (mass 2.0)
```

**Hidden Alleles:**
Players can see both alleles to make informed breeding decisions!

---

## Future Expansions

**Color genetics:**
- Add RGB color traits with codominance
- Red [255,0,0] × Blue [0,0,255] → Purple [128,0,128]

**Pattern genetics:**
- Stripe pattern (recessive)
- Spot pattern (dominant)
- Combine for complex appearances

**Ability genes:**
- Rare mutations unlock special abilities
- Must be homozygous to express (both alleles same)
- Example: [leap, leap] → Gains jump ability

---

This system creates **deep strategic breeding** while remaining mathematically simple (just averaging alleles).
