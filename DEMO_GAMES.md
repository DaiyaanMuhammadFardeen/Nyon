# Nyon Demo Games

This document provides a comprehensive technical reference for all four demo games built on the Nyon ECS engine. Each demo extends `Nyon::ECSApplication`, runs at **1280×720**, uses the ECS component pattern, and iterates `contactManifolds` manually for collision detection logic.

---

## Table of Contents

1. [Demo 1: Simple Physics Demo](#demo-1-simple-physics-demo)
2. [Demo 2: Breakout Demo](#demo-2-breakout-demo)
3. [Demo 3: Flappy Bird Demo](#demo-3-flappy-bird-demo)
4. [Demo 4: Tower Stack Demo](#demo-4-tower-stack-demo)
5. [Cross-Demo Features Table](#cross-demo-features-table)
6. [Physics World Contact Manifolds API](#physics-world-contact-manifolds-api)

---

## Demo 1: Simple Physics Demo

A sandbox for testing core physics engine features: gravity, collision detection, restitution, friction, sloped surfaces, and dynamic spawning.

- **Directory**: `game/simple-physics-demo/`
- **Files**:
  - `include/SimplePhysicsDemo.h`
  - `src/SimplePhysicsDemo.cpp`
  - `src/main.cpp`
  - `CMakeLists.txt`

### Window & Gravity

| Property | Value |
|---|---|
| Window title | `"Nyon – Simple Physics Demo"` |
| Resolution | 1280×720 |
| Gravity | `{0, -980}` px/s² (Y-up convention) |

### Entities

#### World (`PhysicsWorldComponent`)

```cpp
world.gravity              = { 0.0f, -980.0f };
world.timeStep             = 1.0f / 60.0f;
world.velocityIterations   = 8;
world.positionIterations   = 3;
world.linearSlop           = 0.05f;
world.maxLinearCorrection  = 2.0f;
world.enableSleep          = true;
world.enableWarmStarting   = true;
world.enableContinuous     = false;
```

> **Note**: The pipeline currently reads `m_Config.linearSlop` (value `0.005`) instead of the world component field. The world component is set correctly (`0.05`), but the pipeline bug must be fixed for this to take effect.

#### Platform (static)

- **Shape**: Polygon 800×50 (centered on origin)
- **Position**: `{640, 100}`
- **Friction**: `0.6`
- **Restitution**: `0.8`
- **Color**: `{0.35, 0.35, 0.35}` (grey)
- **Render origin**: `{400, 25}` (half-extents, matching physics centroid)

#### Sloped Platform Left (static)

- **Shape**: Polygon 500×50
- **Position**: `{150, 80}`
- **Rotation**: `-0.436332` rad (−25°)
- **Friction**: `0.5`
- **Restitution**: `0.8`
- **Color**: `{0.4, 0.4, 0.8}` (blueish)

#### Sloped Platform Right (static)

- **Shape**: Polygon 500×50
- **Position**: `{1130, 80}`
- **Rotation**: `0.436332` rad (+25°)
- **Friction**: `0.7`
- **Restitution**: `0.8`
- **Color**: `{0.8, 0.4, 0.4}` (redish)

#### Player (dynamic)

| Property | Value |
|---|---|
| Shape | Circle, radius = 25 |
| Position | `{640, 360}` |
| Mass | `2.0` |
| Inertia | `collider.CalculateInertiaPerUnitMass() * mass` |
| Density | `0.08` |
| Friction | `0.4` |
| Restitution | `0.8` |
| Angular damping | `0.1` |
| Linear drag | `0.01` |
| Lock rotation | `true` |
| Allow sleep | `false` |
| Color | `{1.0, 0.5, 0.0}` (orange) |
| Render shape | `Circle` |

#### Auto-Spawned Circles

- **Interval**: Random `0.5`–`1.5` seconds
- **Spawn position**: Random X within `[100, width-100]`, Y at 80% of screen height
- **Radius**: Random `10`–`20` pixels
- **Mass**: `1.0`
- **Color**: Random RGB per circle
- **Lock rotation**: `true`
- **Restitution**: `0.8`

#### Click-Spawned Circles

- Triggered by left mouse button at mouse position (Y flipped for Y-up convention)
- **Radius**: Random `10`–`20` pixels (uses `CreateSpawnedCircle`)
- **Mass**: `2.0`
- **Restitution**: `0.8`
- **Color**: Random RGB

### Player Controls

| Key | Action | Mechanics |
|---|---|---|
| `A` / `D` | Move horizontally | `body.velocity.x = horizontal * PLAYER_MOVE_SPEED` |
| `W` | Jump (when grounded) | `body.velocity.y += PLAYER_JUMP_FORCE / body.mass` |
| No input | Horizontal damping | `body.velocity.x *= 0.9f` |

- **`PLAYER_MOVE_SPEED`**: `300.0` px/s
- **`PLAYER_JUMP_FORCE`**: `900.0` (impulse applied as velocity change)
- **Grounded detection**: Iterates `contactManifolds`, checks if player is involved, computes contact normal direction, and validates `dot(contactNormal, {0,1}) > 0.7` with at least one contact point having `separation < 1.0`

### Cleanup

Entities with Y position below `DESPAWN_Y_THRESHOLD` = `-100.0` are destroyed and removed from the tracking list.

---

## Demo 2: Breakout Demo

A classic Breakout game with zero-gravity physics, perfect bounce, and BFS-generated brick formations.

- **Directory**: `game/breakout-demo/`
- **Files**:
  - `include/BreakoutDemo.h`
  - `src/BreakoutDemo.cpp`
  - `src/main.cpp`
  - `CMakeLists.txt`

### Window & Gravity

| Property | Value |
|---|---|
| Window title | `"Nyon – Breakout Demo"` |
| Resolution | 1280×720 |
| Gravity | `{0, 0}` (zero gravity — ball maintains constant speed) |

### World Configuration

```cpp
world.gravity              = { 0.0f, 0.0f };
world.timeStep             = 1.0f / 60.0f;
world.velocityIterations   = 8;
world.positionIterations   = 3;
world.linearSlop           = 0.5f;
world.maxLinearCorrection  = 2.0f;
world.enableSleep          = false;   // Keep everything awake
world.enableWarmStarting   = true;
world.enableContinuous     = false;
```

### Entities

#### Walls (3 static)

All walls are **invisible** (no `RenderComponent`), with `restitution = 1.0`, `friction = 0.0`.

| Wall | Position | Size |
|---|---|---|
| Top | `{width/2, height + 25}` | `width + 100` × `50` |
| Left | `{-25, height/2}` | `50` × `height + 100` |
| Right | `{width + 25, height/2}` | `50` × `height + 100` |

#### Paddle (user-controlled)

| Property | Value |
|---|---|
| Type | Static (position set directly via transform) |
| Shape | Polygon 120×20 |
| Position | `{width/2, PADDLE_Y}` (initial) |
| **`PADDLE_WIDTH`** | `120.0` |
| **`PADDLE_HEIGHT`** | `20.0` |
| **`PADDLE_Y`** | `80.0` |
| Restitution | `1.0` |
| Color | `{0.2, 0.6, 1.0}` (blue) |

#### Ball (dynamic)

| Property | Value |
|---|---|
| Shape | Circle, radius = 10 |
| Mass | `1.0` |
| Inertia | `10.0` |
| Drag | `0.0` (constant speed) |
| Restitution | `1.0` (perfect bounce) |
| Color | `{1.0, 1.0, 1.0}` (white) |

#### Bricks (static)

| Property | Value |
|---|---|
| Shape | Polygon 40×40 |
| **`BRICK_SIZE`** | `40.0` |
| **`BRICK_GAP`** | `4.0` |
| **`BRICK_START_Y`** | `600.0` |
| **`TARGET_BRICK_MIN`** | `50` |
| **`TARGET_BRICK_MAX`** | `70` |
| Restitution | `1.0` |
| Grid | 14 columns (`SHAPE_MAX_COLS`) × 8 rows (`SHAPE_MAX_ROWS`) |

7-color palette (shuffled per game):

| Color | RGB |
|---|---|
| Red | `{1.0, 0.0, 0.0}` |
| Orange | `{1.0, 0.5, 0.0}` |
| Yellow | `{1.0, 1.0, 0.0}` |
| Green | `{0.0, 1.0, 0.0}` |
| Blue | `{0.0, 0.5, 1.0}` |
| Purple | `{0.6, 0.0, 1.0}` |
| Pink | `{1.0, 0.0, 0.5}` |

##### Brick Generation Algorithm

1. Choose a target brick count randomly between 50–70
2. Pick a random seed cell near the grid center (columns 3–10, rows 2–5)
3. **BFS flood-fill**: repeatedly pick a random placed cell with an unplaced neighbor, then place that neighbor
4. Continue until target reached or no candidates remain
5. Compute world positions centered horizontally on screen
6. Assign shuffled colors from the 7-color palette

### Game Mechanics

#### Paddle Movement

- **Left**: Arrow Left or `A` at `PADDLE_SPEED` = `500.0` px/s
- **Right**: Arrow Right or `D`
- **Clamping**: Paddle position is clamped to `[halfWidth, width - halfWidth]`

#### Ball Launch & Physics

- Ball starts **on the paddle** (Y = `PADDLE_Y + PADDLE_HEIGHT/2 + BALL_RADIUS + 2`)
- **SPACE** launches it with velocity:
  - `velocity.x = BALL_SPEED * random(0.5, 0.9) * (±1 direction)`
  - `velocity.y = BALL_SPEED`
  - **`BALL_SPEED`**: `250.0` px/s
- All entities have `restitution = 1.0` (perfect elastic bounce) — ball bounces forever
- Ball has `drag = 0.0` (constant speed maintained)

#### Collision Detection

```cpp
for (const auto& manifold : world.contactManifolds) {
    // Check if ball is involved
    // If the other entity is a brick → destroy brick, +10 points
}
```

#### Game States

| Condition | Action |
|---|---|
| Ball hits brick | Destroy brick, `m_Score += 10` |
| Ball below Y = 0 | `ResetBall()` — repositions to paddle, clears velocity |
| All bricks destroyed | `m_GameWon = true`, prints `"*** YOU WIN!"` |
| R key pressed | `ResetGame()` — destroys all bricks, regenerates new shape, resets score |

---

## Demo 3: Flappy Bird Demo

A Flappy Bird clone with velocity-based bird tilt, procedural pipe generation, and a three-state menu system.

- **Directory**: `game/flappy-demo/`
- **Files**:
  - `include/FlappyDemo.h`
  - `src/FlappyDemo.cpp`
  - `src/main.cpp`
  - `CMakeLists.txt`

### Window & Gravity

| Property | Value |
|---|---|
| Window title | `"Flappy Bird"` |
| Resolution | 1280×720 |
| Gravity | `{0, -1200}` px/s² |

### World Configuration

```cpp
world.gravity              = { 0.0f, -1200.0f };
world.timeStep             = 1.0f / 60.0f;
world.velocityIterations   = 8;
world.positionIterations   = 3;
world.linearSlop           = 0.3f;
world.maxLinearCorrection  = 4.0f;
world.enableSleep          = false;
world.enableWarmStarting   = true;
world.enableContinuous     = false;
world.baumgarteBeta        = 0.2f;
world.contactHertz         = 30.0f;
world.contactDampingRatio  = 1.0f;
world.contactPushSpeed     = 30.0f;
```

### State Machine

```
MENU ──SPACE──▶ PLAYING ──collision/fell──▶ GAME_OVER ──R──▶ MENU
```

### Entities

#### Camera (fixed)

```cpp
ECS::CameraComponent cam(1280.0f, 720.0f);
cam.isActive = true;
cam.camera.zoom = 1.0f;
cam.camera.position = { 0.0f, 0.0f };
```

#### Bird (dynamic)

| Property | Value |
|---|---|
| Shape | Circle, radius = 18 |
| Position | `{BIRD_X, 500.0}` |
| Mass | `1.0` |
| Restitution | `0.0` (no bounce) |
| Friction | `0.0` |
| Drag | `0.0` |
| Angular damping | `10.0` |
| Lock translation X | `true` (fixed horizontal position) |
| Color | `{1.0, 0.8, 0.0}` (yellow) |

Constants:
- **`BIRD_RADIUS`**: `18.0`
- **`BIRD_X`**: `250.0`
- **`FLAP_IMPULSE`**: `420.0`

#### Pipes (static, scrolled manually)

| Property | Value |
|---|---|
| Shape | Polygon rectangles |
| Color | `{0.2, 0.8, 0.2}` (green) |
| **`PIPE_WIDTH`** | `80.0` |
| **`PIPE_GAP`** | `180.0` |
| **`PIPE_SPAWN_INTERVAL`** | `1.6` seconds |
| **`PIPE_MIN_HEIGHT`** | `60.0` |
| **`PIPE_MAX_HEIGHT`** | `400.0` |
| **`SCROLL_SPEED`** | `200.0` px/s |

Pipes are **not physics-driven** — their X position is decremented manually each tick:
```cpp
t.position.x -= SCROLL_SPEED * deltaTime;
```

### Game Mechanics by State

#### `MENU` State

- Bird velocity and angular velocity are **zeroed** each tick
- Bird Y position oscillates: `500.0 + sin(time * 2.0) * 15.0`
- Bird rotation is reset to `0`
- **SPACE** → transitions to `PLAYING` (falls through to flap on same frame)

#### `PLAYING` State

**Flap (SPACE):**
```cpp
body.velocity.y = 0.0f;                          // Reset vertical velocity
body.ApplyLinearImpulse({ 0.0f, FLAP_IMPULSE }); // Apply +420 upward impulse
```

**Bird tilt:**
```cpp
float targetAngle = body.velocity.y * 0.003f;
t.rotation = std::clamp(targetAngle, -0.5f, 1.5f);
```

**Pipe management:**
- Pipes scroll left at `200` px/s (manual transform update)
- New pipe pair spawned every `1.6` seconds
- Pipe gap center Y is randomly positioned between `PIPE_MIN_HEIGHT + PIPE_GAP/2` and `screenH - PIPE_MIN_HEIGHT - PIPE_GAP/2`
- Pipes destroyed when `t.position.x < -PIPE_WIDTH`

**Collision detection:**
```cpp
for (const auto& manifold : world.contactManifolds) {
    if (manifold.entityIdA == m_BirdEntity || manifold.entityIdB == m_BirdEntity) {
        m_State = GameState::GAME_OVER;
    }
}
```

**Fall death:** Bird Y below `-BIRD_RADIUS * 2` (i.e., `-36`) → `GAME_OVER`

**Scoring:**
- Score increments when a pipe's right edge passes the bird's X position
- Only pipes with Y center `< 360` (bottom pipes) trigger scoring to avoid double-counting
- `m_LastScoredX` prevents re-scoring the same pipe

#### `GAME_OVER` State

- Game loop pauses (no updates beyond camera/cleanup)
- **R** → `ResetGame()` — destroys all pipes, resets bird position/velocity, clears state

### Reset Sequence

```cpp
void FlappyDemo::ResetGame() {
    // Destroy all pipes
    for (auto pipeId : m_Pipes)
        entities.DestroyEntity(pipeId, cs);
    m_Pipes.clear();

    // Reset bird position to (BIRD_X, 500), velocity zeroed
    // Reset state to MENU, score = 0, timers reset
    m_State = GameState::MENU;
    m_Score = 0;
    m_PipeSpawnTimer = 0.0f;
    m_LastScoredX = -1000.0f;
}
```

---

## Demo 4: Tower Stack Demo

A physics-based tower stacking game where blocks slide back and forth and must be dropped precisely onto the growing tower.

- **Directory**: `game/tower-stack-demo/`
- **Files**:
  - `include/TowerStackDemo.h`
  - `src/TowerStackDemo.cpp`
  - `src/main.cpp`
  - `CMakeLists.txt`

### Window & Gravity

| Property | Value |
|---|---|
| Window title | `"Tower Stack"` |
| Resolution | 1280×720 |
| Gravity | `{0, -1200}` px/s² |

### World Configuration

```cpp
world.gravity              = { 0.0f, -1200.0f };
world.timeStep             = 1.0f / 60.0f;
world.velocityIterations   = 8;
world.positionIterations   = 4;
world.linearSlop           = 0.3f;
world.maxLinearCorrection  = 4.0f;
world.enableSleep          = true;
world.enableWarmStarting   = true;
world.enableContinuous     = false;
world.baumgarteBeta        = 0.1f;
```

### State Machine

```
PLAYING ──block falls below -200──▶ FALLING ──after 2s──▶ GAME_OVER ──R──▶ PLAYING
```

### Entities

#### Camera (dynamic follow)

```cpp
ECS::CameraComponent cam(1280.0f, 720.0f);
cam.isActive = true;
cam.camera.zoom = 1.0f;
cam.camera.position = { 0.0f, 0.0f };
```

Camera behavior depends on game state:

**PLAYING:**
```cpp
float targetY = std::max(0.0f, m_HighestBlockY - 400.0f);
float lerpFactor = 1.0f - std::pow(0.01f, deltaTime);
cam.camera.position.y += (targetY - cam.camera.position.y) * lerpFactor;
cam.camera.zoom = 1.0f;
```

**FALLING** (smoothstep animation over 2 seconds):
```cpp
float t = m_FallTimer / FALL_DURATION;
float smoothT = t * t * (3.0f - 2.0f * t);  // smoothstep

cam.camera.zoom = lerp(m_StartCamZoom, FALL_TARGET_ZOOM, smoothT);
cam.camera.position.y = lerp(m_StartCamY, FALL_TARGET_CAM_Y, smoothT);
```

- **`FALL_DURATION`**: `2.0` seconds
- **`FALL_TARGET_ZOOM`**: `0.3`
- **`FALL_TARGET_CAM_Y`**: `-200.0`

#### Platform (static base)

| Property | Value |
|---|---|
| Shape | Polygon 400×30 |
| Position | `{640, PLATFORM_Y}` |
| **`PLATFORM_WIDTH`** | `400.0` |
| **`PLATFORM_HEIGHT`** | `30.0` |
| **`PLATFORM_Y`** | `50.0` |
| Friction | `0.6` |
| Restitution | `0.0` |
| Color | `{0.4, 0.4, 0.4}` (grey) |

#### Active Block (no physics, slides)

The active block has **no `PhysicsBodyComponent` or `ColliderComponent`** — it is rendered only and slides via direct transform manipulation.

| Property | Value |
|---|---|
| Shape | Rendered 120×30 rectangle |
| Spawn Y | `m_HighestBlockY + BLOCK_HEIGHT + 1.0` |
| Slide speed | `350.0` px/s |
| Bounce bounds | X = `100` … `1180` |
| **`BLOCK_WIDTH`** | `120.0` |
| **`BLOCK_HEIGHT`** | `30.0` |

Color cycle (7 colors, incrementing `m_ColorIndex` per spawn):

| Index | Color | RGB |
|---|---|---|
| 0 | Blue | `{0.2, 0.6, 1.0}` |
| 1 | Green | `{0.2, 0.8, 0.3}` |
| 2 | Yellow | `{1.0, 0.8, 0.0}` |
| 3 | Orange | `{1.0, 0.4, 0.2}` |
| 4 | Purple | `{0.8, 0.3, 1.0}` |
| 5 | Red | `{1.0, 0.2, 0.2}` |
| 6 | Cyan | `{0.2, 1.0, 0.8}` |

#### Placed Blocks (dynamic)

When dropped, the active block gets a `PhysicsBodyComponent` and `ColliderComponent` added:

| Property | Value |
|---|---|
| Shape | Polygon 120×30 |
| Mass | `1.0` |
| Inertia | `mass * (BLOCK_WIDTH² + BLOCK_HEIGHT²) / 12.0` |
| Friction | `0.6` |
| Restitution | `0.0` |
| Density | `0.5` |
| Drag | `0.0` |
| Angular damping | `0.5` |

### Game Mechanics

#### `PLAYING` State

1. **Active block slides** left-right at `350` px/s
2. **Bounces** at screen edges: X = `100` (left) and `1180` (right)
3. Block Y is locked to `m_HighestBlockY + BLOCK_HEIGHT + 1.0`
4. **SPACE** → `DropActiveBlock()`:
   - Records current position
   - Adds `PhysicsBodyComponent` (mass 1, inertia calculated, drag 0)
   - Adds `ColliderComponent` (polygon 120×30, friction 0.6, restitution 0, density 0.5)
   - Updates `m_HighestBlockY` if dropped higher than previous
   - Increments score (`m_Score++`)
   - Spawns next active block via `SpawnActiveBlock()`
5. `CheckGameOver()` — if any placed block's Y < `-200.0`, transitions to `FALLING`

#### `FALLING` State

1. `m_FallTimer` accumulates
2. Camera animates: zoom from current → `0.3`, Y from current → `-200.0` (smoothstep over 2s)
3. After `FALL_DURATION` (2s) elapsed → transitions to `GAME_OVER`

#### `GAME_OVER` State

- Game loop paused (only cleanup and input processed)
- **R** → `ResetGame()`

### Cleanup

Blocks below Y = `-1500.0` are destroyed and removed from the tracking list continuously.

### Reset Sequence

```cpp
void TowerStackDemo::ResetGame() {
    // Destroy all placed blocks and active block
    // Reset camera to {0, 0} with zoom 1.0
    // Reset state to PLAYING, score = 0
    // Reset slide direction, highest block Y, color index
    // Spawn new active block
}
```

---

## Cross-Demo Features Table

| Feature | Simple Physics | Breakout | Flappy Bird | Tower Stack |
|---|---|---|---|---|
| **Gravity** | `{0, -980}` | `{0, 0}` | `{0, -1200}` | `{0, -1200}` |
| **Restitution** | `0.8` | `1.0` | `0.0` | `0.0` |
| **Camera** | Default | Default | Custom (fixed) | Custom (follow + zoom) |
| **State machine** | None | Implicit (win) | `MENU → PLAYING → GAME_OVER` | `PLAYING → FALLING → GAME_OVER` |
| **Dynamic spawning** | Auto-timer + click | BFS at game start | Pipes at interval | After each drop |
| **Collision detection** | `contactManifolds` | `contactManifolds` | `contactManifolds` | Via falling (no explicit contact) |
| **Player movement** | WASD | A/D + Space | Space (flap) | Space (drop) |
| **Extra** | Grounded check, slope physics | BFS brick layout, score | Bird rotation, scrolling pipes | Camera animation, smoothstep |

### Physics Configuration Comparison

| Parameter | Simple Physics | Breakout | Flappy Bird | Tower Stack |
|---|---|---|---|---|
| `velocityIterations` | 8 | 8 | 8 | 8 |
| `positionIterations` | 3 | 3 | 3 | 4 |
| `linearSlop` | 0.05 | 0.5 | 0.3 | 0.3 |
| `maxLinearCorrection` | 2.0 | 2.0 | 4.0 | 4.0 |
| `enableSleep` | true | false | false | true |
| `enableWarmStarting` | true | true | true | true |
| `baumgarteBeta` | — | — | 0.2 | 0.1 |
| `contactHertz` | — | — | 30.0 | — |
| `contactDampingRatio` | — | — | 1.0 | — |
| `contactPushSpeed` | — | — | 30.0 | — |

---

## Physics World Contact Manifolds API

All demos access collision results by iterating the `contactManifolds` vector on the `PhysicsWorldComponent`:

```cpp
// Access collision results each frame
auto& world = GetComponentStore().GetComponent<Nyon::ECS::PhysicsWorldComponent>(worldEntity);
for (auto& manifold : world.contactManifolds) {
    // manifold.entityIdA, manifold.entityIdB — the two colliding entities
    // manifold.points — vector of ContactPoint structs
    // manifold.normal — contact normal (points from A toward B)
    // manifold.friction — combined friction coefficient
    // manifold.restitution — combined restitution coefficient

    for (auto& point : manifold.points) {
        // point.position — world-space contact position
        // point.separation — penetration depth (positive = separated, negative = overlapping)
        // point.normalImpulse — accumulated normal impulse
        // point.tangentImpulse — accumulated friction impulse
    }
}
```

The manifold `normal` always points **from entity A toward entity B**. When checking if the player (entity A or B) is grounded, flip the normal if the player is entity A so the comparison is always in the "toward player" direction.

---

*Generated from source code. All values are extracted directly from the demo source files under `game/`.*
