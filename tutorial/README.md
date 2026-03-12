## Nyon Engine Tutorials

Welcome to the Nyon 2D physics and game engine tutorial series. These guides walk you through creating your own game or physics demo on top of the engine.

### Who this is for

- **Engine users**: You want to *use* Nyon to build games/demos, not modify the engine itself.
- **C++ developers**: You are comfortable with basic CMake and C++17.

### Tutorial structure

Read the documents in this order:

1. **`01-project-setup.md`** – Create a new game target, hook it into CMake, and run a blank window.
2. **`02-ecs-basics-and-entities.md`** – Learn how `ECSApplication`, entities, and components (like `TransformComponent`) fit together.
3. **`03-physics-and-collisions.md`** – Add `PhysicsWorldComponent`, `PhysicsBodyComponent`, and `ColliderComponent` to simulate rigid bodies.
4. **`04-rendering-and-debug-views.md`** – Render your world using the built–in render system and `Renderer2D`, plus physics debug overlays.
5. **`05-input-and-gameplay-loop.md`** – Use `InputManager` and the update hooks to implement gameplay.
6. **`06-building-a-simple-physics-demo.md`** – A complete worked example: a falling–boxes physics playground.

You can skim or skip sections if you are already familiar with ECS or CMake, but following the order once is recommended.

### High–level architecture (quick recap)

- **Application layer**: `Nyon::Application` manages the window and the fixed–timestep game loop with interpolation.
- **ECS layer**: `Nyon::ECSApplication` extends `Application` with `EntityManager`, `ComponentStore`, and `SystemManager`.
- **Physics**: `PhysicsWorldComponent`, `PhysicsBodyComponent`, `ColliderComponent`, and physics systems handle simulation and collisions.
- **Rendering**: `RenderSystem` and `Renderer2D` draw entities and debug information, using an orthographic camera.
- **Input**: `InputSystem` and `Utils::InputManager` provide keyboard and mouse helpers.

Each tutorial document points to concrete APIs and gives small but realistic code snippets you can copy into your own project.

