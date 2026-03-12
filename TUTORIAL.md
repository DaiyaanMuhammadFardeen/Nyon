## Nyon Engine – Developer Tutorial Guide

This repository now includes a structured, multi-part tutorial for building games and physics demos on top of the Nyon engine.

### Main tutorial series

The primary, step-by-step guide lives in the `tutorial/` folder. Follow these files in order:

1. `tutorial/README.md` – Overview and roadmap.
2. `tutorial/01-project-setup.md` – Create a new game target, hook it into CMake, and run an `ECSApplication` window.
3. `tutorial/02-ecs-basics-and-entities.md` – Work with entities, components, and the ECS update hooks.
4. `tutorial/03-physics-and-collisions.md` – Configure `PhysicsWorldComponent`, rigid bodies, and colliders.
5. `tutorial/04-rendering-and-debug-views.md` – Use the built-in render systems and `Renderer2D`, plus physics debug visualization.
6. `tutorial/05-input-and-gameplay-loop.md` – Handle keyboard/mouse input and structure gameplay logic.
7. `tutorial/06-building-a-simple-physics-demo.md` – Full worked example of a small physics playground.

These documents are intended for **engine users** (game/demo authors) rather than engine contributors.

### Additional references

For more detailed information about the physics internals, see:

- `PHYSICS_QUICK_REFERENCE.md` – High-level summary of the physics pipeline and major components.
- `NYON_PHYSICS_ENGINE_REPORT(1).md` – Deep dive into the physics engine design and refactors.

Those files are optional reading if you only need to *use* the engine, but can be valuable if you are extending or debugging the physics layer.

