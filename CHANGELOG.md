# Changelog

All notable changes to the Nyon Game Engine will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Full Entity-Component-System (ECS) architecture implementation
- ECS core infrastructure: EntityManager, ComponentStore, System base class
- Component system with Transform, PhysicsBody, Collider, Render, and Behavior components
- ECS systems: PhysicsSystem, CollisionSystem, RenderSystem, and InputSystem
- ECSApplication base class for ECS-enabled games
- Multi-shape collision support (Polygon, Circle, Capsule, Composite)
- Physics materials with friction, restitution, and density properties
- Behavior system with lambda-based attachable logic
- SystemManager for orchestrating system execution order
- Comprehensive tutorial documentation with complete examples
- Updated README with detailed ECS documentation

### Changed
- Migrated existing smash-bros-clone game to use new ECS architecture
- Enhanced collision detection with broader shape support
- Improved physics system with better material properties
- Refactored component storage to use Structure of Arrays for better performance
- Updated build system to accommodate new ECS files

### Fixed
- Various compilation warnings and errors in component templates
- Function name conflicts in ComponentStore implementation
- Missing includes in system header files
- Physics body to transform synchronization issues

## [1.0.0] - 2024-01-XX

### Added
- Initial release of Nyon Game Engine
- Core Application framework with fixed-timestep game loop
- 2D graphics rendering system with quad rendering
- Basic physics system with AABB and SAT collision detection
- Input management system for keyboard and mouse
- Mathematical utilities (Vector2, Vector3)
- Sample smash-bros-clone game demonstrating engine usage
- Basic project structure and build system
- Initial documentation and README

### Features
- Window management and OpenGL context creation
- Batch rendering for improved performance
- Gravity simulation and physics integration
- Collision detection and response
- Keyboard and mouse input handling
- Entity management (basic structure)
- Component-based architecture foundation

[Unreleased]: https://github.com/yourusername/Nyon/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/yourusername/Nyon/releases/tag/v1.0.0