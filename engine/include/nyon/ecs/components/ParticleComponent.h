#pragma once

namespace Nyon {

/**
 * @brief Particle component for particle system simulation
 * 
 * Contains all properties needed for particle physics and rendering.
 * Used by ParticleRenderSystem and particle simulation components.
 */
struct Particle {
    float x, y;       // Position
    float vx, vy;     // Velocity
    float radius;     // Particle radius
    float r, g, b;    // Color (RGB)
    float mass;       // Particle mass
};

}
