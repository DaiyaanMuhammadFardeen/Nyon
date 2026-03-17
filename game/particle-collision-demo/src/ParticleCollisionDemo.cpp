#include "ParticleCollisionDemo.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include <iostream>
#include <algorithm>

namespace Nyon {

ParticleCollisionDemo::ParticleCollisionDemo()
    : ECSApplication("Particle Collision Demo", 1280, 720)
    , m_Rng(std::random_device{}())
{
}

void ParticleCollisionDemo::OnECSStart() {
    std::cout << "[DEMO] Initializing particle collision demo with " << NUM_PARTICLES << " particles\n";

    // Initialize particles
    InitializeParticles();

    // Create particle render system
    GetSystemManager().AddSystem(std::make_unique<ECS::ParticleRenderSystem>());

    std::cout << "[DEMO] Particle collision demo initialized\n";
}

void ParticleCollisionDemo::OnECSUpdate(float deltaTime) {
    // Update particle simulation
    UpdateParticles(deltaTime);

    // Update render system with current particles
    auto* particleSystem = GetSystemManager().GetSystem<ECS::ParticleRenderSystem>();
    if (particleSystem) {
        particleSystem->SetParticles(m_Particles);
    }
}

void ParticleCollisionDemo::InitializeParticles() {
    m_Particles.resize(NUM_PARTICLES);

    std::uniform_real_distribution<float> posX(50.0f, WORLD_WIDTH - 50.0f);
    std::uniform_real_distribution<float> posY(50.0f, WORLD_HEIGHT - 200.0f);
    std::uniform_real_distribution<float> vel(-200.0f, 200.0f);
    std::uniform_real_distribution<float> radius(10.0f, 50.0f);
    std::uniform_real_distribution<float> color(0.0f, 1.0f);

    for (auto& p : m_Particles) {
        p.x = posX(m_Rng);
        p.y = posY(m_Rng);
        p.vx = vel(m_Rng);
        p.vy = vel(m_Rng);
        p.radius = radius(m_Rng);
        p.r = color(m_Rng);
        p.g = color(m_Rng);
        p.b = color(m_Rng);
        p.mass = p.radius * p.radius; // Area-based mass
    }

    std::cout << "[DEMO] Initialized " << m_Particles.size() << " particles\n";
}

void ParticleCollisionDemo::UpdateParticles(float deltaTime) {
    // Apply gravity and update positions
    for (auto& p : m_Particles) {
        p.vy += GRAVITY * deltaTime;
        p.x += p.vx * deltaTime;
        p.y += p.vy * deltaTime;

        // Apply damping
        p.vx *= DAMPING;
        p.vy *= DAMPING;

        // Bounce off walls
        if (p.x - p.radius < 0) {
            p.x = p.radius;
            p.vx = -p.vx * BOUNCE_DAMPING;
        } else if (p.x + p.radius > WORLD_WIDTH) {
            p.x = WORLD_WIDTH - p.radius;
            p.vx = -p.vx * BOUNCE_DAMPING;
        }

        if (p.y - p.radius < 0) {
            p.y = p.radius;
            p.vy = -p.vy * BOUNCE_DAMPING;
        } else if (p.y + p.radius > WORLD_HEIGHT) {
            p.y = WORLD_HEIGHT - p.radius;
            p.vy = -p.vy * BOUNCE_DAMPING;
        }
    }

    // Simple collision detection (brute force for demo - not efficient for 100k)
    // For performance, we'll skip detailed collisions and just do basic repulsion
    HandleCollisions();
}

void ParticleCollisionDemo::HandleCollisions() {
    // For 100k particles, brute force is O(n^2) = 10^10 operations - impossible
    // Instead, use a simplified approach: check only nearby particles using grid

    const float CELL_SIZE = 50.0f;
    const int GRID_WIDTH = static_cast<int>(WORLD_WIDTH / CELL_SIZE) + 1;
    const int GRID_HEIGHT = static_cast<int>(WORLD_HEIGHT / CELL_SIZE) + 1;

    std::vector<std::vector<std::vector<size_t>>> grid(GRID_WIDTH, std::vector<std::vector<size_t>>(GRID_HEIGHT));

    // Assign particles to grid cells
    for (size_t i = 0; i < m_Particles.size(); ++i) {
        const auto& p = m_Particles[i];
        int cellX = static_cast<int>(p.x / CELL_SIZE);
        int cellY = static_cast<int>(p.y / CELL_SIZE);
        cellX = std::clamp(cellX, 0, GRID_WIDTH - 1);
        cellY = std::clamp(cellY, 0, GRID_HEIGHT - 1);
        grid[cellX][cellY].push_back(i);
    }

    // Check collisions within each cell and neighboring cells
    for (int cx = 0; cx < GRID_WIDTH; ++cx) {
        for (int cy = 0; cy < GRID_HEIGHT; ++cy) {
            // Check this cell and 8 neighbors
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    int nx = cx + dx;
                    int ny = cy + dy;
                    if (nx < 0 || nx >= GRID_WIDTH || ny < 0 || ny >= GRID_HEIGHT) continue;

                    // Check collisions between particles in cell (cx,cy) and (nx,ny)
                    for (size_t i : grid[cx][cy]) {
                        for (size_t j : grid[nx][ny]) {
                            if (i >= j) continue; // Avoid double checking

                            auto& p1 = m_Particles[i];
                            auto& p2 = m_Particles[j];

                            float dx_pos = p2.x - p1.x;
                            float dy_pos = p2.y - p1.y;
                            float dist_sq = dx_pos * dx_pos + dy_pos * dy_pos;
                            float min_dist = p1.radius + p2.radius;
                            float min_dist_sq = min_dist * min_dist;

                            if (dist_sq < min_dist_sq && dist_sq > 0.001f) {
                                // Collision detected - simple elastic collision
                                float dist = std::sqrt(dist_sq);
                                float nx = dx_pos / dist;
                                float ny = dy_pos / dist;

                                // Separate particles
                                float overlap = min_dist - dist;
                                float separation_x = nx * overlap * 0.5f;
                                float separation_y = ny * overlap * 0.5f;

                                p1.x -= separation_x;
                                p1.y -= separation_y;
                                p2.x += separation_x;
                                p2.y += separation_y;

                                // Velocity impulse
                                float relative_vx = p2.vx - p1.vx;
                                float relative_vy = p2.vy - p1.vy;
                                float velocity_along_normal = relative_vx * nx + relative_vy * ny;

                                if (velocity_along_normal < 0) {
                                    float restitution = 0.8f;
                                    float impulse = -(1.0f + restitution) * velocity_along_normal / (1.0f / p1.mass + 1.0f / p2.mass);

                                    p1.vx -= impulse * nx / p1.mass;
                                    p1.vy -= impulse * ny / p1.mass;
                                    p2.vx += impulse * nx / p2.mass;
                                    p2.vy += impulse * ny / p2.mass;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

} // namespace Nyon