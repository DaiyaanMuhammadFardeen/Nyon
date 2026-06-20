#pragma once
#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <cstdint>

namespace Nyon::Graphics {

    // Per-instance data uploaded to the GPU.
    // Kept tightly packed: 32 bytes per particle — fits exactly in one cache line half.
    struct alignas(16) ParticleInstance {
        float x, y;          // world position (8 bytes)
        float angle;         // rotation in radians (4 bytes)
        float radius;        // for circles; half-extent for quads (4 bytes)
        float r, g, b;       // color (12 bytes)
        float aspectRatio;   // width/height for quads; 1.0 for circles (4 bytes)
        // Total: 32 bytes
    };

    static_assert(sizeof(ParticleInstance) == 32, "ParticleInstance must be 32 bytes");

    class ParticleRenderer {
    public:
        // Maximum number of particles. 4M = 128 MB of instance data.
        static constexpr uint32_t MAX_PARTICLES = 4'000'000;
        // Circle mesh resolution. 16 sides is enough for smooth appearance at small sizes.
        static constexpr int CIRCLE_SEGMENTS = 16;
        // Quad uses 6 vertices (2 triangles).
        static constexpr int QUAD_VERTEX_COUNT = 6;

        ParticleRenderer();
        ~ParticleRenderer();

        // Non-copyable, non-movable (owns GPU resources).
        ParticleRenderer(const ParticleRenderer&) = delete;
        ParticleRenderer& operator=(const ParticleRenderer&) = delete;

        void Init();
        void Shutdown();

        // Called once per frame before submitting any particles.
        void BeginFrame();

        // Submit a circle instance. Batched until Flush().
        void SubmitCircle(float x, float y, float radius, float r, float g, float b);

        // Submit a quad instance. aspectRatio = width / height.
        void SubmitQuad(float x, float y, float halfExtent, float angle,
                        float r, float g, float b, float aspectRatio = 1.0f);

        // Upload all pending instances and issue instanced draw calls.
        void Flush(const glm::mat4& viewProjection);

        uint32_t GetLastFrameCircleCount()  const { return m_LastCircleCount;  }
        uint32_t GetLastFrameQuadCount()    const { return m_LastQuadCount;    }

    private:
        void BuildCircleMesh();
        void BuildQuadMesh();
        void SetupInstanceBuffer();
        void SetupShaders();

        // Circle VAO
        GLuint m_CircleVAO      = 0;
        GLuint m_CircleMeshVBO  = 0;   // static mesh vertices (unit circle)
        GLuint m_CircleInstVBO  = 0;   // per-instance data (dynamic, persistently mapped)

        // Quad VAO
        GLuint m_QuadVAO        = 0;
        GLuint m_QuadMeshVBO    = 0;   // static mesh vertices (unit quad)
        GLuint m_QuadInstVBO    = 0;   // per-instance data

        // Shader programs
        GLuint m_CircleShader   = 0;
        GLuint m_QuadShader     = 0;

        // CPU-side staging buffers (written this frame, then uploaded)
        std::vector<ParticleInstance> m_CircleInstances;
        std::vector<ParticleInstance> m_QuadInstances;

        // Stats
        uint32_t m_LastCircleCount = 0;
        uint32_t m_LastQuadCount   = 0;
    };

} // namespace Nyon::Graphics