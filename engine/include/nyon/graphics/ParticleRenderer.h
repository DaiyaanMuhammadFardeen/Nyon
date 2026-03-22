#pragma once
#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <cstdint>
namespace Nyon::Graphics {
    struct alignas(16) ParticleInstance {
        float x, y;           
        float angle;          
        float radius;         
        float r, g, b;        
        float aspectRatio;     };
    static_assert(sizeof(ParticleInstance) == 32, "ParticleInstance must be 32 bytes");
    class ParticleRenderer {
    public:
        static constexpr uint32_t MAX_PARTICLES = 4'000'000;
        static constexpr int CIRCLE_SEGMENTS = 16;
        static constexpr int QUAD_VERTEX_COUNT = 6;
        ParticleRenderer();
        ~ParticleRenderer();
        ParticleRenderer(const ParticleRenderer&) = delete;
        ParticleRenderer& operator=(const ParticleRenderer&) = delete;
        void Init();
        void Shutdown();
        void BeginFrame();
        void SubmitCircle(float x, float y, float radius, float r, float g, float b);
        void SubmitQuad(float x, float y, float halfExtent, float angle,
                        float r, float g, float b, float aspectRatio = 1.0f);
        void Flush(const glm::mat4& viewProjection);
        uint32_t GetLastFrameCircleCount()  const { return m_LastCircleCount;  }
        uint32_t GetLastFrameQuadCount()    const { return m_LastQuadCount;    }
    private:
        void BuildCircleMesh();
        void BuildQuadMesh();
        void SetupInstanceBuffer();
        void SetupShaders();
        GLuint m_CircleVAO      = 0;
        GLuint m_CircleMeshVBO  = 0;    
        GLuint m_CircleInstVBO  = 0;    
        GLuint m_QuadVAO        = 0;
        GLuint m_QuadMeshVBO    = 0;    
        GLuint m_QuadInstVBO    = 0;    
        GLuint m_CircleShader   = 0;
        GLuint m_QuadShader     = 0;
        std::vector<ParticleInstance> m_CircleInstances;
        std::vector<ParticleInstance> m_QuadInstances;
        uint32_t m_LastCircleCount = 0;
        uint32_t m_LastQuadCount   = 0; }; }  