#include "nyon/graphics/ParticleRenderer.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <filesystem>
#include <unistd.h>
#include <limits.h>

namespace Nyon::Graphics {

// Get shader directory relative to executable
std::string GetShaderDir() {
    char exePath[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", exePath, sizeof(exePath)-1);
    if (len == -1) {
        // Fallback: assume run from build/game/simple-physics-demo/
        return "../../../engine/assets/shaders/";
    }
    exePath[len] = '\0';
    std::filesystem::path exe(exePath);
    // exe is .../build/game/simple-physics-demo/nyon_simple_physics_demo
    // Go up to Nyon, then to engine/assets/shaders
    auto nyonDir = exe.parent_path().parent_path().parent_path().parent_path(); // exe -> simple-physics-demo -> game -> build -> Nyon
    auto shaderDir = nyonDir / "engine" / "assets" / "shaders";
    return shaderDir.string() + "/";
}

ParticleRenderer::ParticleRenderer() = default;

ParticleRenderer::~ParticleRenderer() {
    Shutdown();
}

void ParticleRenderer::Init() {
    BuildCircleMesh();
    BuildQuadMesh();
    SetupInstanceBuffer();
    SetupShaders();
}

void ParticleRenderer::Shutdown() {
    if (m_CircleVAO) glDeleteVertexArrays(1, &m_CircleVAO);
    if (m_CircleMeshVBO) glDeleteBuffers(1, &m_CircleMeshVBO);
    if (m_CircleInstVBO) glDeleteBuffers(1, &m_CircleInstVBO);
    if (m_QuadVAO) glDeleteVertexArrays(1, &m_QuadVAO);
    if (m_QuadMeshVBO) glDeleteBuffers(1, &m_QuadMeshVBO);
    if (m_QuadInstVBO) glDeleteBuffers(1, &m_QuadInstVBO);
    if (m_CircleShader) glDeleteProgram(m_CircleShader);
    if (m_QuadShader) glDeleteProgram(m_QuadShader);
}

void ParticleRenderer::BeginFrame() {
    m_CircleInstances.clear();
    m_QuadInstances.clear();
}

void ParticleRenderer::SubmitCircle(float x, float y, float radius, float r, float g, float b) {
    if (m_CircleInstances.size() >= MAX_PARTICLES) return;
    ParticleInstance& inst = m_CircleInstances.emplace_back();
    inst.x = x; inst.y = y;
    inst.angle = 0.0f;
    inst.radius = radius;
    inst.r = r; inst.g = g; inst.b = b;
    inst.aspectRatio = 1.0f;
}

void ParticleRenderer::SubmitQuad(float x, float y, float halfExtent, float angle,
                                  float r, float g, float b, float aspectRatio) {
    if (m_QuadInstances.size() >= MAX_PARTICLES) return;
    ParticleInstance& inst = m_QuadInstances.emplace_back();
    inst.x = x; inst.y = y;
    inst.angle = angle;
    inst.radius = halfExtent;
    inst.r = r; inst.g = g; inst.b = b;
    inst.aspectRatio = aspectRatio;
}

void ParticleRenderer::Flush(const glm::mat4& viewProjection) {
    // Flush circles
    if (!m_CircleInstances.empty()) {
        glBindBuffer(GL_ARRAY_BUFFER, m_CircleInstVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        m_CircleInstances.size() * sizeof(ParticleInstance),
                        m_CircleInstances.data());

        glUseProgram(m_CircleShader);
        glUniformMatrix4fv(glGetUniformLocation(m_CircleShader, "u_VP"),
                           1, GL_FALSE, &viewProjection[0][0]);

        glBindVertexArray(m_CircleVAO);
        glDrawArraysInstanced(GL_TRIANGLES,
                              0,
                              CIRCLE_SEGMENTS * 3,
                              static_cast<GLsizei>(m_CircleInstances.size()));
        glBindVertexArray(0);

        m_LastCircleCount = static_cast<uint32_t>(m_CircleInstances.size());
        m_CircleInstances.clear();
    }

    // Flush quads
    if (!m_QuadInstances.empty()) {
        glBindBuffer(GL_ARRAY_BUFFER, m_QuadInstVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        m_QuadInstances.size() * sizeof(ParticleInstance),
                        m_QuadInstances.data());

        glUseProgram(m_QuadShader);
        glUniformMatrix4fv(glGetUniformLocation(m_QuadShader, "u_VP"),
                           1, GL_FALSE, &viewProjection[0][0]);

        glBindVertexArray(m_QuadVAO);
        glDrawArraysInstanced(GL_TRIANGLES,
                              0,
                              QUAD_VERTEX_COUNT,
                              static_cast<GLsizei>(m_QuadInstances.size()));
        glBindVertexArray(0);

        m_LastQuadCount = static_cast<uint32_t>(m_QuadInstances.size());
        m_QuadInstances.clear();
    }
}

void ParticleRenderer::BuildCircleMesh() {
    const int vertCount = CIRCLE_SEGMENTS * 3;
    std::vector<float> verts;
    verts.reserve(vertCount * 2);

    const float step = 2.0f * 3.14159265f / static_cast<float>(CIRCLE_SEGMENTS);
    for (int i = 0; i < CIRCLE_SEGMENTS; ++i) {
        float a0 = step * i;
        float a1 = step * (i + 1);
        // Center
        verts.push_back(0.0f); verts.push_back(0.0f);
        // Perimeter point 0
        verts.push_back(std::cos(a0)); verts.push_back(std::sin(a0));
        // Perimeter point 1
        verts.push_back(std::cos(a1)); verts.push_back(std::sin(a1));
    }

    glGenVertexArrays(1, &m_CircleVAO);
    glGenBuffers(1, &m_CircleMeshVBO);

    glBindVertexArray(m_CircleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_CircleMeshVBO);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void ParticleRenderer::BuildQuadMesh() {
    // Unit quad: two triangles
    std::vector<float> verts = {
        -1.0f, -1.0f,
         1.0f, -1.0f,
         1.0f,  1.0f,
        -1.0f, -1.0f,
         1.0f,  1.0f,
        -1.0f,  1.0f
    };

    glGenVertexArrays(1, &m_QuadVAO);
    glGenBuffers(1, &m_QuadMeshVBO);

    glBindVertexArray(m_QuadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_QuadMeshVBO);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void ParticleRenderer::SetupInstanceBuffer() {
    // Circle instance VBO
    glBindVertexArray(m_CircleVAO);
    glGenBuffers(1, &m_CircleInstVBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_CircleInstVBO);
    glBufferData(GL_ARRAY_BUFFER,
                 MAX_PARTICLES * sizeof(ParticleInstance),
                 nullptr, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(ParticleInstance), (void*)offsetof(ParticleInstance, x));
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);

    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(ParticleInstance), (void*)offsetof(ParticleInstance, angle));
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2, 1);

    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(ParticleInstance), (void*)offsetof(ParticleInstance, r));
    glEnableVertexAttribArray(3);
    glVertexAttribDivisor(3, 1);

    glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, sizeof(ParticleInstance), (void*)offsetof(ParticleInstance, aspectRatio));
    glEnableVertexAttribArray(4);
    glVertexAttribDivisor(4, 1);

    glBindVertexArray(0);

    // Quad instance VBO (similar)
    glBindVertexArray(m_QuadVAO);
    glGenBuffers(1, &m_QuadInstVBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_QuadInstVBO);
    glBufferData(GL_ARRAY_BUFFER,
                 MAX_PARTICLES * sizeof(ParticleInstance),
                 nullptr, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(ParticleInstance), (void*)offsetof(ParticleInstance, x));
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);

    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(ParticleInstance), (void*)offsetof(ParticleInstance, angle));
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2, 1);

    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(ParticleInstance), (void*)offsetof(ParticleInstance, r));
    glEnableVertexAttribArray(3);
    glVertexAttribDivisor(3, 1);

    glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, sizeof(ParticleInstance), (void*)offsetof(ParticleInstance, aspectRatio));
    glEnableVertexAttribArray(4);
    glVertexAttribDivisor(4, 1);

    glBindVertexArray(0);
}

void ParticleRenderer::SetupShaders() {
    // Load and compile circle shader
    auto loadShader = [](const std::string& filename) {
        std::string path = GetShaderDir() + filename;
        std::ifstream file(path);
        if (!file.is_open()) throw std::runtime_error("Failed to open shader: " + path);
        std::stringstream ss;
        ss << file.rdbuf();
        return ss.str();
    };

    std::string circleVertSrc = loadShader("circle_instanced.vert");
    std::string circleFragSrc = loadShader("circle_instanced.frag");
    std::string quadVertSrc = loadShader("quad_instanced.vert");
    std::string quadFragSrc = loadShader("quad_instanced.frag");

    auto compileShader = [](GLenum type, const std::string& src) {
        GLuint shader = glCreateShader(type);
        const char* srcPtr = src.c_str();
        glShaderSource(shader, 1, &srcPtr, nullptr);
        glCompileShader(shader);
        GLint success;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glGetShaderInfoLog(shader, 512, nullptr, infoLog);
            throw std::runtime_error("Shader compilation failed: " + std::string(infoLog));
        }
        return shader;
    };

    auto linkProgram = [](GLuint vert, GLuint frag) {
        GLuint program = glCreateProgram();
        glAttachShader(program, vert);
        glAttachShader(program, frag);
        glLinkProgram(program);
        GLint success;
        glGetProgramiv(program, GL_LINK_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glGetProgramInfoLog(program, 512, nullptr, infoLog);
            throw std::runtime_error("Program linking failed: " + std::string(infoLog));
        }
        glDeleteShader(vert);
        glDeleteShader(frag);
        return program;
    };

    GLuint circleVert = compileShader(GL_VERTEX_SHADER, circleVertSrc);
    GLuint circleFrag = compileShader(GL_FRAGMENT_SHADER, circleFragSrc);
    m_CircleShader = linkProgram(circleVert, circleFrag);

    GLuint quadVert = compileShader(GL_VERTEX_SHADER, quadVertSrc);
    GLuint quadFrag = compileShader(GL_FRAGMENT_SHADER, quadFragSrc);
    m_QuadShader = linkProgram(quadVert, quadFrag);
}

} // namespace Nyon::Graphics