// =============================================================================
// Renderer2D.cpp  —  GPU-accelerated instanced renderer
// Nyon Engine
//
// Public API (Renderer2D.h) is unchanged.  Only this file changes.
//
// Pipeline summary:
//   QuadPipeline    — glDrawArraysInstanced, unit quad mesh, instance: pos/size/origin/angle/color
//   CirclePipeline  — glDrawArraysInstanced, unit circle fan, SDF frag, instance: center/radius/color/mode
//   LinePipeline    — glDrawArraysInstanced, unit rect mesh, instance: p0/p1/color/thickness
//   CapsulePipeline — glDrawArraysInstanced, pre-built capsule mesh, instance: c1/c2/radius/color/mode
//   PolyFillBuffer  — CPU fan tessellation → persistent-mapped VBO, no glBufferData
//   PolyLineBuffer  — CPU outline tessellation → persistent-mapped VBO, no glBufferData
//
// All instance VBOs use GL_ARB_buffer_storage (GL 4.4+) persistent coherent mapping
// with NUM_FRAMES=3 triple-buffering guarded by GLsync fences.
// =============================================================================

#include "nyon/graphics/Renderer2D.h"
#include "nyon/core/Application.h"

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <filesystem>
#include <unistd.h>
#include <limits.h>
#include <fstream>

// GL 4.4 persistent map flags — define manually in case the glad header predates them
#ifndef GL_MAP_PERSISTENT_BIT
#define GL_MAP_PERSISTENT_BIT  0x0040
#endif
#ifndef GL_MAP_COHERENT_BIT
#define GL_MAP_COHERENT_BIT    0x0080
#endif
#ifndef GL_MAP_WRITE_BIT
#define GL_MAP_WRITE_BIT       0x0002
#endif

constexpr float PI = 3.14159265358979323846f;

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

namespace Nyon::Graphics {

// =============================================================================
// Shader file loader
// =============================================================================

static std::string LoadShaderSource(const std::string& filename)
{
    std::string filepath = GetShaderDir() + filename;
    std::ifstream file(filepath, std::ios::in | std::ios::binary);
    if (!file.is_open())
        throw std::runtime_error("Renderer2D: failed to open shader: " + filepath);
    std::stringstream buf;
    buf << file.rdbuf();
    return buf.str();
}

// =============================================================================
// Camera2D  (unchanged from original)
// =============================================================================

glm::mat4 Camera2D::GetViewMatrix() const
{
    glm::mat4 v = glm::mat4(1.0f);
    v = glm::rotate(v, -rotation, glm::vec3(0.0f, 0.0f, 1.0f));
    v = glm::translate(v, glm::vec3(-position.x, -position.y, 0.0f));
    return v;
}

glm::mat4 Camera2D::GetProjectionMatrix(float sw, float sh) const
{
    return glm::ortho(0.0f, sw / zoom, 0.0f, sh / zoom, nearPlane, farPlane);
}

glm::mat4 Camera2D::GetViewProjectionMatrix(float sw, float sh) const
{
    return GetProjectionMatrix(sw, sh) * GetViewMatrix();
}

Math::Vector2 Camera2D::ScreenToWorld(Math::Vector2 sp, float sw, float sh) const
{
    float nx = (2.0f * sp.x / sw) - 1.0f;
    float ny = 1.0f - (2.0f * sp.y / sh);
    glm::vec4 wp = glm::inverse(GetViewProjectionMatrix(sw, sh)) * glm::vec4(nx, ny, 0.0f, 1.0f);
    return {wp.x, wp.y};
}

Math::Vector2 Camera2D::WorldToScreen(Math::Vector2 wp, float sw, float sh) const
{
    glm::vec4 cp = GetViewProjectionMatrix(sw, sh) * glm::vec4(wp.x, wp.y, 0.0f, 1.0f);
    return {(cp.x + 1.0f) * 0.5f * sw, (1.0f - cp.y) * 0.5f * sh};
}

// =============================================================================
// Renderer2D::Impl
// =============================================================================

struct Renderer2D::Impl
{
    // -------------------------------------------------------------------------
    // Per-instance data layouts — tightly packed, GPU-friendly
    // -------------------------------------------------------------------------

    // 40 bytes — quad: world pivot, size, origin offset, rotation, rgb
    struct QuadInstance {
        float px, py;       // world pivot position
        float sx, sy;       // size (width, height)
        float ox, oy;       // origin offset
        float angle;        // rotation in radians
        float r, g, b;      // color
    };
    static_assert(sizeof(QuadInstance) == 40, "QuadInstance size mismatch");

    // 28 bytes — circle: center, radius, rgb, outlined flag
    struct CircleInstance {
        float cx, cy;       // world center
        float radius;
        float r, g, b;      // color
        float outlined;     // 0.0 = filled, 1.0 = outline ring
    };
    static_assert(sizeof(CircleInstance) == 28, "CircleInstance size mismatch");

    // 32 bytes — line: two endpoints, rgb, world-space thickness
    struct LineInstance {
        float x0, y0;       // start point
        float x1, y1;       // end point
        float r, g, b;      // color
        float thickness;    // world-space width
    };
    static_assert(sizeof(LineInstance) == 32, "LineInstance size mismatch");

    // 36 bytes — capsule: two centers, radius, rgb, outlined flag
    struct CapsuleInstance {
        float cx0, cy0;     // center1
        float cx1, cy1;     // center2
        float radius;
        float r, g, b;      // color
        float outlined;     // 0.0 = filled, 1.0 = outline
    };
    static_assert(sizeof(CapsuleInstance) == 36, "CapsuleInstance size mismatch");

    // -------------------------------------------------------------------------
    // Capacity limits
    // -------------------------------------------------------------------------
    static constexpr uint32_t MAX_QUADS      = 131072;   // 128K quads   ≈  5 MB
    static constexpr uint32_t MAX_CIRCLES    = 524288;   // 512K circles ≈ 14 MB
    static constexpr uint32_t MAX_LINES      = 262144;   // 256K lines   ≈  8 MB
    static constexpr uint32_t MAX_CAPSULES   = 32768;    //  32K capsules ≈  3 MB
    static constexpr uint32_t MAX_POLY_FILL  = 65536;    //  64K polygon fill verts
    static constexpr uint32_t MAX_POLY_LINE  = 65536;    //  64K polygon outline verts

    // -------------------------------------------------------------------------
    // Mesh segment counts
    // -------------------------------------------------------------------------
    static constexpr int CIRCLE_SEG  = 16;  // SDF rounds the 16-sided polygon perfectly
    static constexpr int CAPSULE_SEG = 8;   // per semicircle

    // -------------------------------------------------------------------------
    // Triple-buffering
    // -------------------------------------------------------------------------
    static constexpr int NUM_FRAMES = 3;
    int CurrentFrame = 0;

    // -------------------------------------------------------------------------
    // Quad pipeline
    // -------------------------------------------------------------------------
    GLuint QuadVAO      = 0;
    GLuint QuadMeshVBO  = 0;   // static unit quad mesh (6 verts, immutable)
    GLuint QuadInstVBO  = 0;   // instance data (persistently mapped, triple-buffered)
    GLuint QuadShader   = 0;
    GLint  QuadVP_Loc   = -1;
    QuadInstance* QuadInstBase  = nullptr;  // CPU pointer into persistent GPU memory
    uint32_t      QuadInstCount = 0;
    GLsync QuadFences[NUM_FRAMES] = {};

    // -------------------------------------------------------------------------
    // Circle pipeline
    // -------------------------------------------------------------------------
    GLuint CircleVAO      = 0;
    GLuint CircleMeshVBO  = 0;
    GLuint CircleInstVBO  = 0;
    GLuint CircleShader   = 0;
    GLint  CircleVP_Loc   = -1;
    CircleInstance* CircleInstBase  = nullptr;
    uint32_t        CircleInstCount = 0;
    GLsync CircleFences[NUM_FRAMES] = {};

    // -------------------------------------------------------------------------
    // Line pipeline
    // -------------------------------------------------------------------------
    GLuint LineVAO      = 0;
    GLuint LineMeshVBO  = 0;
    GLuint LineInstVBO  = 0;
    GLuint LineShader   = 0;
    GLint  LineVP_Loc   = -1;
    LineInstance* LineInstBase  = nullptr;
    uint32_t      LineInstCount = 0;
    GLsync LineFences[NUM_FRAMES] = {};

    // -------------------------------------------------------------------------
    // Capsule pipeline
    // -------------------------------------------------------------------------
    GLuint CapsuleVAO      = 0;
    GLuint CapsuleMeshVBO  = 0;
    GLuint CapsuleInstVBO  = 0;
    GLuint CapsuleShader   = 0;
    GLint  CapsuleVP_Loc   = -1;
    CapsuleInstance* CapsuleInstBase  = nullptr;
    uint32_t         CapsuleInstCount = 0;
    GLsync CapsuleFences[NUM_FRAMES] = {};
    int CapsuleMeshVertCount = 0;  // set after mesh is built

    // -------------------------------------------------------------------------
    // Polygon passthrough buffer
    // CPU tessellates world-space geometry directly into GPU-mapped memory.
    // No glBufferData, no intermediate std::vector — direct pointer write.
    // PolyFill → GL_TRIANGLES, PolyLine → GL_LINES
    // -------------------------------------------------------------------------
    GLuint PolyFillVAO = 0, PolyFillVBO = 0;
    GLuint PolyLineVAO = 0, PolyLineVBO = 0;
    GLuint PolyShader  = 0;
    GLint  PolyVP_Loc  = -1;
    Vertex* PolyFillBase  = nullptr;
    Vertex* PolyLineBase  = nullptr;
    uint32_t PolyFillCount = 0;
    uint32_t PolyLineCount = 0;
    GLsync PolyFillFences[NUM_FRAMES] = {};
    GLsync PolyLineFences[NUM_FRAMES] = {};

    // -------------------------------------------------------------------------
    // Camera and matrices
    // -------------------------------------------------------------------------
    Camera2D  CurrentCamera;
    glm::mat4 ViewMatrix       = glm::mat4(1.0f);
    glm::mat4 ProjectionMatrix = glm::mat4(1.0f);

    // -------------------------------------------------------------------------
    // Render state
    // -------------------------------------------------------------------------
    bool  Initialized       = false;
    bool  GLAvailable       = false;
    bool  BlendingEnabled   = true;
    bool  DepthTestEnabled  = false;
    bool  CullingEnabled    = false;
    float CurrentLineWidth  = 1.0f;

    // =========================================================================
    // GL helpers
    // =========================================================================

    bool CheckGLFunctionsLoaded()
    {
        // Core functions required for basic rendering
        bool core =
            glGenVertexArrays     != nullptr &&
            glDeleteVertexArrays  != nullptr &&
            glBindVertexArray     != nullptr &&
            glGenBuffers          != nullptr &&
            glDeleteBuffers       != nullptr &&
            glBindBuffer          != nullptr &&
            glBufferData          != nullptr &&
            glVertexAttribPointer != nullptr &&
            glEnableVertexAttribArray != nullptr &&
            glCreateShader        != nullptr &&
            glShaderSource        != nullptr &&
            glCompileShader       != nullptr &&
            glGetShaderiv         != nullptr &&
            glGetShaderInfoLog    != nullptr &&
            glDeleteShader        != nullptr &&
            glCreateProgram       != nullptr &&
            glAttachShader        != nullptr &&
            glLinkProgram         != nullptr &&
            glGetProgramiv        != nullptr &&
            glGetProgramInfoLog   != nullptr &&
            glValidateProgram     != nullptr &&
            glDeleteProgram       != nullptr &&
            glUseProgram          != nullptr &&
            glGetUniformLocation  != nullptr &&
            glUniformMatrix4fv    != nullptr &&
            glUniform1f           != nullptr &&
            glUniform1i           != nullptr &&
            glDrawArrays          != nullptr &&
            glEnable              != nullptr &&
            glDisable             != nullptr &&
            glBlendFunc           != nullptr;

        // GL 4.3+ instanced draw
        bool instanced =
            glDrawArraysInstanced != nullptr &&
            glVertexAttribDivisor != nullptr;

        // GL 4.4+ persistent buffer storage
        bool persistent =
            glBufferStorage   != nullptr &&
            glMapBufferRange  != nullptr &&
            glUnmapBuffer     != nullptr;

        // GL 3.2+ sync objects
        bool sync =
            glFenceSync        != nullptr &&
            glClientWaitSync   != nullptr &&
            glDeleteSync       != nullptr;

        if (!instanced)  printf("Renderer2D: GL 4.3 instanced draw functions missing\n");
        if (!persistent) printf("Renderer2D: GL 4.4 buffer storage functions missing\n");
        if (!sync)       printf("Renderer2D: GL 3.2 sync functions missing\n");

        return core && instanced && persistent && sync;
    }

    // -------------------------------------------------------------------------

    GLuint CompileShader(GLenum type, const char* source)
    {
        GLuint id = glCreateShader(type);
        if (id == 0) return 0;
        glShaderSource(id, 1, &source, nullptr);
        glCompileShader(id);
        GLint ok = GL_FALSE;
        glGetShaderiv(id, GL_COMPILE_STATUS, &ok);
        if (ok == GL_FALSE)
        {
            GLint len = 0;
            glGetShaderiv(id, GL_INFO_LOG_LENGTH, &len);
            if (len > 0)
            {
                auto* msg = static_cast<char*>(alloca(static_cast<size_t>(len)));
                glGetShaderInfoLog(id, len, &len, msg);
                printf("Renderer2D: shader compile error (%s):\n%s\n",
                       type == GL_VERTEX_SHADER ? "vert" : "frag", msg);
            }
            glDeleteShader(id);
            return 0;
        }
        return id;
    }

    // -------------------------------------------------------------------------

    GLuint CreateProgram(const char* vsSrc, const char* fsSrc)
    {
        GLuint vs = CompileShader(GL_VERTEX_SHADER,   vsSrc);
        GLuint fs = CompileShader(GL_FRAGMENT_SHADER, fsSrc);
        if (vs == 0 || fs == 0)
        {
            if (vs) glDeleteShader(vs);
            if (fs) glDeleteShader(fs);
            return 0;
        }
        GLuint prog = glCreateProgram();
        if (prog == 0) { glDeleteShader(vs); glDeleteShader(fs); return 0; }
        glAttachShader(prog, vs);
        glAttachShader(prog, fs);
        glLinkProgram(prog);
        GLint ok = GL_FALSE;
        glGetProgramiv(prog, GL_LINK_STATUS, &ok);
        if (ok == GL_FALSE)
        {
            GLint len = 0;
            glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &len);
            if (len > 0)
            {
                auto* msg = static_cast<char*>(alloca(static_cast<size_t>(len)));
                glGetProgramInfoLog(prog, len, &len, msg);
                printf("Renderer2D: shader link error:\n%s\n", msg);
            }
            glDeleteShader(vs); glDeleteShader(fs);
            glDeleteProgram(prog);
            return 0;
        }
        glValidateProgram(prog);
        glDeleteShader(vs);
        glDeleteShader(fs);
        return prog;
    }

    // -------------------------------------------------------------------------
    // AllocatePersistentBuffer
    //
    // Creates a VBO with glBufferStorage using MAP_WRITE | MAP_PERSISTENT |
    // MAP_COHERENT and returns the permanently-mapped CPU pointer.
    // Total size = singleFrameBytes * NUM_FRAMES (triple-buffered).
    // Caller must glBindBuffer(GL_ARRAY_BUFFER, vbo) before calling.
    // -------------------------------------------------------------------------
    void* AllocatePersistentBuffer(GLuint& vbo, GLsizeiptr singleFrameBytes)
    {
        const GLbitfield flags = GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT;
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferStorage(GL_ARRAY_BUFFER, singleFrameBytes * NUM_FRAMES, nullptr, flags);
        void* ptr = glMapBufferRange(GL_ARRAY_BUFFER, 0, singleFrameBytes * NUM_FRAMES, flags);
        if (!ptr)
            printf("Renderer2D: glMapBufferRange failed (size=%zu)\n",
                   static_cast<size_t>(singleFrameBytes * NUM_FRAMES));
        return ptr;
    }

    // -------------------------------------------------------------------------
    // Fence helpers
    // -------------------------------------------------------------------------

    // Wait on the fence for 'frame' then release it.
    // In practice returns immediately if the CPU is ≤ (NUM_FRAMES-1) frames ahead.
    void WaitFence(GLsync* fences, int frame)
    {
        if (!fences[frame]) return;
        GLenum r;
        do {
            // 1-second timeout; should never actually loop
            r = glClientWaitSync(fences[frame], GL_SYNC_FLUSH_COMMANDS_BIT, 1'000'000'000ULL);
        } while (r == GL_TIMEOUT_EXPIRED);
        glDeleteSync(fences[frame]);
        fences[frame] = nullptr;
    }

    // Place a fence after a draw call for 'frame'.
    void PlaceFence(GLsync* fences, int frame)
    {
        if (fences[frame]) glDeleteSync(fences[frame]);
        fences[frame] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    }

    // =========================================================================
    // Pipeline setup helpers
    // =========================================================================

    // Called during Init.  Loads shaders, builds meshes, allocates VBOs.
    void SetupQuadPipeline()
    {
        // --- Load shader ---
        std::string vsSrc = LoadShaderSource("quad.vert");
        std::string fsSrc = LoadShaderSource("quad.frag");
        QuadShader = CreateProgram(vsSrc.c_str(), fsSrc.c_str());
        if (!QuadShader) return;
        QuadVP_Loc = glGetUniformLocation(QuadShader, "u_VP");

        // --- Static unit quad mesh: XY in [0,1]x[0,1] ---
        // The vertex shader maps these to world space using per-instance
        // position, size, origin, and angle.
        // Two CCW triangles covering the full unit square.
        const float mesh[] = {
            0.0f, 0.0f,    1.0f, 0.0f,    1.0f, 1.0f,   // tri 0
            0.0f, 0.0f,    1.0f, 1.0f,    0.0f, 1.0f,   // tri 1
        };
        glGenBuffers(1, &QuadMeshVBO);
        glBindBuffer(GL_ARRAY_BUFFER, QuadMeshVBO);
        glBufferStorage(GL_ARRAY_BUFFER, sizeof(mesh), mesh, 0);  // immutable

        // --- Instance VBO (persistent mapped, triple-buffered) ---
        QuadInstBase = static_cast<QuadInstance*>(
            AllocatePersistentBuffer(QuadInstVBO, MAX_QUADS * sizeof(QuadInstance))
        );

        // --- VAO ---
        glGenVertexArrays(1, &QuadVAO);
        glBindVertexArray(QuadVAO);

        // Attrib 0: mesh local XY (per vertex, divisor 0)
        glBindBuffer(GL_ARRAY_BUFFER, QuadMeshVBO);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
        glEnableVertexAttribArray(0);

        // Attribs 1-5: per-instance data (divisor 1).
        // We only enable here; the actual pointer (with frame offset) is set in Flush.
        glBindBuffer(GL_ARRAY_BUFFER, QuadInstVBO);
        using QI = QuadInstance;
        const GLsizei s = sizeof(QI);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(QI, px));    // position
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(QI, sx));    // size
        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(QI, ox));    // origin
        glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, s, (void*)offsetof(QI, angle)); // angle
        glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, s, (void*)offsetof(QI, r));     // color
        glEnableVertexAttribArray(1); glVertexAttribDivisor(1, 1);
        glEnableVertexAttribArray(2); glVertexAttribDivisor(2, 1);
        glEnableVertexAttribArray(3); glVertexAttribDivisor(3, 1);
        glEnableVertexAttribArray(4); glVertexAttribDivisor(4, 1);
        glEnableVertexAttribArray(5); glVertexAttribDivisor(5, 1);

        glBindVertexArray(0);
    }

    // -------------------------------------------------------------------------

    void SetupCirclePipeline()
    {
        // --- Load shader ---
        std::string vsSrc = LoadShaderSource("circle.vert");
        std::string fsSrc = LoadShaderSource("circle.frag");
        CircleShader = CreateProgram(vsSrc.c_str(), fsSrc.c_str());
        if (!CircleShader) return;
        CircleVP_Loc = glGetUniformLocation(CircleShader, "u_VP");

        // --- Static unit circle mesh (triangle fan) ---
        // Each vertex is a 2D unit-circle point; VS scales by radius.
        // 16-sided polygon: fragment shader SDF rounds it perfectly.
        std::vector<float> mesh;
        mesh.reserve(CIRCLE_SEG * 3 * 2);
        const float step = 2.0f * PI / static_cast<float>(CIRCLE_SEG);
        for (int i = 0; i < CIRCLE_SEG; ++i)
        {
            float a0 = step * i;
            float a1 = step * (i + 1);
            mesh.push_back(0.0f); mesh.push_back(0.0f);              // center
            mesh.push_back(std::cos(a0)); mesh.push_back(std::sin(a0)); // p0
            mesh.push_back(std::cos(a1)); mesh.push_back(std::sin(a1)); // p1
        }

        glGenBuffers(1, &CircleMeshVBO);
        glBindBuffer(GL_ARRAY_BUFFER, CircleMeshVBO);
        glBufferStorage(GL_ARRAY_BUFFER,
                        static_cast<GLsizeiptr>(mesh.size() * sizeof(float)),
                        mesh.data(), 0);

        // --- Instance VBO ---
        CircleInstBase = static_cast<CircleInstance*>(
            AllocatePersistentBuffer(CircleInstVBO, MAX_CIRCLES * sizeof(CircleInstance))
        );

        // --- VAO ---
        glGenVertexArrays(1, &CircleVAO);
        glBindVertexArray(CircleVAO);

        glBindBuffer(GL_ARRAY_BUFFER, CircleMeshVBO);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, CircleInstVBO);
        using CI = CircleInstance;
        const GLsizei s = sizeof(CI);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CI, cx));       // center
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CI, radius));   // radius
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CI, r));        // color
        glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CI, outlined)); // mode
        glEnableVertexAttribArray(1); glVertexAttribDivisor(1, 1);
        glEnableVertexAttribArray(2); glVertexAttribDivisor(2, 1);
        glEnableVertexAttribArray(3); glVertexAttribDivisor(3, 1);
        glEnableVertexAttribArray(4); glVertexAttribDivisor(4, 1);

        glBindVertexArray(0);
    }

    // -------------------------------------------------------------------------

    void SetupLinePipeline()
    {
        // --- Load shader ---
        std::string vsSrc = LoadShaderSource("line.vert");
        std::string fsSrc = LoadShaderSource("line.frag");
        LineShader = CreateProgram(vsSrc.c_str(), fsSrc.c_str());
        if (!LineShader) return;
        LineVP_Loc = glGetUniformLocation(LineShader, "u_VP");

        // --- Static unit rectangle mesh ---
        // X ∈ [0,1]  →  parametric position along the line (p0 → p1)
        // Y ∈ [-0.5, +0.5]  →  lateral offset (scaled by thickness in VS)
        // Two CCW triangles.
        const float mesh[] = {
            0.0f, -0.5f,   1.0f, -0.5f,   1.0f,  0.5f,   // tri 0
            0.0f, -0.5f,   1.0f,  0.5f,   0.0f,  0.5f,   // tri 1
        };
        glGenBuffers(1, &LineMeshVBO);
        glBindBuffer(GL_ARRAY_BUFFER, LineMeshVBO);
        glBufferStorage(GL_ARRAY_BUFFER, sizeof(mesh), mesh, 0);

        // --- Instance VBO ---
        LineInstBase = static_cast<LineInstance*>(
            AllocatePersistentBuffer(LineInstVBO, MAX_LINES * sizeof(LineInstance))
        );

        // --- VAO ---
        glGenVertexArrays(1, &LineVAO);
        glBindVertexArray(LineVAO);

        glBindBuffer(GL_ARRAY_BUFFER, LineMeshVBO);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, LineInstVBO);
        using LI = LineInstance;
        const GLsizei s = sizeof(LI);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(LI, x0));        // start
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(LI, x1));        // end
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, s, (void*)offsetof(LI, r));         // color
        glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, s, (void*)offsetof(LI, thickness)); // thickness
        glEnableVertexAttribArray(1); glVertexAttribDivisor(1, 1);
        glEnableVertexAttribArray(2); glVertexAttribDivisor(2, 1);
        glEnableVertexAttribArray(3); glVertexAttribDivisor(3, 1);
        glEnableVertexAttribArray(4); glVertexAttribDivisor(4, 1);

        glBindVertexArray(0);
    }

    // -------------------------------------------------------------------------
    // SetupCapsulePipeline
    //
    // Builds a capsule mesh in normalized "capsule-local" space:
    //   vertex = (localX, localY, type)
    //   type 0.0 → left cap  relative to c1 in (axisDir, perpDir) frame × radius
    //   type 1.0 → right cap relative to c2 in (axisDir, perpDir) frame × radius
    //   type 2.0 → body strip: lerp(c1,c2,localX) + perpDir × (localY × radius)
    //
    // The capsule vertex shader reconstructs world positions from this encoding.
    // -------------------------------------------------------------------------
    void SetupCapsulePipeline()
    {
        // --- Load shader ---
        std::string vsSrc = LoadShaderSource("capsule.vert");
        std::string fsSrc = LoadShaderSource("capsule.frag");
        CapsuleShader = CreateProgram(vsSrc.c_str(), fsSrc.c_str());
        if (!CapsuleShader) return;
        CapsuleVP_Loc = glGetUniformLocation(CapsuleShader, "u_VP");

        // --- Build capsule mesh ---
        // Each element: (localX, localY, type) — 3 floats per vertex
        struct CMVert { float lx, ly, type; };
        std::vector<CMVert> mesh;

        const float halfStep = PI / static_cast<float>(CAPSULE_SEG);

        // Left cap (type=0): semicircle from PI/2 to 3*PI/2
        // Angles wrap the left side of the capsule (pointing away from c2).
        for (int i = 0; i < CAPSULE_SEG; ++i)
        {
            float a0 = PI / 2.0f + halfStep * i;
            float a1 = PI / 2.0f + halfStep * (i + 1);
            mesh.push_back({0.0f,                0.0f,               0.0f});
            mesh.push_back({std::cos(a0),        std::sin(a0),       0.0f});
            mesh.push_back({std::cos(a1),        std::sin(a1),       0.0f});
        }

        // Right cap (type=1): semicircle from -PI/2 to PI/2
        for (int i = 0; i < CAPSULE_SEG; ++i)
        {
            float a0 = -PI / 2.0f + halfStep * i;
            float a1 = -PI / 2.0f + halfStep * (i + 1);
            mesh.push_back({0.0f,                0.0f,               1.0f});
            mesh.push_back({std::cos(a0),        std::sin(a0),       1.0f});
            mesh.push_back({std::cos(a1),        std::sin(a1),       1.0f});
        }

        // Body rectangle (type=2): u ∈ [0,1] along c1→c2, v ∈ [-1,+1] lateral
        // Two CCW triangles
        mesh.push_back({0.0f, -1.0f, 2.0f}); mesh.push_back({1.0f, -1.0f, 2.0f}); mesh.push_back({1.0f,  1.0f, 2.0f});
        mesh.push_back({0.0f, -1.0f, 2.0f}); mesh.push_back({1.0f,  1.0f, 2.0f}); mesh.push_back({0.0f,  1.0f, 2.0f});

        CapsuleMeshVertCount = static_cast<int>(mesh.size());

        glGenBuffers(1, &CapsuleMeshVBO);
        glBindBuffer(GL_ARRAY_BUFFER, CapsuleMeshVBO);
        glBufferStorage(GL_ARRAY_BUFFER,
                        static_cast<GLsizeiptr>(mesh.size() * sizeof(CMVert)),
                        mesh.data(), 0);

        // --- Instance VBO ---
        CapsuleInstBase = static_cast<CapsuleInstance*>(
            AllocatePersistentBuffer(CapsuleInstVBO, MAX_CAPSULES * sizeof(CapsuleInstance))
        );

        // --- VAO ---
        glGenVertexArrays(1, &CapsuleVAO);
        glBindVertexArray(CapsuleVAO);

        // Mesh: (localX, localY, type) — 3 floats, stride 12 bytes
        glBindBuffer(GL_ARRAY_BUFFER, CapsuleMeshVBO);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, CapsuleInstVBO);
        using CAI = CapsuleInstance;
        const GLsizei s = sizeof(CAI);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CAI, cx0));      // c1
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CAI, cx1));      // c2
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CAI, radius));   // radius
        glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CAI, r));        // color
        glVertexAttribPointer(5, 1, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CAI, outlined)); // mode
        glEnableVertexAttribArray(1); glVertexAttribDivisor(1, 1);
        glEnableVertexAttribArray(2); glVertexAttribDivisor(2, 1);
        glEnableVertexAttribArray(3); glVertexAttribDivisor(3, 1);
        glEnableVertexAttribArray(4); glVertexAttribDivisor(4, 1);
        glEnableVertexAttribArray(5); glVertexAttribDivisor(5, 1);

        glBindVertexArray(0);
    }

    // -------------------------------------------------------------------------
    // SetupPolygonPipeline
    //
    // Two persistent-mapped VBOs: one for filled triangles (GL_TRIANGLES),
    // one for outline segments (GL_LINES).  CPU writes world-space Vertex
    // structs directly into GPU memory — no glBufferData, no copies.
    // Both VAOs share the same polygon shader (passthrough pos + color).
    // -------------------------------------------------------------------------
    void SetupPolygonPipeline()
    {
        // --- Load shader ---
        std::string vsSrc = LoadShaderSource("polygon.vert");
        std::string fsSrc = LoadShaderSource("polygon.frag");
        PolyShader = CreateProgram(vsSrc.c_str(), fsSrc.c_str());
        if (!PolyShader) return;
        PolyVP_Loc = glGetUniformLocation(PolyShader, "u_VP");

        // --- Fill VBO ---
        PolyFillBase = static_cast<Vertex*>(
            AllocatePersistentBuffer(PolyFillVBO, MAX_POLY_FILL * sizeof(Vertex))
        );

        glGenVertexArrays(1, &PolyFillVAO);
        glBindVertexArray(PolyFillVAO);
        glBindBuffer(GL_ARRAY_BUFFER, PolyFillVBO);
        // Attrib 0: position (x, y)
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<void*>(offsetof(Vertex, x)));
        glEnableVertexAttribArray(0);
        // Attrib 1: color (r, g, b)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<void*>(offsetof(Vertex, r)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);

        // --- Line VBO ---
        PolyLineBase = static_cast<Vertex*>(
            AllocatePersistentBuffer(PolyLineVBO, MAX_POLY_LINE * sizeof(Vertex))
        );

        glGenVertexArrays(1, &PolyLineVAO);
        glBindVertexArray(PolyLineVAO);
        glBindBuffer(GL_ARRAY_BUFFER, PolyLineVBO);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<void*>(offsetof(Vertex, x)));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<void*>(offsetof(Vertex, r)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
    }

    // =========================================================================
    // Camera matrix updates
    // =========================================================================

    void UpdateProjectionMatrix(float w, float h)
    {
        const float left   = 0.0f;
        const float right  = w / CurrentCamera.zoom;
        const float bottom = 0.0f;
        const float top    = h / CurrentCamera.zoom;
        ProjectionMatrix = glm::ortho(left, right, bottom, top,
                                      CurrentCamera.nearPlane, CurrentCamera.farPlane);
    }

    void UpdateViewMatrix()
    {
        ViewMatrix = CurrentCamera.GetViewMatrix();
    }

    // =========================================================================
    // Inline write helpers — write one instance into the current frame's slot
    // =========================================================================

    // Write one QuadInstance; returns false if the buffer is full (should not
    // happen in normal use — raise MAX_QUADS if needed).
    inline bool PushQuad(float px, float py, float sx, float sy,
                         float ox, float oy, float angle,
                         float r,  float g,  float b)
    {
        if (QuadInstCount >= MAX_QUADS) return false;
        QuadInstance& inst = QuadInstBase[CurrentFrame * MAX_QUADS + QuadInstCount++];
        inst.px = px; inst.py = py;
        inst.sx = sx; inst.sy = sy;
        inst.ox = ox; inst.oy = oy;
        inst.angle = angle;
        inst.r = r; inst.g = g; inst.b = b;
        return true;
    }

    inline bool PushCircle(float cx, float cy, float radius,
                           float r, float g, float b, float outlined)
    {
        if (CircleInstCount >= MAX_CIRCLES) return false;
        CircleInstance& inst = CircleInstBase[CurrentFrame * MAX_CIRCLES + CircleInstCount++];
        inst.cx = cx; inst.cy = cy;
        inst.radius = radius;
        inst.r = r; inst.g = g; inst.b = b;
        inst.outlined = outlined;
        return true;
    }

    inline bool PushLine(float x0, float y0, float x1, float y1,
                         float r, float g, float b, float thickness)
    {
        if (LineInstCount >= MAX_LINES) return false;
        LineInstance& inst = LineInstBase[CurrentFrame * MAX_LINES + LineInstCount++];
        inst.x0 = x0; inst.y0 = y0;
        inst.x1 = x1; inst.y1 = y1;
        inst.r = r; inst.g = g; inst.b = b;
        inst.thickness = thickness;
        return true;
    }

    inline bool PushCapsule(float cx0, float cy0, float cx1, float cy1,
                            float radius, float r, float g, float b, float outlined)
    {
        if (CapsuleInstCount >= MAX_CAPSULES) return false;
        CapsuleInstance& inst = CapsuleInstBase[CurrentFrame * MAX_CAPSULES + CapsuleInstCount++];
        inst.cx0 = cx0; inst.cy0 = cy0;
        inst.cx1 = cx1; inst.cy1 = cy1;
        inst.radius = radius;
        inst.r = r; inst.g = g; inst.b = b;
        inst.outlined = outlined;
        return true;
    }

    // Write world-space Vertex directly into PolyFill or PolyLine persistent buffer
    inline bool PushPolyFillVert(float x, float y, float r, float g, float b)
    {
        if (PolyFillCount >= MAX_POLY_FILL) return false;
        Vertex& v = PolyFillBase[CurrentFrame * MAX_POLY_FILL + PolyFillCount++];
        v.x = x; v.y = y;
        v.r = r; v.g = g; v.b = b;
        v.u = 0; v.v = 0; v.nx = 0; v.ny = 0;
        return true;
    }

    inline bool PushPolyLineVert(float x, float y, float r, float g, float b)
    {
        if (PolyLineCount >= MAX_POLY_LINE) return false;
        Vertex& v = PolyLineBase[CurrentFrame * MAX_POLY_LINE + PolyLineCount++];
        v.x = x; v.y = y;
        v.r = r; v.g = g; v.b = b;
        v.u = 0; v.v = 0; v.nx = 0; v.ny = 0;
        return true;
    }
};

// =============================================================================
// Static instance
// =============================================================================

std::unique_ptr<Renderer2D::Impl> Renderer2D::s_Instance;

// =============================================================================
// Init
// =============================================================================

void Renderer2D::Init()
{
    s_Instance = std::make_unique<Impl>();
    s_Instance->GLAvailable = s_Instance->CheckGLFunctionsLoaded();

    if (!s_Instance->GLAvailable)
    {
        printf("Renderer2D::Init — required GL functions not available. "
               "Rendering disabled.\n");
        s_Instance->Initialized = true;
        return;
    }

    // Each Setup*Pipeline call loads its shader, builds its static mesh,
    // and allocates its triple-buffered persistent instance VBO.
    // A failure in one pipeline (e.g. missing shader file) prints an error
    // but does not prevent the others from initialising.
    s_Instance->SetupQuadPipeline();
    s_Instance->SetupCirclePipeline();
    s_Instance->SetupLinePipeline();
    s_Instance->SetupCapsulePipeline();
    s_Instance->SetupPolygonPipeline();

    s_Instance->UpdateProjectionMatrix(1280.0f, 720.0f);
    s_Instance->Initialized = true;

    // Debug: Print GPU info
    printf("Renderer2D: Initialized successfully. GPU: %s, OpenGL: %s\n",
           glGetString(GL_RENDERER), glGetString(GL_VERSION));
}

// =============================================================================
// Shutdown
// =============================================================================

void Renderer2D::Shutdown()
{
    if (!s_Instance) return;

    if (s_Instance->GLAvailable)
    {
        // Wait for all in-flight GPU work before unmapping/deleting
        for (int f = 0; f < Impl::NUM_FRAMES; ++f)
        {
            s_Instance->WaitFence(s_Instance->QuadFences,      f);
            s_Instance->WaitFence(s_Instance->CircleFences,    f);
            s_Instance->WaitFence(s_Instance->LineFences,      f);
            s_Instance->WaitFence(s_Instance->CapsuleFences,   f);
            s_Instance->WaitFence(s_Instance->PolyFillFences,  f);
            s_Instance->WaitFence(s_Instance->PolyLineFences,  f);
        }

        // Unmap all persistent buffers before deleting
        auto unmap = [](GLuint vbo) {
            if (vbo) { glBindBuffer(GL_ARRAY_BUFFER, vbo); glUnmapBuffer(GL_ARRAY_BUFFER); }
        };
        unmap(s_Instance->QuadInstVBO);
        unmap(s_Instance->CircleInstVBO);
        unmap(s_Instance->LineInstVBO);
        unmap(s_Instance->CapsuleInstVBO);
        unmap(s_Instance->PolyFillVBO);
        unmap(s_Instance->PolyLineVBO);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        // Delete VBOs
        GLuint vbos[] = {
            s_Instance->QuadMeshVBO,    s_Instance->QuadInstVBO,
            s_Instance->CircleMeshVBO,  s_Instance->CircleInstVBO,
            s_Instance->LineMeshVBO,    s_Instance->LineInstVBO,
            s_Instance->CapsuleMeshVBO, s_Instance->CapsuleInstVBO,
            s_Instance->PolyFillVBO,    s_Instance->PolyLineVBO
        };
        glDeleteBuffers(10, vbos);

        // Delete VAOs
        GLuint vaos[] = {
            s_Instance->QuadVAO,
            s_Instance->CircleVAO,
            s_Instance->LineVAO,
            s_Instance->CapsuleVAO,
            s_Instance->PolyFillVAO,
            s_Instance->PolyLineVAO
        };
        glDeleteVertexArrays(6, vaos);

        // Delete shader programs
        GLuint progs[] = {
            s_Instance->QuadShader,
            s_Instance->CircleShader,
            s_Instance->LineShader,
            s_Instance->CapsuleShader,
            s_Instance->PolyShader
        };
        for (GLuint p : progs) if (p) glDeleteProgram(p);
    }

    s_Instance->Initialized = false;
    s_Instance.reset();
}

// =============================================================================
// BeginScene
//
// Waits on all fences for the INCOMING frame slot (the one we're about to
// write into), then resets all instance counts and updates the camera.
// =============================================================================

void Renderer2D::BeginScene(const Camera2D& camera)
{
    if (!s_Instance || !s_Instance->Initialized) return;

    s_Instance->CurrentCamera = camera;

    if (!s_Instance->GLAvailable) return;

    // Wait on the slot we are about to fill in.
    // In steady state (CPU ≤ 2 frames ahead of GPU) this returns immediately.
    const int f = s_Instance->CurrentFrame;
    s_Instance->WaitFence(s_Instance->QuadFences,      f);
    s_Instance->WaitFence(s_Instance->CircleFences,    f);
    s_Instance->WaitFence(s_Instance->LineFences,      f);
    s_Instance->WaitFence(s_Instance->CapsuleFences,   f);
    s_Instance->WaitFence(s_Instance->PolyFillFences,  f);
    s_Instance->WaitFence(s_Instance->PolyLineFences,  f);

    // Reset write heads for this frame slot
    s_Instance->QuadInstCount    = 0;
    s_Instance->CircleInstCount  = 0;
    s_Instance->LineInstCount    = 0;
    s_Instance->CapsuleInstCount = 0;
    s_Instance->PolyFillCount    = 0;
    s_Instance->PolyLineCount    = 0;

    // Update camera matrices
    GLFWwindow* window = nullptr;
    try { window = Application::Get().GetWindow(); } catch (...) {}
    int w = 1280, h = 720;
    if (window) glfwGetFramebufferSize(window, &w, &h);
    if (w > 0 && h > 0) s_Instance->UpdateProjectionMatrix(float(w), float(h));
    s_Instance->UpdateViewMatrix();

    // Blending / depth / cull state
    if (s_Instance->BlendingEnabled) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    } else glDisable(GL_BLEND);
    if (s_Instance->DepthTestEnabled) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
    if (s_Instance->CullingEnabled) { glEnable(GL_CULL_FACE); glCullFace(GL_BACK); }
    else glDisable(GL_CULL_FACE);
}

void Renderer2D::EndScene() { Flush(); }

const Camera2D& Renderer2D::GetActiveCamera()
{
    if (!s_Instance) {
        static Camera2D defaultCamera;
        return defaultCamera;
    }
    return s_Instance->CurrentCamera;
}

void Renderer2D::SetScreenDimensions(float width, float height)
{
    if (!s_Instance || !s_Instance->Initialized) return;
    
    if (width > 0 && height > 0) {
        s_Instance->UpdateProjectionMatrix(width, height);
    }
}

// =============================================================================
// Flush
//
// Issues one glDrawArraysInstanced per active shape type using the current
// frame's region of each persistent buffer.  Places a GLsync fence after each
// draw so the next frame's BeginScene can safely wait before writing.
// Advances CurrentFrame at the end.
// =============================================================================

void Renderer2D::Flush()
{
    if (!s_Instance || !s_Instance->Initialized || !s_Instance->GLAvailable) return;

    auto& I = *s_Instance;
    const int f = I.CurrentFrame;

    // VP matrix used by all pipelines this frame
    const glm::mat4 vp = I.ProjectionMatrix * I.ViewMatrix;

    // Macro to re-point an instanced attrib to the current frame's region.
    // Must be called with the correct VAO and instance VBO already bound.
    // The divisor is already set from Setup; we just update the byte offset.
#define REPOINT_INST_ATTRIB(idx, comps, stride, baseOff, memberOff)                    \
    glVertexAttribPointer((idx), (comps), GL_FLOAT, GL_FALSE, (stride),                \
        reinterpret_cast<const void*>((baseOff) + static_cast<GLintptr>(memberOff)))

    // -----------------------------------------------------------------
    // 1. Quads
    // -----------------------------------------------------------------
    if (I.QuadInstCount > 0 && I.QuadShader)
    {
        glUseProgram(I.QuadShader);
        glUniformMatrix4fv(I.QuadVP_Loc, 1, GL_FALSE, &vp[0][0]);
        glBindVertexArray(I.QuadVAO);

        // Update the instance attribute pointers to this frame's buffer slot.
        const GLintptr base = static_cast<GLintptr>(f * I.MAX_QUADS * sizeof(Impl::QuadInstance));
        const GLsizei  s    = sizeof(Impl::QuadInstance);
        glBindBuffer(GL_ARRAY_BUFFER, I.QuadInstVBO);
        REPOINT_INST_ATTRIB(1, 2, s, base, offsetof(Impl::QuadInstance, px));
        REPOINT_INST_ATTRIB(2, 2, s, base, offsetof(Impl::QuadInstance, sx));
        REPOINT_INST_ATTRIB(3, 2, s, base, offsetof(Impl::QuadInstance, ox));
        REPOINT_INST_ATTRIB(4, 1, s, base, offsetof(Impl::QuadInstance, angle));
        REPOINT_INST_ATTRIB(5, 3, s, base, offsetof(Impl::QuadInstance, r));

        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, static_cast<GLsizei>(I.QuadInstCount));
        I.PlaceFence(I.QuadFences, f);
        glBindVertexArray(0);
    }

    // -----------------------------------------------------------------
    // 2. Circles (filled and outlined share the same draw call;
    //    the fragment shader branches on the 'outlined' per-instance flag)
    // -----------------------------------------------------------------
    if (I.CircleInstCount > 0 && I.CircleShader)
    {
        glUseProgram(I.CircleShader);
        glUniformMatrix4fv(I.CircleVP_Loc, 1, GL_FALSE, &vp[0][0]);
        glBindVertexArray(I.CircleVAO);

        const GLintptr base = static_cast<GLintptr>(f * I.MAX_CIRCLES * sizeof(Impl::CircleInstance));
        const GLsizei  s    = sizeof(Impl::CircleInstance);
        glBindBuffer(GL_ARRAY_BUFFER, I.CircleInstVBO);
        REPOINT_INST_ATTRIB(1, 2, s, base, offsetof(Impl::CircleInstance, cx));
        REPOINT_INST_ATTRIB(2, 1, s, base, offsetof(Impl::CircleInstance, radius));
        REPOINT_INST_ATTRIB(3, 3, s, base, offsetof(Impl::CircleInstance, r));
        REPOINT_INST_ATTRIB(4, 1, s, base, offsetof(Impl::CircleInstance, outlined));

        glDrawArraysInstanced(GL_TRIANGLES, 0,
                              Impl::CIRCLE_SEG * 3,
                              static_cast<GLsizei>(I.CircleInstCount));
        I.PlaceFence(I.CircleFences, f);
        glBindVertexArray(0);
    }

    // -----------------------------------------------------------------
    // 3. Lines
    // -----------------------------------------------------------------
    if (I.LineInstCount > 0 && I.LineShader)
    {
        glUseProgram(I.LineShader);
        glUniformMatrix4fv(I.LineVP_Loc, 1, GL_FALSE, &vp[0][0]);
        glBindVertexArray(I.LineVAO);

        const GLintptr base = static_cast<GLintptr>(f * I.MAX_LINES * sizeof(Impl::LineInstance));
        const GLsizei  s    = sizeof(Impl::LineInstance);
        glBindBuffer(GL_ARRAY_BUFFER, I.LineInstVBO);
        REPOINT_INST_ATTRIB(1, 2, s, base, offsetof(Impl::LineInstance, x0));
        REPOINT_INST_ATTRIB(2, 2, s, base, offsetof(Impl::LineInstance, x1));
        REPOINT_INST_ATTRIB(3, 3, s, base, offsetof(Impl::LineInstance, r));
        REPOINT_INST_ATTRIB(4, 1, s, base, offsetof(Impl::LineInstance, thickness));

        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, static_cast<GLsizei>(I.LineInstCount));
        I.PlaceFence(I.LineFences, f);
        glBindVertexArray(0);
    }

    // -----------------------------------------------------------------
    // 4. Capsules
    // -----------------------------------------------------------------
    if (I.CapsuleInstCount > 0 && I.CapsuleShader && I.CapsuleMeshVertCount > 0)
    {
        glUseProgram(I.CapsuleShader);
        glUniformMatrix4fv(I.CapsuleVP_Loc, 1, GL_FALSE, &vp[0][0]);
        glBindVertexArray(I.CapsuleVAO);

        const GLintptr base = static_cast<GLintptr>(f * I.MAX_CAPSULES * sizeof(Impl::CapsuleInstance));
        const GLsizei  s    = sizeof(Impl::CapsuleInstance);
        glBindBuffer(GL_ARRAY_BUFFER, I.CapsuleInstVBO);
        REPOINT_INST_ATTRIB(1, 2, s, base, offsetof(Impl::CapsuleInstance, cx0));
        REPOINT_INST_ATTRIB(2, 2, s, base, offsetof(Impl::CapsuleInstance, cx1));
        REPOINT_INST_ATTRIB(3, 1, s, base, offsetof(Impl::CapsuleInstance, radius));
        REPOINT_INST_ATTRIB(4, 3, s, base, offsetof(Impl::CapsuleInstance, r));
        REPOINT_INST_ATTRIB(5, 1, s, base, offsetof(Impl::CapsuleInstance, outlined));

        glDrawArraysInstanced(GL_TRIANGLES, 0,
                              I.CapsuleMeshVertCount,
                              static_cast<GLsizei>(I.CapsuleInstCount));
        I.PlaceFence(I.CapsuleFences, f);
        glBindVertexArray(0);
    }

    // -----------------------------------------------------------------
    // 5. Polygon fills (CPU tessellated, no glBufferData needed)
    // -----------------------------------------------------------------
    if (I.PolyFillCount > 0 && I.PolyShader)
    {
        glUseProgram(I.PolyShader);
        glUniformMatrix4fv(I.PolyVP_Loc, 1, GL_FALSE, &vp[0][0]);
        glBindVertexArray(I.PolyFillVAO);

        const GLintptr base = static_cast<GLintptr>(f * I.MAX_POLY_FILL * sizeof(Vertex));
        const GLsizei  s    = sizeof(Vertex);
        glBindBuffer(GL_ARRAY_BUFFER, I.PolyFillVBO);
        // Re-point to this frame's region (no buffer re-upload required)
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, s,
            reinterpret_cast<const void*>(base + static_cast<GLintptr>(offsetof(Vertex, x))));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, s,
            reinterpret_cast<const void*>(base + static_cast<GLintptr>(offsetof(Vertex, r))));

        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(I.PolyFillCount));
        I.PlaceFence(I.PolyFillFences, f);
        glBindVertexArray(0);
    }

    // -----------------------------------------------------------------
    // 6. Polygon outlines (CPU tessellated line pairs)
    // -----------------------------------------------------------------
    if (I.PolyLineCount > 0 && I.PolyShader)
    {
        glUseProgram(I.PolyShader);
        glUniformMatrix4fv(I.PolyVP_Loc, 1, GL_FALSE, &vp[0][0]);
        glBindVertexArray(I.PolyLineVAO);

        const GLintptr base = static_cast<GLintptr>(f * I.MAX_POLY_LINE * sizeof(Vertex));
        const GLsizei  s    = sizeof(Vertex);
        glBindBuffer(GL_ARRAY_BUFFER, I.PolyLineVBO);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, s,
            reinterpret_cast<const void*>(base + static_cast<GLintptr>(offsetof(Vertex, x))));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, s,
            reinterpret_cast<const void*>(base + static_cast<GLintptr>(offsetof(Vertex, r))));

        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(I.PolyLineCount));
        I.PlaceFence(I.PolyLineFences, f);
        glBindVertexArray(0);
    }

#undef REPOINT_INST_ATTRIB

    // Advance to the next triple-buffer slot
    s_Instance->CurrentFrame = (s_Instance->CurrentFrame + 1) % Impl::NUM_FRAMES;
}

// =============================================================================
// DrawQuad
//
// Old: computed 4 rotated corners + 6 Vertex push_backs on CPU.
// New: 10 float writes into a persistent GPU pointer.  The vertex shader
// performs the same computation for all instances in parallel on the GPU.
// =============================================================================

void Renderer2D::DrawQuad(const Math::Vector2& position,
                          const Math::Vector2& size,
                          const Math::Vector2& origin,
                          const Math::Vector3& color,
                          float rotation)
{
    if (!s_Instance || !s_Instance->Initialized) return;
    s_Instance->PushQuad(position.x, position.y,
                         size.x,     size.y,
                         origin.x,   origin.y,
                         rotation,
                         color.x, color.y, color.z);
}

// =============================================================================
// DrawSolidCircle / DrawCircle
//
// Old: N×cos/sin + N×3 Vertex writes on CPU.
// New: 7 float writes.  The vertex shader scales the unit mesh; the fragment
// shader SDF rounds the 16-sided polygon into a mathematically perfect circle.
// The 'segments' parameter is accepted but ignored — the SDF makes it moot.
// =============================================================================

void Renderer2D::DrawSolidCircle(const Math::Vector2& center, float radius,
                                 const Math::Vector3& color, int /*segments*/)
{
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    s_Instance->PushCircle(center.x, center.y, radius,
                           color.x, color.y, color.z, 0.0f);
}

void Renderer2D::DrawCircle(const Math::Vector2& center, float radius,
                            const Math::Vector3& color, int /*segments*/)
{
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    s_Instance->PushCircle(center.x, center.y, radius,
                           color.x, color.y, color.z, 1.0f);  // outlined
}

// =============================================================================
// DrawLine
//
// Old: split into two completely different code paths (thick → DrawSolidPolygon;
// thin → LineBuffer push_back).
// New: a single LineInstance write for all thicknesses.  The line vertex shader
// builds the rectangle from endpoints + thickness at VS time.
// =============================================================================

void Renderer2D::DrawLine(const Math::Vector2& start, const Math::Vector2& end,
                          const Math::Vector3& color, float thickness)
{
    if (!s_Instance || !s_Instance->Initialized) return;
    const float t = (thickness < 1.0f) ? 1.0f : thickness;
    s_Instance->PushLine(start.x, start.y, end.x, end.y,
                         color.x, color.y, color.z, t);
}

// =============================================================================
// DrawSolidCapsule / DrawCapsule
//
// Solid → instanced capsule pipeline (7 float writes).
// Outline → CPU tessellation into the PolyLine persistent buffer
//   (capsule outlines are used only by the debug renderer — low frequency,
//    low count, not worth a dedicated outline pipeline).
// =============================================================================

void Renderer2D::DrawSolidCapsule(const Math::Vector2& c1, const Math::Vector2& c2,
                                  float radius, const Math::Vector3& color, int /*seg*/)
{
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    if ((c2 - c1).Length() < 0.0001f)
    {
        DrawSolidCircle(c1, radius, color);
        return;
    }
    s_Instance->PushCapsule(c1.x, c1.y, c2.x, c2.y, radius,
                            color.x, color.y, color.z, 0.0f);
}

void Renderer2D::DrawCapsule(const Math::Vector2& c1, const Math::Vector2& c2,
                             float radius, const Math::Vector3& color, int segments)
{
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    if (segments < 8) segments = 8;

    Math::Vector2 axis  = c2 - c1;
    float len = axis.Length();
    if (len < 0.0001f) { DrawCircle(c1, radius, color, segments); return; }

    Math::Vector2 nor = axis * (1.0f / len);
    Math::Vector2 tan = {-nor.y, nor.x};
    const float cr = color.x, cg = color.y, cb = color.z;
    const float step = PI / static_cast<float>(segments);

    // Left semicircle → PolyLine buffer as GL_LINES pairs
    for (int i = 0; i < segments; ++i)
    {
        float a0 = PI / 2.0f + step * i;
        float a1 = PI / 2.0f + step * (i + 1);
        Math::Vector2 p0 = c1 + (tan * std::cos(a0) - nor * std::sin(a0)) * radius;
        Math::Vector2 p1 = c1 + (tan * std::cos(a1) - nor * std::sin(a1)) * radius;
        s_Instance->PushPolyLineVert(p0.x, p0.y, cr, cg, cb);
        s_Instance->PushPolyLineVert(p1.x, p1.y, cr, cg, cb);
    }

    // Right semicircle
    for (int i = 0; i < segments; ++i)
    {
        float a0 = -PI / 2.0f + step * i;
        float a1 = -PI / 2.0f + step * (i + 1);
        Math::Vector2 p0 = c2 + (tan * std::cos(a0) + nor * std::sin(a0)) * radius;
        Math::Vector2 p1 = c2 + (tan * std::cos(a1) + nor * std::sin(a1)) * radius;
        s_Instance->PushPolyLineVert(p0.x, p0.y, cr, cg, cb);
        s_Instance->PushPolyLineVert(p1.x, p1.y, cr, cg, cb);
    }

    // Two connecting side lines via the line instance pipeline
    DrawLine(c1 + tan * radius, c2 + tan * radius, color, 1.0f);
    DrawLine(c1 - tan * radius, c2 - tan * radius, color, 1.0f);
}

// =============================================================================
// DrawSolidPolygon
//
// CPU fan tessellation — arbitrary vertex counts cannot use a fixed mesh.
// Writes directly into the PolyFill persistent buffer (no glBufferData).
// =============================================================================

void Renderer2D::DrawSolidPolygon(const std::vector<Math::Vector2>& vertices,
                                  const Math::Vector3& color)
{
    if (!s_Instance || !s_Instance->Initialized || vertices.size() < 3) return;

#ifndef NDEBUG
    auto isConvex = [](const std::vector<Math::Vector2>& v) -> bool {
        int sign = 0;
        for (size_t i = 0; i < v.size(); ++i)
        {
            const auto& p1 = v[i];
            const auto& p2 = v[(i + 1) % v.size()];
            const auto& p3 = v[(i + 2) % v.size()];
            float cross = (p2.x - p1.x) * (p3.y - p2.y) - (p2.y - p1.y) * (p3.x - p2.x);
            if (cross != 0.0f) {
                int s = (cross > 0.0f) ? 1 : -1;
                if (sign == 0) sign = s;
                else if (s != sign) return false;
            }
        }
        return true;
    };
    assert(isConvex(vertices) && "DrawSolidPolygon requires a convex polygon");
#endif

    const float cr = color.x, cg = color.y, cb = color.z;
    const Math::Vector2& v0 = vertices[0];
    for (size_t i = 1; i + 1 < vertices.size(); ++i)
    {
        const Math::Vector2& v1 = vertices[i];
        const Math::Vector2& v2 = vertices[i + 1];
        s_Instance->PushPolyFillVert(v0.x, v0.y, cr, cg, cb);
        s_Instance->PushPolyFillVert(v1.x, v1.y, cr, cg, cb);
        s_Instance->PushPolyFillVert(v2.x, v2.y, cr, cg, cb);
    }
}

// =============================================================================
// DrawPolygon (outline)
//
// Routes each edge through the Line instance pipeline — one LineInstance per
// edge, no CPU vertex generation needed.
// =============================================================================

void Renderer2D::DrawPolygon(const std::vector<Math::Vector2>& vertices,
                             const Math::Vector3& color)
{
    if (!s_Instance || !s_Instance->Initialized || vertices.size() < 2) return;
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        const Math::Vector2& a = vertices[i];
        const Math::Vector2& b = vertices[(i + 1) % vertices.size()];
        s_Instance->PushLine(a.x, a.y, b.x, b.y,
                             color.x, color.y, color.z, 1.0f);
    }
}

// =============================================================================
// DrawSegment
//
// Thick: rectangle body (DrawSolidPolygon → PolyFill) + two end circles
//        (DrawSolidCircle → Circle instances)
// Thin:  single line instance
// =============================================================================

void Renderer2D::DrawSegment(const Math::Vector2& p1, const Math::Vector2& p2,
                             float thickness, const Math::Vector3& color)
{
    if (!s_Instance || !s_Instance->Initialized) return;
    if (thickness <= 0.0f) { DrawLine(p1, p2, color, 1.0f); return; }

    Math::Vector2 dir = p2 - p1;
    float len = dir.Length();
    if (len < 0.0001f) { DrawSolidCircle(p1, thickness * 0.5f, color, 16); return; }

    Math::Vector2 n = dir * (1.0f / len);
    Math::Vector2 t = {-n.y, n.x};
    float ht = thickness * 0.5f;

    std::vector<Math::Vector2> rect = {
        p1 + t * ht,   p1 - t * ht,
        p2 + t * ht,   p2 - t * ht
    };
    DrawSolidPolygon(rect, color);
    DrawSolidCircle(p1, ht, color, 16);
    DrawSolidCircle(p2, ht, color, 16);
}

// =============================================================================
// DrawChain — iterates and calls DrawSegment
// =============================================================================

void Renderer2D::DrawChain(const std::vector<Math::Vector2>& vertices,
                           const Math::Vector3& color, float thickness, bool closed)
{
    if (!s_Instance || !s_Instance->Initialized || vertices.size() < 2) return;
    for (size_t i = 0; i < vertices.size() - 1; ++i)
        DrawSegment(vertices[i], vertices[i + 1], thickness, color);
    if (closed && vertices.size() > 2)
        DrawSegment(vertices.back(), vertices.front(), thickness, color);
}

// =============================================================================
// DrawEllipse / DrawSolidEllipse
//
// DrawEllipse: arcs → DrawLine per segment → LineInstance per segment
// DrawSolidEllipse: triangle fan → PolyFill buffer
// =============================================================================

void Renderer2D::DrawEllipse(const Math::Vector2& center, float radiusX, float radiusY,
                             const Math::Vector3& color, int segments)
{
    if (!s_Instance || !s_Instance->Initialized) return;
    if (segments < 8) segments = 8;
    const float step = 2.0f * PI / static_cast<float>(segments);
    for (int i = 0; i < segments; ++i)
    {
        float a0 = step * i, a1 = step * (i + 1);
        s_Instance->PushLine(
            center.x + radiusX * std::cos(a0), center.y + radiusY * std::sin(a0),
            center.x + radiusX * std::cos(a1), center.y + radiusY * std::sin(a1),
            color.x, color.y, color.z, 1.0f);
    }
}

void Renderer2D::DrawSolidEllipse(const Math::Vector2& center, float radiusX, float radiusY,
                                  const Math::Vector3& color, int segments)
{
    if (!s_Instance || !s_Instance->Initialized) return;
    if (segments < 8) segments = 8;
    const float cr = color.x, cg = color.y, cb = color.z;
    const float step = 2.0f * PI / static_cast<float>(segments);
    Math::Vector2 prev{center.x + radiusX, center.y};
    for (int i = 1; i <= segments; ++i)
    {
        float a = step * i;
        Math::Vector2 next{center.x + radiusX * std::cos(a), center.y + radiusY * std::sin(a)};
        s_Instance->PushPolyFillVert(center.x, center.y, cr, cg, cb);
        s_Instance->PushPolyFillVert(next.x,   next.y,   cr, cg, cb);
        s_Instance->PushPolyFillVert(prev.x,   prev.y,   cr, cg, cb);
        prev = next;
    }
}

// =============================================================================
// DrawArc / DrawSector / DrawSolidSector
// =============================================================================

void Renderer2D::DrawArc(const Math::Vector2& center, float radius,
                         float angleStart, float angleEnd,
                         const Math::Vector3& color, float thickness, int segments)
{
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    if (segments < 8) segments = 8;
    float range = angleEnd - angleStart;
    if (std::abs(range) < 0.0001f) return;
    const float step = range / static_cast<float>(segments);
    for (int i = 0; i < segments; ++i)
    {
        float a0 = angleStart + step * i;
        float a1 = angleStart + step * (i + 1);
        s_Instance->PushLine(
            center.x + radius * std::cos(a0), center.y + radius * std::sin(a0),
            center.x + radius * std::cos(a1), center.y + radius * std::sin(a1),
            color.x, color.y, color.z,
            (thickness < 1.0f) ? 1.0f : thickness);
    }
}

void Renderer2D::DrawSector(const Math::Vector2& center, float radius,
                            float angleStart, float angleEnd,
                            const Math::Vector3& color, int segments)
{
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    if (segments < 8) segments = 8;
    float range = angleEnd - angleStart;
    if (std::abs(range) < 0.0001f) return;

    // Two radial lines
    DrawLine(center,
             {center.x + radius * std::cos(angleStart), center.y + radius * std::sin(angleStart)},
             color, 1.0f);
    DrawLine(center,
             {center.x + radius * std::cos(angleEnd),   center.y + radius * std::sin(angleEnd)},
             color, 1.0f);
    // Arc outline
    DrawArc(center, radius, angleStart, angleEnd, color, 1.0f, segments);
}

void Renderer2D::DrawSolidSector(const Math::Vector2& center, float radius,
                                 float angleStart, float angleEnd,
                                 const Math::Vector3& color, int segments)
{
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    if (segments < 8) segments = 8;
    float range = angleEnd - angleStart;
    if (std::abs(range) < 0.0001f) return;

    const float cr = color.x, cg = color.y, cb = color.z;
    const float step = range / static_cast<float>(segments);
    Math::Vector2 prev{center.x + radius * std::cos(angleStart),
                       center.y + radius * std::sin(angleStart)};
    for (int i = 1; i <= segments; ++i)
    {
        float a = angleStart + step * i;
        Math::Vector2 next{center.x + radius * std::cos(a), center.y + radius * std::sin(a)};
        s_Instance->PushPolyFillVert(center.x, center.y, cr, cg, cb);
        s_Instance->PushPolyFillVert(next.x,   next.y,   cr, cg, cb);
        s_Instance->PushPolyFillVert(prev.x,   prev.y,   cr, cg, cb);
        prev = next;
    }
}

// =============================================================================
// DrawShape — dispatcher, unchanged from original
// =============================================================================

void Renderer2D::DrawShape(const ShapeDescriptor& shape)
{
    if (!s_Instance || !s_Instance->Initialized) return;
    switch (shape.type)
    {
        case ShapeType::Circle:
            shape.filled
                ? DrawSolidCircle(shape.position, shape.params.circle.radius, shape.color, shape.segments)
                : DrawCircle     (shape.position, shape.params.circle.radius, shape.color, shape.segments);
            break;
        case ShapeType::Polygon:
            shape.filled ? DrawSolidPolygon(shape.vertices, shape.color)
                         : DrawPolygon     (shape.vertices, shape.color);
            break;
        case ShapeType::Rectangle:
            DrawQuad(shape.position,
                     {shape.params.rect.width, shape.params.rect.height},
                     {shape.params.rect.width * 0.5f, shape.params.rect.height * 0.5f},
                     shape.color, shape.rotation);
            break;
        case ShapeType::Ellipse:
            shape.filled
                ? DrawSolidEllipse(shape.position, shape.params.ellipse.radiusX, shape.params.ellipse.radiusY, shape.color, shape.segments)
                : DrawEllipse     (shape.position, shape.params.ellipse.radiusX, shape.params.ellipse.radiusY, shape.color, shape.segments);
            break;
        case ShapeType::Capsule:
        {
            Math::Vector2 c1{shape.params.segment.startX, shape.params.segment.startY};
            Math::Vector2 c2{shape.params.segment.endX,   shape.params.segment.endY};
            shape.filled ? DrawSolidCapsule(c1, c2, shape.thickness, shape.color, shape.segments)
                         : DrawCapsule     (c1, c2, shape.thickness, shape.color, shape.segments);
            break;
        }
        case ShapeType::Segment:
            DrawSegment({shape.params.segment.startX, shape.params.segment.startY},
                        {shape.params.segment.endX,   shape.params.segment.endY},
                        shape.thickness, shape.color);
            break;
        case ShapeType::Chain:
            DrawChain(shape.vertices, shape.color, shape.thickness, true);
            break;
        case ShapeType::Arc:
            DrawArc(shape.position, shape.params.arc.radius,
                    shape.params.arc.angleStart, shape.params.arc.angleEnd,
                    shape.color, shape.thickness, shape.segments);
            break;
        case ShapeType::Sector:
            shape.filled
                ? DrawSolidSector(shape.position, shape.params.sector.outerRadius,
                                  shape.params.sector.angleStart, shape.params.sector.angleEnd,
                                  shape.color, shape.segments)
                : DrawSector     (shape.position, shape.params.sector.outerRadius,
                                  shape.params.sector.angleStart, shape.params.sector.angleEnd,
                                  shape.color, shape.segments);
            break;
    }
}

// =============================================================================
// Debug draw helpers
// =============================================================================

void Renderer2D::DrawManifold(const Math::Vector2& contactPoint,
                              const Math::Vector2& normal,
                              float /*separation*/,
                              bool isTouching)
{
    if (!s_Instance || !s_Instance->Initialized) return;
    Math::Vector3 pColor = isTouching ? Math::Vector3{1,0,0} : Math::Vector3{1,1,0};
    DrawSolidCircle(contactPoint, 3.0f, pColor, 16);
    Math::Vector2 nEnd = contactPoint + normal * 20.0f;
    DrawLine(contactPoint, nEnd, {0,1,0}, 2.0f);
    // Arrowhead
    float asz = 5.0f;
    Math::Vector2 base = nEnd - normal * asz;
    Math::Vector2 perp = {-normal.y, normal.x};
    DrawLine(nEnd, base + perp * (asz * 0.5f), {0,1,0}, 2.0f);
    DrawLine(nEnd, base - perp * (asz * 0.5f), {0,1,0}, 2.0f);
}

void Renderer2D::DrawAABB(const Math::Vector2& min, const Math::Vector2& max,
                          const Math::Vector3& color)
{
    if (!s_Instance || !s_Instance->Initialized) return;
    // 4 edges as line instances
    DrawLine(min,               {max.x, min.y}, color, 1.0f);
    DrawLine({max.x, min.y},    max,            color, 1.0f);
    DrawLine(max,               {min.x, max.y}, color, 1.0f);
    DrawLine({min.x, max.y},    min,            color, 1.0f);
}

void Renderer2D::DrawTransform(const Math::Vector2& position, float rotation, float scale)
{
    if (!s_Instance || !s_Instance->Initialized) return;
    float len = 50.0f * scale;
    Math::Vector2 xAxis{std::cos(rotation), std::sin(rotation)};
    Math::Vector2 yAxis{-std::sin(rotation), std::cos(rotation)};
    DrawLine(position, position + xAxis * len, {1,0,0}, 2.0f);
    DrawLine(position, position + yAxis * len, {0,1,0}, 2.0f);
}

// =============================================================================
// State setters
// =============================================================================

void Renderer2D::SetLineWidth(float width)
{
    if (!s_Instance) return;
    s_Instance->CurrentLineWidth = (width < 1.0f) ? 1.0f : width;
    // Note: glLineWidth is not called — all lines are rendered as quads.
    // CurrentLineWidth serves as the default thickness for DrawLine calls
    // that do not specify a thickness explicitly (no such calls exist in the
    // current API, but callers that set width before drawing can pass it).
}

float Renderer2D::GetLineWidth()
{
    return s_Instance ? s_Instance->CurrentLineWidth : 1.0f;
}

void Renderer2D::EnableBlending(bool enable)
{
    if (s_Instance) s_Instance->BlendingEnabled = enable;
}

void Renderer2D::EnableDepthTest(bool enable)
{
    if (s_Instance) s_Instance->DepthTestEnabled = enable;
}

void Renderer2D::EnableCulling(bool enable)
{
    if (s_Instance) s_Instance->CullingEnabled = enable;
}

} // namespace Nyon::Graphics