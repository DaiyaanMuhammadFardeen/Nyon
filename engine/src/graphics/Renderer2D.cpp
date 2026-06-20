#include "nyon/graphics/Renderer2D.h"
#include "nyon/core/Application.h"
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <filesystem>
#include <unistd.h>
#include <limits.h>
#include <fstream>
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
std::string GetShaderDir() {
    char exePath[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", exePath, sizeof(exePath)-1);
    if (len == -1) {
        return "../../../engine/assets/shaders/"; }
    exePath[len] = '\0';
    std::filesystem::path exe(exePath);
    auto nyonDir = exe.parent_path().parent_path().parent_path().parent_path();  
    auto shaderDir = nyonDir / "engine" / "assets" / "shaders";
    return shaderDir.string() + "/"; }
namespace Nyon::Graphics {
static std::string LoadShaderSource(const std::string& filename) {
    std::string filepath = GetShaderDir() + filename;
    std::ifstream file(filepath, std::ios::in | std::ios::binary);
    if (!file.is_open())
        throw std::runtime_error("Renderer2D: failed to open shader: " + filepath);
    std::stringstream buf;
    buf << file.rdbuf();
    return buf.str(); }
glm::mat4 Camera2D::GetViewMatrix() const {
    glm::mat4 v = glm::mat4(1.0f);
    v = glm::rotate(v, -rotation, glm::vec3(0.0f, 0.0f, 1.0f));
    v = glm::translate(v, glm::vec3(-position.x, -position.y, 0.0f));
    return v; }
glm::mat4 Camera2D::GetProjectionMatrix(float sw, float sh) const {
    return glm::ortho(0.0f, sw / zoom, 0.0f, sh / zoom, nearPlane, farPlane); }
glm::mat4 Camera2D::GetViewProjectionMatrix(float sw, float sh) const {
    return GetProjectionMatrix(sw, sh) * GetViewMatrix(); }
Math::Vector2 Camera2D::ScreenToWorld(Math::Vector2 sp, float sw, float sh) const {
    float nx = (2.0f * sp.x / sw) - 1.0f;
    float ny = 1.0f - (2.0f * sp.y / sh);
    glm::vec4 wp = glm::inverse(GetViewProjectionMatrix(sw, sh)) * glm::vec4(nx, ny, 0.0f, 1.0f);
    return {wp.x, wp.y}; }
Math::Vector2 Camera2D::WorldToScreen(Math::Vector2 wp, float sw, float sh) const {
    glm::vec4 cp = GetViewProjectionMatrix(sw, sh) * glm::vec4(wp.x, wp.y, 0.0f, 1.0f);
    return {(cp.x + 1.0f) * 0.5f * sw, (1.0f - cp.y) * 0.5f * sh}; }
struct Renderer2D::Impl {
    struct QuadInstance {
        float px, py;        
        float sx, sy;        
        float ox, oy;        
        float angle;         
        float r, g, b;        };
    static_assert(sizeof(QuadInstance) == 40, "QuadInstance size mismatch");
    struct CircleInstance {
        float cx, cy;        
        float radius;
        float r, g, b;       
        float outlined;       };
    static_assert(sizeof(CircleInstance) == 28, "CircleInstance size mismatch");
    struct LineInstance {
        float x0, y0;        
        float x1, y1;        
        float r, g, b;       
        float thickness;      };
    static_assert(sizeof(LineInstance) == 32, "LineInstance size mismatch");
    struct CapsuleInstance {
        float cx0, cy0;      
        float cx1, cy1;      
        float radius;
        float r, g, b;       
        float outlined;       };
    static_assert(sizeof(CapsuleInstance) == 36, "CapsuleInstance size mismatch");
    static constexpr uint32_t MAX_QUADS      = 131072;    
    static constexpr uint32_t MAX_CIRCLES    = 524288;    
    static constexpr uint32_t MAX_LINES      = 262144;    
    static constexpr uint32_t MAX_CAPSULES   = 32768;     
    static constexpr uint32_t MAX_POLY_FILL  = 65536;     
    static constexpr uint32_t MAX_POLY_LINE  = 65536;     
    static constexpr int CIRCLE_SEG  = 16;   
    static constexpr int CAPSULE_SEG = 8;    
    static constexpr int NUM_FRAMES = 3;
    int CurrentFrame = 0;
    GLuint QuadVAO      = 0;
    GLuint QuadMeshVBO  = 0;    
    GLuint QuadInstVBO  = 0;    
    GLuint QuadShader   = 0;
    GLint  QuadVP_Loc   = -1;
    QuadInstance* QuadInstBase  = nullptr;   
    uint32_t      QuadInstCount = 0;
    GLsync QuadFences[NUM_FRAMES] = {};
    GLuint CircleVAO      = 0;
    GLuint CircleMeshVBO  = 0;
    GLuint CircleInstVBO  = 0;
    GLuint CircleShader   = 0;
    GLint  CircleVP_Loc   = -1;
    CircleInstance* CircleInstBase  = nullptr;
    uint32_t        CircleInstCount = 0;
    GLsync CircleFences[NUM_FRAMES] = {};
    GLuint LineVAO      = 0;
    GLuint LineMeshVBO  = 0;
    GLuint LineInstVBO  = 0;
    GLuint LineShader   = 0;
    GLint  LineVP_Loc   = -1;
    LineInstance* LineInstBase  = nullptr;
    uint32_t      LineInstCount = 0;
    GLsync LineFences[NUM_FRAMES] = {};
    GLuint CapsuleVAO      = 0;
    GLuint CapsuleMeshVBO  = 0;
    GLuint CapsuleInstVBO  = 0;
    GLuint CapsuleShader   = 0;
    GLint  CapsuleVP_Loc   = -1;
    CapsuleInstance* CapsuleInstBase  = nullptr;
    uint32_t         CapsuleInstCount = 0;
    GLsync CapsuleFences[NUM_FRAMES] = {};
    int CapsuleMeshVertCount = 0;   
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
    Camera2D  CurrentCamera;
    glm::mat4 ViewMatrix       = glm::mat4(1.0f);
    glm::mat4 ProjectionMatrix = glm::mat4(1.0f);
    bool  Initialized       = false;
    bool  GLAvailable       = false;
    bool  BlendingEnabled   = true;
    bool  DepthTestEnabled  = false;
    bool  CullingEnabled    = false;
    float CurrentLineWidth  = 1.0f;
    bool CheckGLFunctionsLoaded() {
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
        bool instanced =
            glDrawArraysInstanced != nullptr &&
            glVertexAttribDivisor != nullptr;
        bool persistent =
            glBufferStorage   != nullptr &&
            glMapBufferRange  != nullptr &&
            glUnmapBuffer     != nullptr;
        bool sync =
            glFenceSync        != nullptr &&
            glClientWaitSync   != nullptr &&
            glDeleteSync       != nullptr;
        if (!instanced)  printf("Renderer2D: GL 4.3 instanced draw functions missing\n");
        if (!persistent) printf("Renderer2D: GL 4.4 buffer storage functions missing\n");
        if (!sync)       printf("Renderer2D: GL 3.2 sync functions missing\n");
        return core && instanced && persistent && sync; }
    GLuint CompileShader(GLenum type, const char* source) {
        GLuint id = glCreateShader(type);
        if (id == 0) return 0;
        glShaderSource(id, 1, &source, nullptr);
        glCompileShader(id);
        GLint ok = GL_FALSE;
        glGetShaderiv(id, GL_COMPILE_STATUS, &ok);
        if (ok == GL_FALSE) {
            GLint len = 0;
            glGetShaderiv(id, GL_INFO_LOG_LENGTH, &len);
            if (len > 0) {
                auto* msg = static_cast<char*>(alloca(static_cast<size_t>(len)));
                glGetShaderInfoLog(id, len, &len, msg);
                printf("Renderer2D: shader compile error (%s):\n%s\n",
                       type == GL_VERTEX_SHADER ? "vert" : "frag", msg); }
            glDeleteShader(id);
            return 0; }
        return id; }
    GLuint CreateProgram(const char* vsSrc, const char* fsSrc) {
        GLuint vs = CompileShader(GL_VERTEX_SHADER,   vsSrc);
        GLuint fs = CompileShader(GL_FRAGMENT_SHADER, fsSrc);
        if (vs == 0 || fs == 0) {
            if (vs) glDeleteShader(vs);
            if (fs) glDeleteShader(fs);
            return 0; }
        GLuint prog = glCreateProgram();
        if (prog == 0) { glDeleteShader(vs); glDeleteShader(fs); return 0; }
        glAttachShader(prog, vs);
        glAttachShader(prog, fs);
        glLinkProgram(prog);
        GLint ok = GL_FALSE;
        glGetProgramiv(prog, GL_LINK_STATUS, &ok);
        if (ok == GL_FALSE) {
            GLint len = 0;
            glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &len);
            if (len > 0) {
                auto* msg = static_cast<char*>(alloca(static_cast<size_t>(len)));
                glGetProgramInfoLog(prog, len, &len, msg);
                printf("Renderer2D: shader link error:\n%s\n", msg); }
            glDeleteShader(vs); glDeleteShader(fs);
            glDeleteProgram(prog);
            return 0; }
        glValidateProgram(prog);
        glDeleteShader(vs);
        glDeleteShader(fs);
        return prog; }
    void* AllocatePersistentBuffer(GLuint& vbo, GLsizeiptr singleFrameBytes) {
        const GLbitfield flags = GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT;
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferStorage(GL_ARRAY_BUFFER, singleFrameBytes * NUM_FRAMES, nullptr, flags);
        void* ptr = glMapBufferRange(GL_ARRAY_BUFFER, 0, singleFrameBytes * NUM_FRAMES, flags);
        if (!ptr)
            printf("Renderer2D: glMapBufferRange failed (size=%zu)\n",
                   static_cast<size_t>(singleFrameBytes * NUM_FRAMES));
        return ptr; }
    void WaitFence(GLsync* fences, int frame) {
        if (!fences[frame]) return;
        GLenum r;
        do {
            r = glClientWaitSync(fences[frame], GL_SYNC_FLUSH_COMMANDS_BIT, 1'000'000'000ULL); } while (r == GL_TIMEOUT_EXPIRED);
        glDeleteSync(fences[frame]);
        fences[frame] = nullptr; }
    void PlaceFence(GLsync* fences, int frame) {
        if (fences[frame]) glDeleteSync(fences[frame]);
        fences[frame] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0); }
    void SetupQuadPipeline() {
        std::string vsSrc = LoadShaderSource("quad.vert");
        std::string fsSrc = LoadShaderSource("quad.frag");
        QuadShader = CreateProgram(vsSrc.c_str(), fsSrc.c_str());
        if (!QuadShader) return;
        QuadVP_Loc = glGetUniformLocation(QuadShader, "u_VP");
        const float mesh[] = {
            0.0f, 0.0f,    1.0f, 0.0f,    1.0f, 1.0f,    
            0.0f, 0.0f,    1.0f, 1.0f,    0.0f, 1.0f,     };
        glGenBuffers(1, &QuadMeshVBO);
        glBindBuffer(GL_ARRAY_BUFFER, QuadMeshVBO);
        glBufferStorage(GL_ARRAY_BUFFER, sizeof(mesh), mesh, 0);   
        QuadInstBase = static_cast<QuadInstance*>(
            AllocatePersistentBuffer(QuadInstVBO, MAX_QUADS * sizeof(QuadInstance))
        );
        glGenVertexArrays(1, &QuadVAO);
        glBindVertexArray(QuadVAO);
        glBindBuffer(GL_ARRAY_BUFFER, QuadMeshVBO);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, QuadInstVBO);
        using QI = QuadInstance;
        const GLsizei s = sizeof(QI);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(QI, px));     
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(QI, sx));     
        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(QI, ox));     
        glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, s, (void*)offsetof(QI, angle));  
        glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, s, (void*)offsetof(QI, r));      
        glEnableVertexAttribArray(1); glVertexAttribDivisor(1, 1);
        glEnableVertexAttribArray(2); glVertexAttribDivisor(2, 1);
        glEnableVertexAttribArray(3); glVertexAttribDivisor(3, 1);
        glEnableVertexAttribArray(4); glVertexAttribDivisor(4, 1);
        glEnableVertexAttribArray(5); glVertexAttribDivisor(5, 1);
        glBindVertexArray(0); }
    void SetupCirclePipeline() {
        std::string vsSrc = LoadShaderSource("circle.vert");
        std::string fsSrc = LoadShaderSource("circle.frag");
        CircleShader = CreateProgram(vsSrc.c_str(), fsSrc.c_str());
        if (!CircleShader) return;
        CircleVP_Loc = glGetUniformLocation(CircleShader, "u_VP");
        std::vector<float> mesh;
        mesh.reserve(CIRCLE_SEG * 3 * 2);
        const float step = 2.0f * PI / static_cast<float>(CIRCLE_SEG);
        for (int i = 0; i < CIRCLE_SEG; ++i) {
            float a0 = step * i;
            float a1 = step * (i + 1);
            mesh.push_back(0.0f); mesh.push_back(0.0f);               
            mesh.push_back(std::cos(a0)); mesh.push_back(std::sin(a0));  
            mesh.push_back(std::cos(a1)); mesh.push_back(std::sin(a1));   }
        glGenBuffers(1, &CircleMeshVBO);
        glBindBuffer(GL_ARRAY_BUFFER, CircleMeshVBO);
        glBufferStorage(GL_ARRAY_BUFFER,
                        static_cast<GLsizeiptr>(mesh.size() * sizeof(float)),
                        mesh.data(), 0);
        CircleInstBase = static_cast<CircleInstance*>(
            AllocatePersistentBuffer(CircleInstVBO, MAX_CIRCLES * sizeof(CircleInstance))
        );
        glGenVertexArrays(1, &CircleVAO);
        glBindVertexArray(CircleVAO);
        glBindBuffer(GL_ARRAY_BUFFER, CircleMeshVBO);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, CircleInstVBO);
        using CI = CircleInstance;
        const GLsizei s = sizeof(CI);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CI, cx));        
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CI, radius));    
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CI, r));         
        glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CI, outlined));  
        glEnableVertexAttribArray(1); glVertexAttribDivisor(1, 1);
        glEnableVertexAttribArray(2); glVertexAttribDivisor(2, 1);
        glEnableVertexAttribArray(3); glVertexAttribDivisor(3, 1);
        glEnableVertexAttribArray(4); glVertexAttribDivisor(4, 1);
        glBindVertexArray(0); }
    void SetupLinePipeline() {
        std::string vsSrc = LoadShaderSource("line.vert");
        std::string fsSrc = LoadShaderSource("line.frag");
        LineShader = CreateProgram(vsSrc.c_str(), fsSrc.c_str());
        if (!LineShader) return;
        LineVP_Loc = glGetUniformLocation(LineShader, "u_VP");
        const float mesh[] = {
            0.0f, -0.5f,   1.0f, -0.5f,   1.0f,  0.5f,    
            0.0f, -0.5f,   1.0f,  0.5f,   0.0f,  0.5f,     };
        glGenBuffers(1, &LineMeshVBO);
        glBindBuffer(GL_ARRAY_BUFFER, LineMeshVBO);
        glBufferStorage(GL_ARRAY_BUFFER, sizeof(mesh), mesh, 0);
        LineInstBase = static_cast<LineInstance*>(
            AllocatePersistentBuffer(LineInstVBO, MAX_LINES * sizeof(LineInstance))
        );
        glGenVertexArrays(1, &LineVAO);
        glBindVertexArray(LineVAO);
        glBindBuffer(GL_ARRAY_BUFFER, LineMeshVBO);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, LineInstVBO);
        using LI = LineInstance;
        const GLsizei s = sizeof(LI);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(LI, x0));         
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(LI, x1));         
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, s, (void*)offsetof(LI, r));          
        glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, s, (void*)offsetof(LI, thickness));  
        glEnableVertexAttribArray(1); glVertexAttribDivisor(1, 1);
        glEnableVertexAttribArray(2); glVertexAttribDivisor(2, 1);
        glEnableVertexAttribArray(3); glVertexAttribDivisor(3, 1);
        glEnableVertexAttribArray(4); glVertexAttribDivisor(4, 1);
        glBindVertexArray(0); }
    void SetupCapsulePipeline() {
        std::string vsSrc = LoadShaderSource("capsule.vert");
        std::string fsSrc = LoadShaderSource("capsule.frag");
        CapsuleShader = CreateProgram(vsSrc.c_str(), fsSrc.c_str());
        if (!CapsuleShader) return;
        CapsuleVP_Loc = glGetUniformLocation(CapsuleShader, "u_VP");
        struct CMVert { float lx, ly, type; };
        std::vector<CMVert> mesh;
        const float halfStep = PI / static_cast<float>(CAPSULE_SEG);
        for (int i = 0; i < CAPSULE_SEG; ++i) {
            float a0 = PI / 2.0f + halfStep * i;
            float a1 = PI / 2.0f + halfStep * (i + 1);
            mesh.push_back({0.0f,                0.0f,               0.0f});
            mesh.push_back({std::cos(a0),        std::sin(a0),       0.0f});
            mesh.push_back({std::cos(a1),        std::sin(a1),       0.0f}); }
        for (int i = 0; i < CAPSULE_SEG; ++i) {
            float a0 = -PI / 2.0f + halfStep * i;
            float a1 = -PI / 2.0f + halfStep * (i + 1);
            mesh.push_back({0.0f,                0.0f,               1.0f});
            mesh.push_back({std::cos(a0),        std::sin(a0),       1.0f});
            mesh.push_back({std::cos(a1),        std::sin(a1),       1.0f}); }
        mesh.push_back({0.0f, -1.0f, 2.0f}); mesh.push_back({1.0f, -1.0f, 2.0f}); mesh.push_back({1.0f,  1.0f, 2.0f});
        mesh.push_back({0.0f, -1.0f, 2.0f}); mesh.push_back({1.0f,  1.0f, 2.0f}); mesh.push_back({0.0f,  1.0f, 2.0f});
        CapsuleMeshVertCount = static_cast<int>(mesh.size());
        glGenBuffers(1, &CapsuleMeshVBO);
        glBindBuffer(GL_ARRAY_BUFFER, CapsuleMeshVBO);
        glBufferStorage(GL_ARRAY_BUFFER,
                        static_cast<GLsizeiptr>(mesh.size() * sizeof(CMVert)),
                        mesh.data(), 0);
        CapsuleInstBase = static_cast<CapsuleInstance*>(
            AllocatePersistentBuffer(CapsuleInstVBO, MAX_CAPSULES * sizeof(CapsuleInstance))
        );
        glGenVertexArrays(1, &CapsuleVAO);
        glBindVertexArray(CapsuleVAO);
        glBindBuffer(GL_ARRAY_BUFFER, CapsuleMeshVBO);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, CapsuleInstVBO);
        using CAI = CapsuleInstance;
        const GLsizei s = sizeof(CAI);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CAI, cx0));       
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CAI, cx1));       
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CAI, radius));    
        glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CAI, r));         
        glVertexAttribPointer(5, 1, GL_FLOAT, GL_FALSE, s, (void*)offsetof(CAI, outlined));  
        glEnableVertexAttribArray(1); glVertexAttribDivisor(1, 1);
        glEnableVertexAttribArray(2); glVertexAttribDivisor(2, 1);
        glEnableVertexAttribArray(3); glVertexAttribDivisor(3, 1);
        glEnableVertexAttribArray(4); glVertexAttribDivisor(4, 1);
        glEnableVertexAttribArray(5); glVertexAttribDivisor(5, 1);
        glBindVertexArray(0); }
    void SetupPolygonPipeline() {
        std::string vsSrc = LoadShaderSource("polygon.vert");
        std::string fsSrc = LoadShaderSource("polygon.frag");
        PolyShader = CreateProgram(vsSrc.c_str(), fsSrc.c_str());
        if (!PolyShader) return;
        PolyVP_Loc = glGetUniformLocation(PolyShader, "u_VP");
        PolyFillBase = static_cast<Vertex*>(
            AllocatePersistentBuffer(PolyFillVBO, MAX_POLY_FILL * sizeof(Vertex))
        );
        glGenVertexArrays(1, &PolyFillVAO);
        glBindVertexArray(PolyFillVAO);
        glBindBuffer(GL_ARRAY_BUFFER, PolyFillVBO);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<void*>(offsetof(Vertex, x)));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<void*>(offsetof(Vertex, r)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
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
        glBindVertexArray(0); }
    void UpdateProjectionMatrix(float w, float h) {
        const float left   = 0.0f;
        const float right  = w / CurrentCamera.zoom;
        const float bottom = 0.0f;
        const float top    = h / CurrentCamera.zoom;
        ProjectionMatrix = glm::ortho(left, right, bottom, top,
                                      CurrentCamera.nearPlane, CurrentCamera.farPlane); }
    void UpdateViewMatrix() {
        ViewMatrix = CurrentCamera.GetViewMatrix(); }
    inline bool PushQuad(float px, float py, float sx, float sy,
                         float ox, float oy, float angle,
                         float r,  float g,  float b) {
        if (QuadInstCount >= MAX_QUADS) return false;
        QuadInstance& inst = QuadInstBase[CurrentFrame * MAX_QUADS + QuadInstCount++];
        inst.px = px; inst.py = py;
        inst.sx = sx; inst.sy = sy;
        inst.ox = ox; inst.oy = oy;
        inst.angle = angle;
        inst.r = r; inst.g = g; inst.b = b;
        return true; }
    inline bool PushCircle(float cx, float cy, float radius,
                           float r, float g, float b, float outlined) {
        if (CircleInstCount >= MAX_CIRCLES) return false;
        CircleInstance& inst = CircleInstBase[CurrentFrame * MAX_CIRCLES + CircleInstCount++];
        inst.cx = cx; inst.cy = cy;
        inst.radius = radius;
        inst.r = r; inst.g = g; inst.b = b;
        inst.outlined = outlined;
        return true; }
    inline bool PushLine(float x0, float y0, float x1, float y1,
                         float r, float g, float b, float thickness) {
        if (LineInstCount >= MAX_LINES) return false;
        LineInstance& inst = LineInstBase[CurrentFrame * MAX_LINES + LineInstCount++];
        inst.x0 = x0; inst.y0 = y0;
        inst.x1 = x1; inst.y1 = y1;
        inst.r = r; inst.g = g; inst.b = b;
        inst.thickness = thickness;
        return true; }
    inline bool PushCapsule(float cx0, float cy0, float cx1, float cy1,
                            float radius, float r, float g, float b, float outlined) {
        if (CapsuleInstCount >= MAX_CAPSULES) return false;
        CapsuleInstance& inst = CapsuleInstBase[CurrentFrame * MAX_CAPSULES + CapsuleInstCount++];
        inst.cx0 = cx0; inst.cy0 = cy0;
        inst.cx1 = cx1; inst.cy1 = cy1;
        inst.radius = radius;
        inst.r = r; inst.g = g; inst.b = b;
        inst.outlined = outlined;
        return true; }
    inline bool PushPolyFillVert(float x, float y, float r, float g, float b) {
        if (PolyFillCount >= MAX_POLY_FILL) return false;
        Vertex& v = PolyFillBase[CurrentFrame * MAX_POLY_FILL + PolyFillCount++];
        v.x = x; v.y = y;
        v.r = r; v.g = g; v.b = b;
        v.u = 0; v.v = 0; v.nx = 0; v.ny = 0;
        return true; }
    inline bool PushPolyLineVert(float x, float y, float r, float g, float b) {
        if (PolyLineCount >= MAX_POLY_LINE) return false;
        Vertex& v = PolyLineBase[CurrentFrame * MAX_POLY_LINE + PolyLineCount++];
        v.x = x; v.y = y;
        v.r = r; v.g = g; v.b = b;
        v.u = 0; v.v = 0; v.nx = 0; v.ny = 0;
        return true; } };
std::unique_ptr<Renderer2D::Impl> Renderer2D::s_Instance;
void Renderer2D::Init() {
    s_Instance = std::make_unique<Impl>();
    s_Instance->GLAvailable = s_Instance->CheckGLFunctionsLoaded();
    if (!s_Instance->GLAvailable) {
        printf("Renderer2D::Init — required GL functions not available. "
               "Rendering disabled.\n");
        s_Instance->Initialized = true;
        return; }
    s_Instance->SetupQuadPipeline();
    s_Instance->SetupCirclePipeline();
    s_Instance->SetupLinePipeline();
    s_Instance->SetupCapsulePipeline();
    s_Instance->SetupPolygonPipeline();
    s_Instance->UpdateProjectionMatrix(1280.0f, 720.0f);
    s_Instance->Initialized = true;
    printf("Renderer2D: Initialized successfully. GPU: %s, OpenGL: %s\n",
           glGetString(GL_RENDERER), glGetString(GL_VERSION)); }
void Renderer2D::Shutdown() {
    if (!s_Instance) return;
    if (s_Instance->GLAvailable) {
        for (int f = 0; f < Impl::NUM_FRAMES; ++f) {
            s_Instance->WaitFence(s_Instance->QuadFences,      f);
            s_Instance->WaitFence(s_Instance->CircleFences,    f);
            s_Instance->WaitFence(s_Instance->LineFences,      f);
            s_Instance->WaitFence(s_Instance->CapsuleFences,   f);
            s_Instance->WaitFence(s_Instance->PolyFillFences,  f);
            s_Instance->WaitFence(s_Instance->PolyLineFences,  f); }
        auto unmap = [](GLuint vbo) {
            if (vbo) { glBindBuffer(GL_ARRAY_BUFFER, vbo); glUnmapBuffer(GL_ARRAY_BUFFER); } };
        unmap(s_Instance->QuadInstVBO);
        unmap(s_Instance->CircleInstVBO);
        unmap(s_Instance->LineInstVBO);
        unmap(s_Instance->CapsuleInstVBO);
        unmap(s_Instance->PolyFillVBO);
        unmap(s_Instance->PolyLineVBO);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        GLuint vbos[] = {
            s_Instance->QuadMeshVBO,    s_Instance->QuadInstVBO,
            s_Instance->CircleMeshVBO,  s_Instance->CircleInstVBO,
            s_Instance->LineMeshVBO,    s_Instance->LineInstVBO,
            s_Instance->CapsuleMeshVBO, s_Instance->CapsuleInstVBO,
            s_Instance->PolyFillVBO,    s_Instance->PolyLineVBO };
        glDeleteBuffers(10, vbos);
        GLuint vaos[] = {
            s_Instance->QuadVAO,
            s_Instance->CircleVAO,
            s_Instance->LineVAO,
            s_Instance->CapsuleVAO,
            s_Instance->PolyFillVAO,
            s_Instance->PolyLineVAO };
        glDeleteVertexArrays(6, vaos);
        GLuint progs[] = {
            s_Instance->QuadShader,
            s_Instance->CircleShader,
            s_Instance->LineShader,
            s_Instance->CapsuleShader,
            s_Instance->PolyShader };
        for (GLuint p : progs) if (p) glDeleteProgram(p); }
    s_Instance->Initialized = false;
    s_Instance.reset(); }
void Renderer2D::BeginScene(const Camera2D& camera) {
    if (!s_Instance || !s_Instance->Initialized) return;
    s_Instance->CurrentCamera = camera;
    if (!s_Instance->GLAvailable) return;
    const int f = s_Instance->CurrentFrame;
    s_Instance->WaitFence(s_Instance->QuadFences,      f);
    s_Instance->WaitFence(s_Instance->CircleFences,    f);
    s_Instance->WaitFence(s_Instance->LineFences,      f);
    s_Instance->WaitFence(s_Instance->CapsuleFences,   f);
    s_Instance->WaitFence(s_Instance->PolyFillFences,  f);
    s_Instance->WaitFence(s_Instance->PolyLineFences,  f);
    s_Instance->QuadInstCount    = 0;
    s_Instance->CircleInstCount  = 0;
    s_Instance->LineInstCount    = 0;
    s_Instance->CapsuleInstCount = 0;
    s_Instance->PolyFillCount    = 0;
    s_Instance->PolyLineCount    = 0;
    GLFWwindow* window = nullptr;
    try { window = Application::Get().GetWindow(); } catch (...) {}
    int w = 1280, h = 720;
    if (window) glfwGetFramebufferSize(window, &w, &h);
    if (w > 0 && h > 0) s_Instance->UpdateProjectionMatrix(float(w), float(h));
    s_Instance->UpdateViewMatrix();
    if (s_Instance->BlendingEnabled) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); } else glDisable(GL_BLEND);
    if (s_Instance->DepthTestEnabled) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
    if (s_Instance->CullingEnabled) { glEnable(GL_CULL_FACE); glCullFace(GL_BACK); }
    else glDisable(GL_CULL_FACE); }
void Renderer2D::EndScene() { Flush(); }
const Camera2D& Renderer2D::GetActiveCamera() {
    if (!s_Instance) {
        static Camera2D defaultCamera;
        return defaultCamera; }
    return s_Instance->CurrentCamera; }
void Renderer2D::SetScreenDimensions(float width, float height) {
    if (!s_Instance || !s_Instance->Initialized) return;
    if (width > 0 && height > 0) {
        s_Instance->UpdateProjectionMatrix(width, height); } }
void Renderer2D::Flush() {
    if (!s_Instance || !s_Instance->Initialized || !s_Instance->GLAvailable) return;
    auto& I = *s_Instance;
    const int f = I.CurrentFrame;
    const glm::mat4 vp = I.ProjectionMatrix * I.ViewMatrix;
#define REPOINT_INST_ATTRIB(idx, comps, stride, baseOff, memberOff)                    \
    glVertexAttribPointer((idx), (comps), GL_FLOAT, GL_FALSE, (stride),                \
        reinterpret_cast<const void*>((baseOff) + static_cast<GLintptr>(memberOff)))
    if (I.QuadInstCount > 0 && I.QuadShader) {
        glUseProgram(I.QuadShader);
        glUniformMatrix4fv(I.QuadVP_Loc, 1, GL_FALSE, &vp[0][0]);
        glBindVertexArray(I.QuadVAO);
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
        glBindVertexArray(0); }
    if (I.CircleInstCount > 0 && I.CircleShader) {
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
        glBindVertexArray(0); }
    if (I.LineInstCount > 0 && I.LineShader) {
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
        glBindVertexArray(0); }
    if (I.CapsuleInstCount > 0 && I.CapsuleShader && I.CapsuleMeshVertCount > 0) {
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
        glBindVertexArray(0); }
    if (I.PolyFillCount > 0 && I.PolyShader) {
        glUseProgram(I.PolyShader);
        glUniformMatrix4fv(I.PolyVP_Loc, 1, GL_FALSE, &vp[0][0]);
        glBindVertexArray(I.PolyFillVAO);
        const GLintptr base = static_cast<GLintptr>(f * I.MAX_POLY_FILL * sizeof(Vertex));
        const GLsizei  s    = sizeof(Vertex);
        glBindBuffer(GL_ARRAY_BUFFER, I.PolyFillVBO);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, s,
            reinterpret_cast<const void*>(base + static_cast<GLintptr>(offsetof(Vertex, x))));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, s,
            reinterpret_cast<const void*>(base + static_cast<GLintptr>(offsetof(Vertex, r))));
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(I.PolyFillCount));
        I.PlaceFence(I.PolyFillFences, f);
        glBindVertexArray(0); }
    if (I.PolyLineCount > 0 && I.PolyShader) {
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
        glBindVertexArray(0); }
#undef REPOINT_INST_ATTRIB
    s_Instance->CurrentFrame = (s_Instance->CurrentFrame + 1) % Impl::NUM_FRAMES; }
void Renderer2D::DrawQuad(const Math::Vector2& position,
                          const Math::Vector2& size,
                          const Math::Vector2& origin,
                          const Math::Vector3& color,
                          float rotation) {
    if (!s_Instance || !s_Instance->Initialized) return;
    s_Instance->PushQuad(position.x, position.y,
                         size.x,     size.y,
                         origin.x,   origin.y,
                         rotation,
                         color.x, color.y, color.z); }
void Renderer2D::DrawSolidCircle(const Math::Vector2& center, float radius,
                                 const Math::Vector3& color, int ) {
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    s_Instance->PushCircle(center.x, center.y, radius,
                           color.x, color.y, color.z, 0.0f); }
void Renderer2D::DrawCircle(const Math::Vector2& center, float radius,
                            const Math::Vector3& color, int ) {
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    s_Instance->PushCircle(center.x, center.y, radius,
                           color.x, color.y, color.z, 1.0f);    }
void Renderer2D::DrawLine(const Math::Vector2& start, const Math::Vector2& end,
                          const Math::Vector3& color, float thickness) {
    if (!s_Instance || !s_Instance->Initialized) return;
    const float t = (thickness < 1.0f) ? 1.0f : thickness;
    s_Instance->PushLine(start.x, start.y, end.x, end.y,
                         color.x, color.y, color.z, t); }
void Renderer2D::DrawSolidCapsule(const Math::Vector2& c1, const Math::Vector2& c2,
                                  float radius, const Math::Vector3& color, int ) {
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    if ((c2 - c1).Length() < 0.0001f) {
        DrawSolidCircle(c1, radius, color);
        return; }
    s_Instance->PushCapsule(c1.x, c1.y, c2.x, c2.y, radius,
                            color.x, color.y, color.z, 0.0f); }
void Renderer2D::DrawCapsule(const Math::Vector2& c1, const Math::Vector2& c2,
                             float radius, const Math::Vector3& color, int segments) {
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    if (segments < 8) segments = 8;
    Math::Vector2 axis  = c2 - c1;
    float len = axis.Length();
    if (len < 0.0001f) { DrawCircle(c1, radius, color, segments); return; }
    Math::Vector2 nor = axis * (1.0f / len);
    Math::Vector2 tan = {-nor.y, nor.x};
    const float cr = color.x, cg = color.y, cb = color.z;
    const float step = PI / static_cast<float>(segments);
    for (int i = 0; i < segments; ++i) {
        float a0 = PI / 2.0f + step * i;
        float a1 = PI / 2.0f + step * (i + 1);
        Math::Vector2 p0 = c1 + (tan * std::cos(a0) - nor * std::sin(a0)) * radius;
        Math::Vector2 p1 = c1 + (tan * std::cos(a1) - nor * std::sin(a1)) * radius;
        s_Instance->PushPolyLineVert(p0.x, p0.y, cr, cg, cb);
        s_Instance->PushPolyLineVert(p1.x, p1.y, cr, cg, cb); }
    for (int i = 0; i < segments; ++i) {
        float a0 = -PI / 2.0f + step * i;
        float a1 = -PI / 2.0f + step * (i + 1);
        Math::Vector2 p0 = c2 + (tan * std::cos(a0) + nor * std::sin(a0)) * radius;
        Math::Vector2 p1 = c2 + (tan * std::cos(a1) + nor * std::sin(a1)) * radius;
        s_Instance->PushPolyLineVert(p0.x, p0.y, cr, cg, cb);
        s_Instance->PushPolyLineVert(p1.x, p1.y, cr, cg, cb); }
    DrawLine(c1 + tan * radius, c2 + tan * radius, color, 1.0f);
    DrawLine(c1 - tan * radius, c2 - tan * radius, color, 1.0f); }
void Renderer2D::DrawSolidPolygon(const std::vector<Math::Vector2>& vertices,
                                  const Math::Vector3& color) {
    if (!s_Instance || !s_Instance->Initialized || vertices.size() < 3) return;
#ifndef NDEBUG
    auto isConvex = [](const std::vector<Math::Vector2>& v) -> bool {
        int sign = 0;
        for (size_t i = 0; i < v.size(); ++i) {
            const auto& p1 = v[i];
            const auto& p2 = v[(i + 1) % v.size()];
            const auto& p3 = v[(i + 2) % v.size()];
            float cross = (p2.x - p1.x) * (p3.y - p2.y) - (p2.y - p1.y) * (p3.x - p2.x);
            if (cross != 0.0f) {
                int s = (cross > 0.0f) ? 1 : -1;
                if (sign == 0) sign = s;
                else if (s != sign) return false; } }
        return true; };
    assert(isConvex(vertices) && "DrawSolidPolygon requires a convex polygon");
#endif
    const float cr = color.x, cg = color.y, cb = color.z;
    const Math::Vector2& v0 = vertices[0];
    for (size_t i = 1; i + 1 < vertices.size(); ++i) {
        const Math::Vector2& v1 = vertices[i];
        const Math::Vector2& v2 = vertices[i + 1];
        s_Instance->PushPolyFillVert(v0.x, v0.y, cr, cg, cb);
        s_Instance->PushPolyFillVert(v1.x, v1.y, cr, cg, cb);
        s_Instance->PushPolyFillVert(v2.x, v2.y, cr, cg, cb); } }
void Renderer2D::DrawPolygon(const std::vector<Math::Vector2>& vertices,
                             const Math::Vector3& color) {
    if (!s_Instance || !s_Instance->Initialized || vertices.size() < 2) return;
    for (size_t i = 0; i < vertices.size(); ++i) {
        const Math::Vector2& a = vertices[i];
        const Math::Vector2& b = vertices[(i + 1) % vertices.size()];
        s_Instance->PushLine(a.x, a.y, b.x, b.y,
                             color.x, color.y, color.z, 1.0f); } }
void Renderer2D::DrawSegment(const Math::Vector2& p1, const Math::Vector2& p2,
                             float thickness, const Math::Vector3& color) {
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
        p2 + t * ht,   p2 - t * ht };
    DrawSolidPolygon(rect, color);
    DrawSolidCircle(p1, ht, color, 16);
    DrawSolidCircle(p2, ht, color, 16); }
void Renderer2D::DrawChain(const std::vector<Math::Vector2>& vertices,
                           const Math::Vector3& color, float thickness, bool closed) {
    if (!s_Instance || !s_Instance->Initialized || vertices.size() < 2) return;
    for (size_t i = 0; i < vertices.size() - 1; ++i)
        DrawSegment(vertices[i], vertices[i + 1], thickness, color);
    if (closed && vertices.size() > 2)
        DrawSegment(vertices.back(), vertices.front(), thickness, color); }
void Renderer2D::DrawEllipse(const Math::Vector2& center, float radiusX, float radiusY,
                             const Math::Vector3& color, int segments) {
    if (!s_Instance || !s_Instance->Initialized) return;
    if (segments < 8) segments = 8;
    const float step = 2.0f * PI / static_cast<float>(segments);
    for (int i = 0; i < segments; ++i) {
        float a0 = step * i, a1 = step * (i + 1);
        s_Instance->PushLine(
            center.x + radiusX * std::cos(a0), center.y + radiusY * std::sin(a0),
            center.x + radiusX * std::cos(a1), center.y + radiusY * std::sin(a1),
            color.x, color.y, color.z, 1.0f); } }
void Renderer2D::DrawSolidEllipse(const Math::Vector2& center, float radiusX, float radiusY,
                                  const Math::Vector3& color, int segments) {
    if (!s_Instance || !s_Instance->Initialized) return;
    if (segments < 8) segments = 8;
    const float cr = color.x, cg = color.y, cb = color.z;
    const float step = 2.0f * PI / static_cast<float>(segments);
    Math::Vector2 prev{center.x + radiusX, center.y};
    for (int i = 1; i <= segments; ++i) {
        float a = step * i;
        Math::Vector2 next{center.x + radiusX * std::cos(a), center.y + radiusY * std::sin(a)};
        s_Instance->PushPolyFillVert(center.x, center.y, cr, cg, cb);
        s_Instance->PushPolyFillVert(next.x,   next.y,   cr, cg, cb);
        s_Instance->PushPolyFillVert(prev.x,   prev.y,   cr, cg, cb);
        prev = next; } }
void Renderer2D::DrawArc(const Math::Vector2& center, float radius,
                         float angleStart, float angleEnd,
                         const Math::Vector3& color, float thickness, int segments) {
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    if (segments < 8) segments = 8;
    float range = angleEnd - angleStart;
    if (std::abs(range) < 0.0001f) return;
    const float step = range / static_cast<float>(segments);
    for (int i = 0; i < segments; ++i) {
        float a0 = angleStart + step * i;
        float a1 = angleStart + step * (i + 1);
        s_Instance->PushLine(
            center.x + radius * std::cos(a0), center.y + radius * std::sin(a0),
            center.x + radius * std::cos(a1), center.y + radius * std::sin(a1),
            color.x, color.y, color.z,
            (thickness < 1.0f) ? 1.0f : thickness); } }
void Renderer2D::DrawSector(const Math::Vector2& center, float radius,
                            float angleStart, float angleEnd,
                            const Math::Vector3& color, int segments) {
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    if (segments < 8) segments = 8;
    float range = angleEnd - angleStart;
    if (std::abs(range) < 0.0001f) return;
    DrawLine(center, {center.x + radius * std::cos(angleStart), center.y + radius * std::sin(angleStart)},
             color, 1.0f);
    DrawLine(center, {center.x + radius * std::cos(angleEnd),   center.y + radius * std::sin(angleEnd)},
             color, 1.0f);
    DrawArc(center, radius, angleStart, angleEnd, color, 1.0f, segments); }
void Renderer2D::DrawSolidSector(const Math::Vector2& center, float radius,
                                 float angleStart, float angleEnd,
                                 const Math::Vector3& color, int segments) {
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    if (segments < 8) segments = 8;
    float range = angleEnd - angleStart;
    if (std::abs(range) < 0.0001f) return;
    const float cr = color.x, cg = color.y, cb = color.z;
    const float step = range / static_cast<float>(segments);
    Math::Vector2 prev{center.x + radius * std::cos(angleStart),
                       center.y + radius * std::sin(angleStart)};
    for (int i = 1; i <= segments; ++i) {
        float a = angleStart + step * i;
        Math::Vector2 next{center.x + radius * std::cos(a), center.y + radius * std::sin(a)};
        s_Instance->PushPolyFillVert(center.x, center.y, cr, cg, cb);
        s_Instance->PushPolyFillVert(next.x,   next.y,   cr, cg, cb);
        s_Instance->PushPolyFillVert(prev.x,   prev.y,   cr, cg, cb);
        prev = next; } }
void Renderer2D::DrawShape(const ShapeDescriptor& shape) {
    if (!s_Instance || !s_Instance->Initialized) return;
    switch (shape.type) {
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
            DrawQuad(shape.position, {shape.params.rect.width, shape.params.rect.height}, {shape.params.rect.width * 0.5f, shape.params.rect.height * 0.5f},
                     shape.color, shape.rotation);
            break;
        case ShapeType::Ellipse:
            shape.filled
                ? DrawSolidEllipse(shape.position, shape.params.ellipse.radiusX, shape.params.ellipse.radiusY, shape.color, shape.segments)
                : DrawEllipse     (shape.position, shape.params.ellipse.radiusX, shape.params.ellipse.radiusY, shape.color, shape.segments);
            break;
        case ShapeType::Capsule: {
            Math::Vector2 c1{shape.params.segment.startX, shape.params.segment.startY};
            Math::Vector2 c2{shape.params.segment.endX,   shape.params.segment.endY};
            shape.filled ? DrawSolidCapsule(c1, c2, shape.thickness, shape.color, shape.segments)
                         : DrawCapsule     (c1, c2, shape.thickness, shape.color, shape.segments);
            break; }
        case ShapeType::Segment:
            DrawSegment({shape.params.segment.startX, shape.params.segment.startY}, {shape.params.segment.endX,   shape.params.segment.endY},
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
            break; } }
void Renderer2D::DrawManifold(const Math::Vector2& contactPoint,
                              const Math::Vector2& normal,
                              float ,
                              bool isTouching) {
    if (!s_Instance || !s_Instance->Initialized) return;
    Math::Vector3 pColor = isTouching ? Math::Vector3{1,0,0} : Math::Vector3{1,1,0};
    DrawSolidCircle(contactPoint, 3.0f, pColor, 16);
    Math::Vector2 nEnd = contactPoint + normal * 20.0f;
    DrawLine(contactPoint, nEnd, {0,1,0}, 2.0f);
    float asz = 5.0f;
    Math::Vector2 base = nEnd - normal * asz;
    Math::Vector2 perp = {-normal.y, normal.x};
    DrawLine(nEnd, base + perp * (asz * 0.5f), {0,1,0}, 2.0f);
    DrawLine(nEnd, base - perp * (asz * 0.5f), {0,1,0}, 2.0f); }
void Renderer2D::DrawAABB(const Math::Vector2& min, const Math::Vector2& max,
                          const Math::Vector3& color) {
    if (!s_Instance || !s_Instance->Initialized) return;
    DrawLine(min,               {max.x, min.y}, color, 1.0f);
    DrawLine({max.x, min.y},    max,            color, 1.0f);
    DrawLine(max,               {min.x, max.y}, color, 1.0f);
    DrawLine({min.x, max.y},    min,            color, 1.0f); }
void Renderer2D::DrawTransform(const Math::Vector2& position, float rotation, float scale) {
    if (!s_Instance || !s_Instance->Initialized) return;
    float len = 50.0f * scale;
    Math::Vector2 xAxis{std::cos(rotation), std::sin(rotation)};
    Math::Vector2 yAxis{-std::sin(rotation), std::cos(rotation)};
    DrawLine(position, position + xAxis * len, {1,0,0}, 2.0f);
    DrawLine(position, position + yAxis * len, {0,1,0}, 2.0f); }
void Renderer2D::SetLineWidth(float width) {
    if (!s_Instance) return;
    s_Instance->CurrentLineWidth = (width < 1.0f) ? 1.0f : width; }
float Renderer2D::GetLineWidth() {
    return s_Instance ? s_Instance->CurrentLineWidth : 1.0f; }
void Renderer2D::EnableBlending(bool enable) {
    if (s_Instance) s_Instance->BlendingEnabled = enable; }
void Renderer2D::EnableDepthTest(bool enable) {
    if (s_Instance) s_Instance->DepthTestEnabled = enable; }
void Renderer2D::EnableCulling(bool enable) {
    if (s_Instance) s_Instance->CullingEnabled = enable; } }  