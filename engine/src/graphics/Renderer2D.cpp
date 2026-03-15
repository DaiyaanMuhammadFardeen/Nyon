#include "nyon/graphics/Renderer2D.h"
#include "nyon/core/Application.h"
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <cassert>

#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif

// PI constant with full precision
constexpr float PI = 3.14159265358979323846f;

namespace Nyon::Graphics
{
    // =======================================================================
    // Camera2D Implementation
    // =======================================================================
    
    glm::mat4 Camera2D::GetViewMatrix() const
    {
        // TRS inverse: scale by zoom, rotate, then translate to -position
        // GLM applies transformations right-to-left (column-major convention)
        glm::mat4 view = glm::mat4(1.0f);
        view = glm::scale(view, glm::vec3(zoom, zoom, 1.0f));
        view = glm::rotate(view, -rotation, glm::vec3(0.0f, 0.0f, 1.0f));
        view = glm::translate(view, glm::vec3(-position.x, -position.y, 0.0f));
        return view;
    }
    
    glm::mat4 Camera2D::GetProjectionMatrix(float screenWidth, float screenHeight) const
    {
        // Y-up coordinate system: bottom=0, top=screenHeight/zoom
        // For Y-down (screen coordinates), swap bottom and top
        float left = 0.0f;
        float right = screenWidth / zoom;
        float bottom = 0.0f;
        float top = screenHeight / zoom;
        
        return glm::ortho(left, right, bottom, top, nearPlane, farPlane);
    }
    
    glm::mat4 Camera2D::GetViewProjectionMatrix(float screenWidth, float screenHeight) const
    {
        return GetProjectionMatrix(screenWidth, screenHeight) * GetViewMatrix();
    }
    
    Math::Vector2 Camera2D::ScreenToWorld(Math::Vector2 screenPos, float screenWidth, float screenHeight) const
    {
        // Convert screen coordinates [0, screenWidth] × [0, screenHeight] to NDC [-1, 1]
        float ndcX = (2.0f * screenPos.x / screenWidth) - 1.0f;
        float ndcY = 1.0f - (2.0f * screenPos.y / screenHeight);  // Flip Y for screen coords
        
        glm::vec4 worldPos = glm::inverse(GetViewProjectionMatrix(screenWidth, screenHeight)) * 
                            glm::vec4(ndcX, ndcY, 0.0f, 1.0f);
        return Math::Vector2(worldPos.x, worldPos.y);
    }
    
    Math::Vector2 Camera2D::WorldToScreen(Math::Vector2 worldPos, float screenWidth, float screenHeight) const
    {
        glm::vec4 clipPos = GetViewProjectionMatrix(screenWidth, screenHeight) * 
                           glm::vec4(worldPos.x, worldPos.y, 0.0f, 1.0f);
        
        // Convert from NDC [-1, 1] to screen coordinates [0, screenWidth] × [0, screenHeight]
        float screenX = (clipPos.x + 1.0f) * 0.5f * screenWidth;
        float screenY = (1.0f - clipPos.y) * 0.5f * screenHeight;  // Flip Y for screen coords
        
        return Math::Vector2(screenX, screenY);
    }

    // =======================================================================
    // Renderer2D Internal Implementation
    // =======================================================================
    
    struct Renderer2D::Impl
    {
        // GL Objects
        unsigned int VAO = 0;
        unsigned int VBO = 0;
        unsigned int EBO = 0;
        unsigned int ShaderProgram = 0;
        unsigned int LineWidthUniform = 0;
        unsigned int ViewUniform = 0;
        unsigned int ProjectionUniform = 0;
        
        // State
        bool Initialized = false;
        bool GLAvailable = false;
        bool BlendingEnabled = true;
        bool DepthTestEnabled = false;
        bool CullingEnabled = false;
        float CurrentLineWidth = 1.0f;
        
        // Buffers
        std::vector<Vertex> QuadBuffer;
        std::vector<Vertex> LineBuffer;
        std::vector<unsigned int> IndexBuffer;
        
        // Camera
        Camera2D CurrentCamera;
        glm::mat4 ViewMatrix = glm::mat4(1.0f);
        glm::mat4 ProjectionMatrix = glm::mat4(1.0f);
        
        // Constants
        const uint32_t MaxQuads = 20000;
        const uint32_t MaxVertices = MaxQuads * 6;
        const uint32_t MaxIndices = MaxQuads * 6;
        
        bool CheckGLFunctionsLoaded()
        {
            return glGenVertexArrays != nullptr &&
                   glDeleteVertexArrays != nullptr &&
                   glBindVertexArray != nullptr &&
                   glGenBuffers != nullptr &&
                   glDeleteBuffers != nullptr &&
                   glBindBuffer != nullptr &&
                   glBufferData != nullptr &&
                   glBufferSubData != nullptr &&
                   glVertexAttribPointer != nullptr &&
                   glEnableVertexAttribArray != nullptr &&
                   glCreateShader != nullptr &&
                   glShaderSource != nullptr &&
                   glCompileShader != nullptr &&
                   glGetShaderiv != nullptr &&
                   glGetShaderInfoLog != nullptr &&
                   glDeleteShader != nullptr &&
                   glCreateProgram != nullptr &&
                   glAttachShader != nullptr &&
                   glLinkProgram != nullptr &&
                   glGetProgramiv != nullptr &&
                   glGetProgramInfoLog != nullptr &&
                   glValidateProgram != nullptr &&
                   glDeleteProgram != nullptr &&
                   glUseProgram != nullptr &&
                   glGetUniformLocation != nullptr &&
                   glUniformMatrix4fv != nullptr &&
                   glUniform1f != nullptr &&
                   glUniform1i != nullptr &&
                   glDrawArrays != nullptr &&
                   glDrawElements != nullptr &&
                   glEnable != nullptr &&
                   glDisable != nullptr &&
                   glBlendFunc != nullptr &&
                   glBlendFuncSeparate != nullptr &&
                   glPolygonMode != nullptr &&
                   glCullFace != nullptr &&
                   glFrontFace != nullptr &&
                   glLineWidth != nullptr;
        }
        
        unsigned int CompileShader(unsigned int type, const char* source)
        {
            unsigned int id = glCreateShader(type);
            if (id == 0) return 0;
            
            glShaderSource(id, 1, &source, nullptr);
            glCompileShader(id);
            
            int result = GL_FALSE;
            glGetShaderiv(id, GL_COMPILE_STATUS, &result);
            if (result == GL_FALSE)
            {
                int length = 0;
                glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
                if (length > 0)
                {
                    char* message = static_cast<char*>(alloca(static_cast<size_t>(length)));
                    glGetShaderInfoLog(id, length, &length, message);
                    printf("Failed to compile %s shader:\n%s\n",
                           type == GL_VERTEX_SHADER ? "vertex" : "fragment", message);
                }
                glDeleteShader(id);
                return 0;
            }
            return id;
        }
        
        unsigned int CreateShaderProgram(const char* vsSrc, const char* fsSrc)
        {
            unsigned int vs = CompileShader(GL_VERTEX_SHADER, vsSrc);
            unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fsSrc);
            
            if (vs == 0 || fs == 0)
            {
                if (vs != 0) glDeleteShader(vs);
                if (fs != 0) glDeleteShader(fs);
                return 0;
            }
            
            unsigned int program = glCreateProgram();
            if (program == 0)
            {
                glDeleteShader(vs);
                glDeleteShader(fs);
                return 0;
            }
            
            glAttachShader(program, vs);
            glAttachShader(program, fs);
            glLinkProgram(program);
            
            int result = GL_FALSE;
            glGetProgramiv(program, GL_LINK_STATUS, &result);
            if (result == GL_FALSE)
            {
                int length = 0;
                glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
                if (length > 0)
                {
                    char* message = static_cast<char*>(alloca(static_cast<size_t>(length)));
                    glGetProgramInfoLog(program, length, &length, message);
                    printf("Failed to link shader program:\n%s\n", message);
                }
                glDeleteShader(vs);
                glDeleteShader(fs);
                glDeleteProgram(program);
                return 0;
            }
            
            glValidateProgram(program);
            glDeleteShader(vs);
            glDeleteShader(fs);
            return program;
        }
        
        void SetupShaderProgram()
        {
            const char* vertexShaderSource = R"(
                #version 460 core
                
                layout(location = 0) in vec2 a_Position;
                layout(location = 1) in vec3 a_Color;
                layout(location = 2) in vec2 a_TexCoord;
                layout(location = 3) in vec2 a_Normal;
                
                out vec3 v_Color;
                out vec2 v_TexCoord;
                out vec2 v_Normal;
                out vec2 v_WorldPos;
                
                uniform mat4 u_View;
                uniform mat4 u_Projection;
                
                void main()
                {
                    v_Color = a_Color;
                    v_TexCoord = a_TexCoord;
                    v_Normal = a_Normal;
                    v_WorldPos = a_Position;
                    gl_Position = u_Projection * u_View * vec4(a_Position, 0.0, 1.0);
                }
            )";
            
            const char* fragmentShaderSource = R"(
                #version 460 core
                
                in vec3 v_Color;
                in vec2 v_TexCoord;
                in vec2 v_Normal;
                in vec2 v_WorldPos;
                
                out vec4 fragColor;
                
                void main()
                {
                    // Use vertex color directly - no fake lighting
                    vec3 color = v_Color;
                    
                    fragColor = vec4(color, 1.0);
                }
            )";
            
            ShaderProgram = CreateShaderProgram(vertexShaderSource, fragmentShaderSource);
            if (ShaderProgram != 0)
            {
                glUseProgram(ShaderProgram);
                
                ViewUniform = glGetUniformLocation(ShaderProgram, "u_View");
                ProjectionUniform = glGetUniformLocation(ShaderProgram, "u_Projection");
                LineWidthUniform = glGetUniformLocation(ShaderProgram, "u_LineWidth");
                
                // Set default projection
                UpdateProjectionMatrix(1280.0f, 720.0f);
            }
        }
        
        void UpdateProjectionMatrix(float screenWidth, float screenHeight)
        {
            // Y-up coordinate system: bottom=0, top=screenHeight/zoom
            float left = 0.0f;
            float right = screenWidth / CurrentCamera.zoom;
            float bottom = 0.0f;
            float top = screenHeight / CurrentCamera.zoom;
            
            ProjectionMatrix = glm::ortho(left, right, bottom, top, 
                                         CurrentCamera.nearPlane, CurrentCamera.farPlane);
            
            if (ShaderProgram != 0)
            {
                glUseProgram(ShaderProgram);
                glUniformMatrix4fv(ProjectionUniform, 1, GL_FALSE, &ProjectionMatrix[0][0]);
            }
        }
        
        void UpdateViewMatrix()
        {
            ViewMatrix = CurrentCamera.GetViewMatrix();
            
            if (ShaderProgram != 0)
            {
                glUseProgram(ShaderProgram);
                glUniformMatrix4fv(ViewUniform, 1, GL_FALSE, &ViewMatrix[0][0]);
            }
        }
        
        void FlushBuffer(std::vector<Vertex>& buffer, GLenum primitiveType)
        {
            if (buffer.empty()) return;
            
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER,
                        static_cast<GLsizeiptr>(buffer.size() * sizeof(Vertex)),
                        buffer.data(),
                        GL_DYNAMIC_DRAW);
            
            glBindVertexArray(VAO);
            glDrawArrays(primitiveType, 0, static_cast<GLsizei>(buffer.size()));
            glBindVertexArray(0);
            
            buffer.clear();
        }
    };
    
    std::unique_ptr<Renderer2D::Impl> Renderer2D::s_Instance;
    
    // Thread-safety note: Renderer2D uses a raw static singleton pointer without mutex protection.
    // This implementation is NOT thread-safe and should only be called from the main render thread.
    // For multi-threaded rendering, convert to a passed-by-reference context object.
    
    // =======================================================================
    // Public API
    // =======================================================================
    
    void Renderer2D::Init()
    {
        s_Instance = std::make_unique<Impl>();
        
        s_Instance->GLAvailable = s_Instance->CheckGLFunctionsLoaded();
        
        if (!s_Instance->GLAvailable)
        {
            s_Instance->Initialized = true;
            return;
        }
        
        // Clean up existing objects
        if (s_Instance->VAO != 0) { glDeleteVertexArrays(1, &s_Instance->VAO); s_Instance->VAO = 0; }
        if (s_Instance->VBO != 0) { glDeleteBuffers(1, &s_Instance->VBO); s_Instance->VBO = 0; }
        if (s_Instance->EBO != 0) { glDeleteBuffers(1, &s_Instance->EBO); s_Instance->EBO = 0; }
        if (s_Instance->ShaderProgram != 0) { glDeleteProgram(s_Instance->ShaderProgram); s_Instance->ShaderProgram = 0; }
        
        // Create GL objects
        glGenVertexArrays(1, &s_Instance->VAO);
        glGenBuffers(1, &s_Instance->VBO);
        glGenBuffers(1, &s_Instance->EBO);
        
        if (s_Instance->VAO == 0 || s_Instance->VBO == 0)
        {
            printf("Renderer2D::Init - failed to allocate GL objects\n");
            if (s_Instance->VAO != 0) { glDeleteVertexArrays(1, &s_Instance->VAO); s_Instance->VAO = 0; }
            if (s_Instance->VBO != 0) { glDeleteBuffers(1, &s_Instance->VBO); s_Instance->VBO = 0; }
            return;
        }
        
        glBindVertexArray(s_Instance->VAO);
        glBindBuffer(GL_ARRAY_BUFFER, s_Instance->VBO);
        
        // Position attribute
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), 
                             reinterpret_cast<void*>(0));
        glEnableVertexAttribArray(0);
        
        // Color attribute
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), 
                             reinterpret_cast<void*>(offsetof(Vertex, r)));
        glEnableVertexAttribArray(1);
        
        // Texture coordinate attribute
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), 
                             reinterpret_cast<void*>(offsetof(Vertex, u)));
        glEnableVertexAttribArray(2);
        
        // Normal attribute
        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), 
                             reinterpret_cast<void*>(offsetof(Vertex, nx)));
        glEnableVertexAttribArray(3);
        
        glBindVertexArray(0);
        
        // Setup shaders
        s_Instance->SetupShaderProgram();
        
        s_Instance->Initialized = true;
    }
    
    void Renderer2D::Shutdown()
    {
        if (!s_Instance) return;
        
        if (s_Instance->GLAvailable)
        {
            if (s_Instance->VAO != 0) { glDeleteVertexArrays(1, &s_Instance->VAO); s_Instance->VAO = 0; }
            if (s_Instance->VBO != 0) { glDeleteBuffers(1, &s_Instance->VBO); s_Instance->VBO = 0; }
            if (s_Instance->EBO != 0) { glDeleteBuffers(1, &s_Instance->EBO); s_Instance->EBO = 0; }
            if (s_Instance->ShaderProgram != 0) { glDeleteProgram(s_Instance->ShaderProgram); s_Instance->ShaderProgram = 0; }
        }
        
        s_Instance->QuadBuffer.clear();
        s_Instance->LineBuffer.clear();
        s_Instance->IndexBuffer.clear();
        s_Instance->Initialized = false;
        s_Instance->GLAvailable = false;
        
        s_Instance.reset();
    }
    
    void Renderer2D::BeginScene(const Camera2D& camera)
    {
        if (!s_Instance || !s_Instance->Initialized) return;
        
        s_Instance->CurrentCamera = camera;
        s_Instance->QuadBuffer.clear();
        s_Instance->LineBuffer.clear();
        
        if (s_Instance->GLAvailable)
        {
            // Update projection matrix with current window size
            // Get window from Application singleton
            GLFWwindow* window = nullptr;
            try {
                Application& app = Application::Get();
                window = app.GetWindow();
            } catch (...) {
                // Application might not be initialized yet
                window = nullptr;
            }
            
            int w, h;
            if (window)
            {
                glfwGetFramebufferSize(window, &w, &h);
            }
            else
            {
                // Fallback: try to get from current context
                w = 1280;
                h = 720;
            }
            
            if (w > 0 && h > 0)
            {
                s_Instance->UpdateProjectionMatrix(static_cast<float>(w), static_cast<float>(h));
            }
            
            s_Instance->UpdateViewMatrix();
            
            // Enable blending by default
            if (s_Instance->BlendingEnabled)
            {
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            }
            else
            {
                glDisable(GL_BLEND);
            }
            
            // Depth test
            if (s_Instance->DepthTestEnabled)
                glEnable(GL_DEPTH_TEST);
            else
                glDisable(GL_DEPTH_TEST);
            
            // Culling
            if (s_Instance->CullingEnabled)
            {
                glEnable(GL_CULL_FACE);
                glCullFace(GL_BACK);
            }
            else
            {
                glDisable(GL_CULL_FACE);
            }
        }
    }
    
    void Renderer2D::EndScene()
    {
        Flush();
    }
    
    void Renderer2D::DrawQuad(const Math::Vector2& position,
                             const Math::Vector2& size,
                             const Math::Vector2& origin,
                             const Math::Vector3& color,
                             float rotation)
    {
        if (!s_Instance || !s_Instance->Initialized) return;
        
        if (s_Instance->QuadBuffer.size() + 6 > s_Instance->MaxVertices)
        {
            if (s_Instance->GLAvailable)
            {
                s_Instance->FlushBuffer(s_Instance->QuadBuffer, GL_TRIANGLES);
            }
            else
            {
                s_Instance->QuadBuffer.clear();
            }
        }
        
        // Build 4 corners, rotate each around position - origin, then draw 2 triangles
        float c = std::cos(rotation);
        float s = std::sin(rotation);
        
        auto rot = [&](Math::Vector2 p) -> Math::Vector2 {
            return {p.x * c - p.y * s, p.x * s + p.y * c};
        };
        
        Math::Vector2 corners[4] = {
            position + rot(Math::Vector2(-origin.x, -origin.y)),
            position + rot(Math::Vector2(size.x - origin.x, -origin.y)),
            position + rot(Math::Vector2(size.x - origin.x, size.y - origin.y)),
            position + rot(Math::Vector2(-origin.x, size.y - origin.y))
        };
        
        const float cr = color.x, cg = color.y, cb = color.z;
        
        // Two triangles, counter-clockwise winding
        s_Instance->QuadBuffer.push_back({corners[0].x, corners[0].y, cr, cg, cb});
        s_Instance->QuadBuffer.push_back({corners[1].x, corners[1].y, cr, cg, cb});
        s_Instance->QuadBuffer.push_back({corners[3].x, corners[3].y, cr, cg, cb});
        
        s_Instance->QuadBuffer.push_back({corners[1].x, corners[1].y, cr, cg, cb});
        s_Instance->QuadBuffer.push_back({corners[2].x, corners[2].y, cr, cg, cb});
        s_Instance->QuadBuffer.push_back({corners[3].x, corners[3].y, cr, cg, cb});
    }
    
    void Renderer2D::DrawCircle(const Math::Vector2& center,
                               float radius,
                               const Math::Vector3& color,
                               int segments)
    {
        if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
        if (segments < 3) segments = 3;
        
        const uint32_t neededVertices = static_cast<uint32_t>(segments) * 2;
        if (s_Instance->LineBuffer.size() + neededVertices > s_Instance->MaxVertices)
        {
            if (s_Instance->GLAvailable)
            {
                s_Instance->FlushBuffer(s_Instance->LineBuffer, GL_LINES);
            }
            else
            {
                s_Instance->LineBuffer.clear();
            }
        }
        
        const float cr = color.x, cg = color.y, cb = color.z;
        const float step = 2.0f * PI / static_cast<float>(segments);
        
        for (int i = 0; i < segments; ++i)
        {
            float angle1 = step * static_cast<float>(i);
            float angle2 = step * static_cast<float>(i + 1);
            
            Math::Vector2 point1{
                center.x + radius * std::cos(angle1),
                center.y + radius * std::sin(angle1)
            };
            Math::Vector2 point2{
                center.x + radius * std::cos(angle2),
                center.y + radius * std::sin(angle2)
            };
            
            s_Instance->LineBuffer.push_back({point1.x, point1.y, cr, cg, cb});
            s_Instance->LineBuffer.push_back({point2.x, point2.y, cr, cg, cb});
        }
    }
    
    void Renderer2D::DrawSolidCircle(const Math::Vector2& center,
                                    float radius,
                                    const Math::Vector3& color,
                                    int segments)
    {
        if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
        if (segments < 3) segments = 3;
        
        const uint32_t neededVertices = static_cast<uint32_t>(segments) * 3;
        if (s_Instance->QuadBuffer.size() + neededVertices > s_Instance->MaxVertices)
        {
            if (s_Instance->GLAvailable)
            {
                s_Instance->FlushBuffer(s_Instance->QuadBuffer, GL_TRIANGLES);
            }
            else
            {
                s_Instance->QuadBuffer.clear();
            }
        }
        
        const float cr = color.x, cg = color.y, cb = color.z;
        const float step = 2.0f * PI / static_cast<float>(segments);
        
        Math::Vector2 prevPoint{
            center.x + radius * std::cos(0.0f),
            center.y + radius * std::sin(0.0f)
        };
        
        for (int i = 1; i <= segments; ++i)
        {
            float angle = step * static_cast<float>(i);
            Math::Vector2 nextPoint{
                center.x + radius * std::cos(angle),
                center.y + radius * std::sin(angle)
            };
            
            // Triangle fan: center, next, prev
            s_Instance->QuadBuffer.push_back({center.x, center.y, cr, cg, cb});
            s_Instance->QuadBuffer.push_back({nextPoint.x, nextPoint.y, cr, cg, cb});
            s_Instance->QuadBuffer.push_back({prevPoint.x, prevPoint.y, cr, cg, cb});
            
            prevPoint = nextPoint;
        }
    }
    
    void Renderer2D::DrawPolygon(const std::vector<Math::Vector2>& vertices,
                                const Math::Vector3& color)
    {
        if (!s_Instance || !s_Instance->Initialized || vertices.size() < 3) return;
        
        const uint32_t neededVertices = static_cast<uint32_t>(vertices.size()) * 2;
        if (s_Instance->LineBuffer.size() + neededVertices > s_Instance->MaxVertices)
        {
            if (s_Instance->GLAvailable)
            {
                s_Instance->FlushBuffer(s_Instance->LineBuffer, GL_LINES);
            }
            else
            {
                s_Instance->LineBuffer.clear();
            }
        }
        
        const float cr = color.x, cg = color.y, cb = color.z;
        
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            size_t next = (i + 1) % vertices.size();
            s_Instance->LineBuffer.push_back({
                vertices[i].x, vertices[i].y, cr, cg, cb
            });
            s_Instance->LineBuffer.push_back({
                vertices[next].x, vertices[next].y, cr, cg, cb
            });
        }
    }
    
    void Renderer2D::DrawSolidPolygon(const std::vector<Math::Vector2>& vertices,
                                     const Math::Vector3& color)
    {
        if (!s_Instance || !s_Instance->Initialized || vertices.size() < 3) return;
        
#ifndef NDEBUG
        // Validate convexity - DrawSolidPolygon requires a convex polygon
        auto isConvex = [](const std::vector<Math::Vector2>& verts) -> bool {
            if (verts.size() < 3) return false;
            
            int sign = 0;
            for (size_t i = 0; i < verts.size(); ++i)
            {
                const Math::Vector2& p1 = verts[i];
                const Math::Vector2& p2 = verts[(i + 1) % verts.size()];
                const Math::Vector2& p3 = verts[(i + 2) % verts.size()];
                
                Math::Vector2 v1 = p2 - p1;
                Math::Vector2 v2 = p3 - p2;
                float cross = v1.x * v2.y - v1.y * v2.x;
                
                if (cross != 0.0f)
                {
                    if (sign == 0)
                        sign = (cross > 0.0f) ? 1 : -1;
                    else if ((cross > 0.0f ? 1 : -1) != sign)
                        return false;
                }
            }
            return true;
        };
        
        assert(isConvex(vertices) && "DrawSolidPolygon requires a convex polygon");
#endif
        
        const uint32_t neededVertices = static_cast<uint32_t>((vertices.size() - 2) * 3);
        if (s_Instance->QuadBuffer.size() + neededVertices > s_Instance->MaxVertices)
        {
            if (s_Instance->GLAvailable)
            {
                s_Instance->FlushBuffer(s_Instance->QuadBuffer, GL_TRIANGLES);
            }
            else
            {
                s_Instance->QuadBuffer.clear();
            }
        }
        
        const float cr = color.x, cg = color.y, cb = color.z;
        const Math::Vector2& v0 = vertices[0];
        
        for (size_t i = 1; i + 1 < vertices.size(); ++i)
        {
            const Math::Vector2& v1 = vertices[i];
            const Math::Vector2& v2 = vertices[i + 1];
            
            s_Instance->QuadBuffer.push_back({v0.x, v0.y, cr, cg, cb});
            s_Instance->QuadBuffer.push_back({v2.x, v2.y, cr, cg, cb});
            s_Instance->QuadBuffer.push_back({v1.x, v1.y, cr, cg, cb});
        }
    }
    
    void Renderer2D::DrawLine(const Math::Vector2& start,
                             const Math::Vector2& end,
                             const Math::Vector3& color,
                             float thickness)
    {
        if (!s_Instance || !s_Instance->Initialized) return;
        
        if (thickness > 1.0f)
        {
            // Draw thick line as a rectangle
            Math::Vector2 dir = end - start;
            float len = dir.Length();
            if (len < 0.0001f) return;
            
            Math::Vector2 normalizedDir = dir * (1.0f / len);
            Math::Vector2 perp = {-normalizedDir.y, normalizedDir.x};
            float halfThickness = thickness * 0.5f;
            
            Math::Vector2 offset = perp * halfThickness;
            
            std::vector<Math::Vector2> quadVerts = {
                start + offset,
                start - offset,
                end + offset,
                end - offset
            };
            
            DrawSolidPolygon(quadVerts, color);
        }
        else
        {
            if (s_Instance->LineBuffer.size() + 2 > s_Instance->MaxVertices)
            {
                if (s_Instance->GLAvailable)
                {
                    s_Instance->FlushBuffer(s_Instance->LineBuffer, GL_LINES);
                }
                else
                {
                    s_Instance->LineBuffer.clear();
                }
            }
            
            const float cr = color.x, cg = color.y, cb = color.z;
            s_Instance->LineBuffer.push_back({start.x, start.y, cr, cg, cb});
            s_Instance->LineBuffer.push_back({end.x, end.y, cr, cg, cb});
        }
    }
    
    void Renderer2D::DrawCapsule(const Math::Vector2& center1,
                                const Math::Vector2& center2,
                                float radius,
                                const Math::Vector3& color,
                                int segments)
    {
        if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
        if (segments < 8) segments = 8;
        
        // Calculate capsule orientation
        Math::Vector2 axis = center2 - center1;
        float length = axis.Length();
        
        if (length < 0.0001f)
        {
            // Degenerate capsule - draw circle
            DrawCircle(center1, radius, color, segments);
            return;
        }
        
        Math::Vector2 normal = axis * (1.0f / length);
        Math::Vector2 tangent = {-normal.y, normal.x};
        
        // Draw outline
        const float step = 3.1415926535f / static_cast<float>(segments);
        
        // First semicircle (around center1)
        for (int i = 0; i < segments; ++i)
        {
            float angle1 = step * static_cast<float>(i);
            float angle2 = step * static_cast<float>(i + 1);
            
            Math::Vector2 dir1 = tangent * std::cos(angle1) - normal * std::sin(angle1);
            Math::Vector2 dir2 = tangent * std::cos(angle2) - normal * std::sin(angle2);
            
            Math::Vector2 point1 = center1 + dir1 * radius;
            Math::Vector2 point2 = center1 + dir2 * radius;
            
            if (s_Instance->LineBuffer.size() + 2 > s_Instance->MaxVertices)
            {
                if (s_Instance->GLAvailable)
                    s_Instance->FlushBuffer(s_Instance->LineBuffer, GL_LINES);
                else
                    s_Instance->LineBuffer.clear();
            }
            
            const float cr = color.x, cg = color.y, cb = color.z;
            s_Instance->LineBuffer.push_back({point1.x, point1.y, cr, cg, cb});
            s_Instance->LineBuffer.push_back({point2.x, point2.y, cr, cg, cb});
        }
        
        // Second semicircle (around center2)
        for (int i = 0; i < segments; ++i)
        {
            float angle1 = step * static_cast<float>(i);
            float angle2 = step * static_cast<float>(i + 1);
            
            Math::Vector2 dir1 = tangent * std::cos(angle1) + normal * std::sin(angle1);
            Math::Vector2 dir2 = tangent * std::cos(angle2) + normal * std::sin(angle2);
            
            Math::Vector2 point1 = center2 + dir1 * radius;
            Math::Vector2 point2 = center2 + dir2 * radius;
            
            if (s_Instance->LineBuffer.size() + 2 > s_Instance->MaxVertices)
            {
                if (s_Instance->GLAvailable)
                    s_Instance->FlushBuffer(s_Instance->LineBuffer, GL_LINES);
                else
                    s_Instance->LineBuffer.clear();
            }
            
            const float cr = color.x, cg = color.y, cb = color.z;
            s_Instance->LineBuffer.push_back({point1.x, point1.y, cr, cg, cb});
            s_Instance->LineBuffer.push_back({point2.x, point2.y, cr, cg, cb});
        }
        
        // Draw connecting lines
        Math::Vector2 side1 = tangent * radius;
        Math::Vector2 side2 = -tangent * radius;
        
        DrawLine(center1 + side1, center2 + side1, color, 1.0f);
        DrawLine(center1 + side2, center2 + side2, color, 1.0f);
    }
    
    void Renderer2D::DrawSolidCapsule(const Math::Vector2& center1,
                                     const Math::Vector2& center2,
                                     float radius,
                                     const Math::Vector3& color,
                                     int segments)
    {
        if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
        if (segments < 8) segments = 8;
        
        Math::Vector2 axis = center2 - center1;
        float length = axis.Length();
        
        if (length < 0.0001f)
        {
            DrawSolidCircle(center1, radius, color, segments);
            return;
        }
        
        Math::Vector2 normal = axis * (1.0f / length);
        Math::Vector2 tangent = {-normal.y, normal.x};
        
        const float step = 3.1415926535f / static_cast<float>(segments);
        const float cr = color.x, cg = color.y, cb = color.z;
        
        // Draw first semicircle fan
        for (int i = 0; i < segments; ++i)
        {
            float angle1 = step * static_cast<float>(i);
            float angle2 = step * static_cast<float>(i + 1);
            
            Math::Vector2 dir1 = tangent * std::cos(angle1) - normal * std::sin(angle1);
            Math::Vector2 dir2 = tangent * std::cos(angle2) - normal * std::sin(angle2);
            
            Math::Vector2 point1 = center1 + dir1 * radius;
            Math::Vector2 point2 = center1 + dir2 * radius;
            
            if (s_Instance->QuadBuffer.size() + 3 > s_Instance->MaxVertices)
            {
                if (s_Instance->GLAvailable)
                    s_Instance->FlushBuffer(s_Instance->QuadBuffer, GL_TRIANGLES);
                else
                    s_Instance->QuadBuffer.clear();
            }
            
            s_Instance->QuadBuffer.push_back({center1.x, center1.y, cr, cg, cb});
            s_Instance->QuadBuffer.push_back({point2.x, point2.y, cr, cg, cb});
            s_Instance->QuadBuffer.push_back({point1.x, point1.y, cr, cg, cb});
        }
        
        // Draw second semicircle fan
        for (int i = 0; i < segments; ++i)
        {
            float angle1 = step * static_cast<float>(i);
            float angle2 = step * static_cast<float>(i + 1);
            
            Math::Vector2 dir1 = tangent * std::cos(angle1) + normal * std::sin(angle1);
            Math::Vector2 dir2 = tangent * std::cos(angle2) + normal * std::sin(angle2);
            
            Math::Vector2 point1 = center2 + dir1 * radius;
            Math::Vector2 point2 = center2 + dir2 * radius;
            
            if (s_Instance->QuadBuffer.size() + 3 > s_Instance->MaxVertices)
            {
                if (s_Instance->GLAvailable)
                    s_Instance->FlushBuffer(s_Instance->QuadBuffer, GL_TRIANGLES);
                else
                    s_Instance->QuadBuffer.clear();
            }
            
            s_Instance->QuadBuffer.push_back({center2.x, center2.y, cr, cg, cb});
            s_Instance->QuadBuffer.push_back({point2.x, point2.y, cr, cg, cb});
            s_Instance->QuadBuffer.push_back({point1.x, point1.y, cr, cg, cb});
        }
        
        // Draw rectangular body as two triangles with correct CCW winding
        Math::Vector2 side1 = tangent * radius;
        Math::Vector2 side2 = -tangent * radius;
        
        // Vertices in CCW order: top-left, top-right, bottom-right, bottom-left
        Math::Vector2 v0 = center1 + side1;  // top-left
        Math::Vector2 v1 = center2 + side1;  // top-right
        Math::Vector2 v2 = center2 + side2;  // bottom-right
        Math::Vector2 v3 = center1 + side2;  // bottom-left
        
        if (s_Instance->QuadBuffer.size() + 6 > s_Instance->MaxVertices)
        {
            if (s_Instance->GLAvailable)
                s_Instance->FlushBuffer(s_Instance->QuadBuffer, GL_TRIANGLES);
            else
                s_Instance->QuadBuffer.clear();
        }
        
        // Triangle 1: v0, v1, v2 (top-left, top-right, bottom-right)
        s_Instance->QuadBuffer.push_back({v0.x, v0.y, cr, cg, cb});
        s_Instance->QuadBuffer.push_back({v1.x, v1.y, cr, cg, cb});
        s_Instance->QuadBuffer.push_back({v2.x, v2.y, cr, cg, cb});
        
        // Triangle 2: v0, v2, v3 (top-left, bottom-right, bottom-left)
        s_Instance->QuadBuffer.push_back({v0.x, v0.y, cr, cg, cb});
        s_Instance->QuadBuffer.push_back({v2.x, v2.y, cr, cg, cb});
        s_Instance->QuadBuffer.push_back({v3.x, v3.y, cr, cg, cb});
    }
    
    void Renderer2D::DrawSegment(const Math::Vector2& point1,
                                const Math::Vector2& point2,
                                float thickness,
                                const Math::Vector3& color)
    {
        if (!s_Instance || !s_Instance->Initialized) return;
        
        if (thickness <= 0.0f)
        {
            DrawLine(point1, point2, color, 1.0f);
        }
        else
        {
            // Draw as rectangle with rounded ends
            Math::Vector2 dir = point2 - point1;
            float len = dir.Length();
            
            if (len < 0.0001f)
            {
                DrawSolidCircle(point1, thickness * 0.5f, color, 16);
                return;
            }
            
            Math::Vector2 normal = dir * (1.0f / len);
            Math::Vector2 tangent = {-normal.y, normal.x};
            float halfThickness = thickness * 0.5f;
            
            // Main rectangle
            std::vector<Math::Vector2> rectVerts = {
                point1 + tangent * halfThickness,
                point1 - tangent * halfThickness,
                point2 + tangent * halfThickness,
                point2 - tangent * halfThickness
            };
            
            DrawSolidPolygon(rectVerts, color);
            
            // End circles
            DrawSolidCircle(point1, halfThickness, color, 16);
            DrawSolidCircle(point2, halfThickness, color, 16);
        }
    }
    
    void Renderer2D::DrawChain(const std::vector<Math::Vector2>& vertices,
                              const Math::Vector3& color,
                              float thickness,
                              bool closed)
    {
        if (!s_Instance || !s_Instance->Initialized || vertices.size() < 2) return;
        
        for (size_t i = 0; i < vertices.size() - 1; ++i)
        {
            DrawSegment(vertices[i], vertices[i + 1], thickness, color);
        }
        
        if (closed && vertices.size() > 2)
        {
            DrawSegment(vertices.back(), vertices.front(), thickness, color);
        }
    }
    
    void Renderer2D::DrawEllipse(const Math::Vector2& center,
                                float radiusX,
                                float radiusY,
                                const Math::Vector3& color,
                                int segments)
    {
        if (!s_Instance || !s_Instance->Initialized) return;
        if (segments < 8) segments = 8;
        
        const float step = 2.0f * PI / static_cast<float>(segments);
        const float cr = color.x, cg = color.y, cb = color.z;
        
        // Draw outline
        for (int i = 0; i < segments; ++i)
        {
            float angle1 = step * static_cast<float>(i);
            float angle2 = step * static_cast<float>(i + 1);
            
            Math::Vector2 point1{
                center.x + radiusX * std::cos(angle1),
                center.y + radiusY * std::sin(angle1)
            };
            Math::Vector2 point2{
                center.x + radiusX * std::cos(angle2),
                center.y + radiusY * std::sin(angle2)
            };
            
            if (s_Instance->LineBuffer.size() + 2 > s_Instance->MaxVertices)
            {
                if (s_Instance->GLAvailable)
                    s_Instance->FlushBuffer(s_Instance->LineBuffer, GL_LINES);
                else
                    s_Instance->LineBuffer.clear();
            }
            
            s_Instance->LineBuffer.push_back({point1.x, point1.y, cr, cg, cb});
            s_Instance->LineBuffer.push_back({point2.x, point2.y, cr, cg, cb});
        }
    }
    
    void Renderer2D::DrawSolidEllipse(const Math::Vector2& center,
                                     float radiusX,
                                     float radiusY,
                                     const Math::Vector3& color,
                                     int segments)
    {
        if (!s_Instance || !s_Instance->Initialized) return;
        if (segments < 8) segments = 8;
        
        const float step = 2.0f * PI / static_cast<float>(segments);
        const float cr = color.x, cg = color.y, cb = color.z;
        
        Math::Vector2 prevPoint{
            center.x + radiusX,
            center.y
        };
        
        for (int i = 1; i <= segments; ++i)
        {
            float angle = step * static_cast<float>(i);
            Math::Vector2 nextPoint{
                center.x + radiusX * std::cos(angle),
                center.y + radiusY * std::sin(angle)
            };
            
            if (s_Instance->QuadBuffer.size() + 3 > s_Instance->MaxVertices)
            {
                if (s_Instance->GLAvailable)
                    s_Instance->FlushBuffer(s_Instance->QuadBuffer, GL_TRIANGLES);
                else
                    s_Instance->QuadBuffer.clear();
            }
            
            s_Instance->QuadBuffer.push_back({center.x, center.y, cr, cg, cb});
            s_Instance->QuadBuffer.push_back({nextPoint.x, nextPoint.y, cr, cg, cb});
            s_Instance->QuadBuffer.push_back({prevPoint.x, prevPoint.y, cr, cg, cb});
            
            prevPoint = nextPoint;
        }
    }
    
    void Renderer2D::DrawArc(const Math::Vector2& center,
                            float radius,
                            float angleStart,
                            float angleEnd,
                            const Math::Vector3& color,
                            float thickness,
                            int segments)
    {
        if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
        if (segments < 8) segments = 8;
        
        float angleRange = angleEnd - angleStart;
        if (std::abs(angleRange) < 0.0001f) return;
        
        const float step = angleRange / static_cast<float>(segments);
        const float cr = color.x, cg = color.y, cb = color.z;
        
        for (int i = 0; i < segments; ++i)
        {
            float angle1 = angleStart + step * static_cast<float>(i);
            float angle2 = angleStart + step * static_cast<float>(i + 1);
            
            Math::Vector2 point1{
                center.x + radius * std::cos(angle1),
                center.y + radius * std::sin(angle1)
            };
            Math::Vector2 point2{
                center.x + radius * std::cos(angle2),
                center.y + radius * std::sin(angle2)
            };
            
            if (thickness > 1.0f)
            {
                DrawLine(point1, point2, color, thickness);
            }
            else
            {
                if (s_Instance->LineBuffer.size() + 2 > s_Instance->MaxVertices)
                {
                    if (s_Instance->GLAvailable)
                        s_Instance->FlushBuffer(s_Instance->LineBuffer, GL_LINES);
                    else
                        s_Instance->LineBuffer.clear();
                }
                
                s_Instance->LineBuffer.push_back({point1.x, point1.y, cr, cg, cb});
                s_Instance->LineBuffer.push_back({point2.x, point2.y, cr, cg, cb});
            }
        }
    }
    
    void Renderer2D::DrawSector(const Math::Vector2& center,
                               float radius,
                               float angleStart,
                               float angleEnd,
                               const Math::Vector3& color,
                               int segments)
    {
        if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
        if (segments < 8) segments = 8;
        
        float angleRange = angleEnd - angleStart;
        if (std::abs(angleRange) < 0.0001f) return;
        
        const float step = angleRange / static_cast<float>(segments);
        const float cr = color.x, cg = color.y, cb = color.z;
        
        // Outline version: draw two radial lines + arc
        Math::Vector2 startPoint{
            center.x + radius * std::cos(angleStart),
            center.y + radius * std::sin(angleStart)
        };
        Math::Vector2 endPoint{
            center.x + radius * std::cos(angleEnd),
            center.y + radius * std::sin(angleEnd)
        };
        
        // Draw two radial lines
        DrawLine(center, startPoint, color, 1.0f);
        DrawLine(center, endPoint, color, 1.0f);
        
        // Draw arc
        for (int i = 0; i < segments; ++i)
        {
            float angle1 = angleStart + step * static_cast<float>(i);
            float angle2 = angleStart + step * static_cast<float>(i + 1);
            
            Math::Vector2 point1{
                center.x + radius * std::cos(angle1),
                center.y + radius * std::sin(angle1)
            };
            Math::Vector2 point2{
                center.x + radius * std::cos(angle2),
                center.y + radius * std::sin(angle2)
            };
            
            if (s_Instance->LineBuffer.size() + 2 > s_Instance->MaxVertices)
            {
                if (s_Instance->GLAvailable)
                    s_Instance->FlushBuffer(s_Instance->LineBuffer, GL_LINES);
                else
                    s_Instance->LineBuffer.clear();
            }
            
            s_Instance->LineBuffer.push_back({point1.x, point1.y, cr, cg, cb});
            s_Instance->LineBuffer.push_back({point2.x, point2.y, cr, cg, cb});
        }
    }
    
    void Renderer2D::DrawSolidSector(const Math::Vector2& center,
                                    float radius,
                                    float angleStart,
                                    float angleEnd,
                                    const Math::Vector3& color,
                                    int segments)
    {
        if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
        if (segments < 8) segments = 8;
        
        float angleRange = angleEnd - angleStart;
        if (std::abs(angleRange) < 0.0001f) return;
        
        const float step = angleRange / static_cast<float>(segments);
        const float cr = color.x, cg = color.y, cb = color.z;
        
        // Filled version: triangle fan from center
        Math::Vector2 prevPoint{
            center.x + radius * std::cos(angleStart),
            center.y + radius * std::sin(angleStart)
        };
        
        for (int i = 1; i <= segments; ++i)
        {
            float angle = angleStart + step * static_cast<float>(i);
            Math::Vector2 nextPoint{
                center.x + radius * std::cos(angle),
                center.y + radius * std::sin(angle)
            };
            
            if (s_Instance->QuadBuffer.size() + 3 > s_Instance->MaxVertices)
            {
                if (s_Instance->GLAvailable)
                    s_Instance->FlushBuffer(s_Instance->QuadBuffer, GL_TRIANGLES);
                else
                    s_Instance->QuadBuffer.clear();
            }
            
            s_Instance->QuadBuffer.push_back({center.x, center.y, cr, cg, cb});
            s_Instance->QuadBuffer.push_back({nextPoint.x, nextPoint.y, cr, cg, cb});
            s_Instance->QuadBuffer.push_back({prevPoint.x, prevPoint.y, cr, cg, cb});
            
            prevPoint = nextPoint;
        }
    }
    
    void Renderer2D::DrawShape(const ShapeDescriptor& shape)
    {
        if (!s_Instance || !s_Instance->Initialized) return;
        
        switch (shape.type)
        {
            case ShapeType::Circle:
                if (shape.filled)
                    DrawSolidCircle(shape.position, shape.params.circle.radius, shape.color, shape.segments);
                else
                    DrawCircle(shape.position, shape.params.circle.radius, shape.color, shape.segments);
                break;
                
            case ShapeType::Polygon:
                if (shape.filled)
                    DrawSolidPolygon(shape.vertices, shape.color);
                else
                    DrawPolygon(shape.vertices, shape.color);
                break;
                
            case ShapeType::Rectangle:
            {
                Math::Vector2 origin = {
                    shape.params.rect.width * 0.5f,
                    shape.params.rect.height * 0.5f
                };
                Math::Vector2 size = {shape.params.rect.width, shape.params.rect.height};
                DrawQuad(shape.position, size, origin, shape.color);
                break;
            }
                
            case ShapeType::Ellipse:
                if (shape.filled)
                    DrawSolidEllipse(shape.position, shape.params.ellipse.radiusX, shape.params.ellipse.radiusY, shape.color, shape.segments);
                else
                    DrawEllipse(shape.position, shape.params.ellipse.radiusX, shape.params.ellipse.radiusY, shape.color, shape.segments);
                break;
                
            case ShapeType::Capsule:
                if (shape.filled)
                {
                    // Extract centers from params
                    Math::Vector2 c1 = {shape.params.segment.startX, shape.params.segment.startY};
                    Math::Vector2 c2 = {shape.params.segment.endX, shape.params.segment.endY};
                    DrawSolidCapsule(c1, c2, shape.thickness, shape.color, shape.segments);
                }
                else
                {
                    Math::Vector2 c1 = {shape.params.segment.startX, shape.params.segment.startY};
                    Math::Vector2 c2 = {shape.params.segment.endX, shape.params.segment.endY};
                    DrawCapsule(c1, c2, shape.thickness, shape.color, shape.segments);
                }
                break;
                
            case ShapeType::Segment:
            {
                Math::Vector2 p1 = {shape.params.segment.startX, shape.params.segment.startY};
                Math::Vector2 p2 = {shape.params.segment.endX, shape.params.segment.endY};
                DrawSegment(p1, p2, shape.thickness, shape.color);
                break;
            }
                
            case ShapeType::Chain:
                DrawChain(shape.vertices, shape.color, shape.thickness, true);
                break;
                
            case ShapeType::Arc:
                DrawArc(shape.position, shape.params.arc.radius, 
                       shape.params.arc.angleStart, shape.params.arc.angleEnd,
                       shape.color, shape.thickness, shape.segments);
                break;
                
            case ShapeType::Sector:
                if (shape.filled)
                    DrawSolidSector(shape.position, shape.params.sector.outerRadius,
                                   shape.params.sector.angleStart, shape.params.sector.angleEnd,
                                   shape.color, shape.segments);
                else
                    DrawArc(shape.position, shape.params.sector.outerRadius,
                           shape.params.sector.angleStart, shape.params.sector.angleEnd,
                           shape.color, shape.thickness, shape.segments);
                break;
        }
    }
    
    void Renderer2D::DrawManifold(const Math::Vector2& contactPoint,
                                 const Math::Vector2& normal,
                                 float separation,
                                 bool isTouching)
    {
        if (!s_Instance || !s_Instance->Initialized) return;
        
        // Draw contact point
        Math::Vector3 pointColor = isTouching ? Math::Vector3{1.0f, 0.0f, 0.0f} : Math::Vector3{1.0f, 1.0f, 0.0f};
        DrawSolidCircle(contactPoint, 3.0f, pointColor, 16);
        
        // Draw normal vector
        Math::Vector2 normalEnd = contactPoint + normal * 20.0f;
        DrawLine(contactPoint, normalEnd, Math::Vector3{0.0f, 1.0f, 0.0f}, 2.0f);
        
        // Draw arrowhead
        float arrowSize = 5.0f;
        Math::Vector2 arrowBase = normalEnd - normal * arrowSize;
        Math::Vector2 perp = {-normal.y, normal.x};
        
        Math::Vector2 arrowLeft = arrowBase + perp * (arrowSize * 0.5f);
        Math::Vector2 arrowRight = arrowBase - perp * (arrowSize * 0.5f);
        
        DrawLine(normalEnd, arrowLeft, Math::Vector3{0.0f, 1.0f, 0.0f}, 2.0f);
        DrawLine(normalEnd, arrowRight, Math::Vector3{0.0f, 1.0f, 0.0f}, 2.0f);
    }
    
    void Renderer2D::DrawAABB(const Math::Vector2& min,
                             const Math::Vector2& max,
                             const Math::Vector3& color)
    {
        if (!s_Instance || !s_Instance->Initialized) return;
        
        std::vector<Math::Vector2> verts = {
            min,
            {max.x, min.y},
            max,
            {min.x, max.y}
        };
        
        DrawPolygon(verts, color);
    }
    
    void Renderer2D::DrawTransform(const Math::Vector2& position,
                                  float rotation,
                                  float scale)
    {
        if (!s_Instance || !s_Instance->Initialized) return;
        
        // Draw coordinate axes
        float axisLength = 50.0f * scale;
        
        Math::Vector2 xAxis{std::cos(rotation), std::sin(rotation)};
        Math::Vector2 yAxis{-std::sin(rotation), std::cos(rotation)};
        
        DrawLine(position, position + xAxis * axisLength, Math::Vector3{1.0f, 0.0f, 0.0f}, 2.0f);
        DrawLine(position, position + yAxis * axisLength, Math::Vector3{0.0f, 1.0f, 0.0f}, 2.0f);
    }
    
    void Renderer2D::Flush()
    {
        if (!s_Instance || !s_Instance->Initialized || !s_Instance->GLAvailable) return;
        if (s_Instance->VAO == 0 || s_Instance->VBO == 0) return;
        
        if (s_Instance->QuadBuffer.empty() && s_Instance->LineBuffer.empty()) return;
        
        if (s_Instance->ShaderProgram != 0)
        {
            glUseProgram(s_Instance->ShaderProgram);
            
            // Update uniforms if needed
            if (s_Instance->LineWidthUniform != -1)
            {
                glUniform1f(s_Instance->LineWidthUniform, s_Instance->CurrentLineWidth);
            }
        }
        
        s_Instance->FlushBuffer(s_Instance->QuadBuffer, GL_TRIANGLES);
        s_Instance->FlushBuffer(s_Instance->LineBuffer, GL_LINES);
    }
    
    void Renderer2D::SetLineWidth(float width)
    {
        if (!s_Instance || !s_Instance->Initialized) return;
        
        s_Instance->CurrentLineWidth = width;
        
        if (s_Instance->GLAvailable)
        {
            glLineWidth(width);
        }
    }
    
    float Renderer2D::GetLineWidth()
    {
        if (!s_Instance) return 1.0f;
        return s_Instance->CurrentLineWidth;
    }
    
    void Renderer2D::EnableBlending(bool enable)
    {
        if (!s_Instance) return;
        s_Instance->BlendingEnabled = enable;
    }
    
    void Renderer2D::EnableDepthTest(bool enable)
    {
        if (!s_Instance) return;
        s_Instance->DepthTestEnabled = enable;
    }
    
    void Renderer2D::EnableCulling(bool enable)
    {
        if (!s_Instance) return;
        s_Instance->CullingEnabled = enable;
    }
    
} // namespace Nyon::Graphics
