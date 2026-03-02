#include "nyon/graphics/Renderer2D.h"
#include <glad/glad.h>
#include <vector>
#include <cstdio>

namespace Nyon::Graphics
{
    static std::vector<Vertex> s_QuadVertexBuffer;
    static std::vector<Vertex> s_LineVertexBuffer;

    static unsigned int s_VAO           = 0;
    static unsigned int s_VBO           = 0;
    static unsigned int s_ShaderProgram = 0;
    static bool         s_Initialized   = false;
    static bool         s_GLAvailable   = false; // true only if GL function pointers loaded

    static const uint32_t MaxQuads    = 10000;
    static const uint32_t MaxVertices = MaxQuads * 6;

    // -----------------------------------------------------------------------
    // GL availability check
    // -----------------------------------------------------------------------
    // Every single GL entry point used in this file is listed here.
    // If ANY of them is null, we have no usable GL context and every GL call
    // below would segfault through a null function pointer.  We check once in
    // Init() and store the result in s_GLAvailable.  All public functions
    // bail out immediately when s_GLAvailable is false.
    static bool CheckGLFunctionsLoaded()
    {
        return glGenVertexArrays        != nullptr
            && glDeleteVertexArrays     != nullptr
            && glBindVertexArray        != nullptr
            && glGenBuffers             != nullptr
            && glDeleteBuffers          != nullptr
            && glBindBuffer             != nullptr
            && glBufferData             != nullptr
            && glVertexAttribPointer    != nullptr
            && glEnableVertexAttribArray!= nullptr
            && glCreateShader           != nullptr
            && glShaderSource           != nullptr
            && glCompileShader          != nullptr
            && glGetShaderiv            != nullptr
            && glGetShaderInfoLog       != nullptr
            && glDeleteShader           != nullptr
            && glCreateProgram          != nullptr
            && glAttachShader           != nullptr
            && glLinkProgram            != nullptr
            && glGetProgramiv           != nullptr
            && glGetProgramInfoLog      != nullptr
            && glValidateProgram        != nullptr
            && glDeleteProgram          != nullptr
            && glUseProgram             != nullptr
            && glGetUniformLocation     != nullptr
            && glUniformMatrix4fv       != nullptr
            && glDrawArrays             != nullptr;
    }

    // -----------------------------------------------------------------------
    // Shader sources
    // -----------------------------------------------------------------------
    static const char* s_VertexShaderSource = R"(
        #version 460 core
        layout(location = 0) in vec2 position;
        layout(location = 1) in vec3 color;
        uniform mat4 projection;
        uniform mat4 view;
        out vec3 v_Color;
        void main()
        {
            gl_Position = projection * view * vec4(position, 0.0, 1.0);
            v_Color = color;
        }
    )";

    static const char* s_FragmentShaderSource = R"(
        #version 460 core
        in vec3 v_Color;
        out vec4 fragColor;
        void main()
        {
            fragColor = vec4(v_Color, 1.0);
        }
    )";

    // -----------------------------------------------------------------------
    // Internal helpers — only called after s_GLAvailable is confirmed true
    // -----------------------------------------------------------------------
    static unsigned int CompileShader(unsigned int type, const char* source)
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

    static unsigned int CreateShaderProgram(const char* vsSrc, const char* fsSrc)
    {
        unsigned int vs = CompileShader(GL_VERTEX_SHADER,   vsSrc);
        unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fsSrc);

        // If either shader failed to compile, clean up and return 0.
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

    // Bind shader and upload projection/view uniforms.
    // Only called from within Flush() which already guards on s_GLAvailable.
    static void BindShaderWithUniforms()
    {
        if (s_ShaderProgram == 0) return;

        glUseProgram(s_ShaderProgram);

        const float l =    0.0f;
        const float r = 1280.0f;
        const float b =  720.0f;
        const float t =    0.0f;
        const float n =   -1.0f;
        const float f =    1.0f;

        // Column-major orthographic projection matrix
        float proj[16] = {
             2.0f / (r - l),         0.0f,                    0.0f,                  0.0f,
             0.0f,                   2.0f / (t - b),          0.0f,                  0.0f,
             0.0f,                   0.0f,                   -2.0f / (f - n),        0.0f,
            -(r + l) / (r - l),    -(t + b) / (t - b),      -(f + n) / (f - n),     1.0f
        };

        int projLoc = glGetUniformLocation(s_ShaderProgram, "projection");
        if (projLoc != -1)
            glUniformMatrix4fv(projLoc, 1, GL_FALSE, proj);

        // Identity view matrix
        float view[16] = {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        };

        int viewLoc = glGetUniformLocation(s_ShaderProgram, "view");
        if (viewLoc != -1)
            glUniformMatrix4fv(viewLoc, 1, GL_FALSE, view);
    }

    // Submit one typed buffer to the GPU, then clear it.
    // Only called from within Flush() which already guards on s_GLAvailable.
    static void FlushBuffer(std::vector<Vertex>& buffer, GLenum primitiveType)
    {
        if (buffer.empty()) return;

        glBindBuffer(GL_ARRAY_BUFFER, s_VBO);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(buffer.size() * sizeof(Vertex)),
                     buffer.data(),
                     GL_DYNAMIC_DRAW);

        glBindVertexArray(s_VAO);
        glDrawArrays(primitiveType, 0, static_cast<GLsizei>(buffer.size()));
        glBindVertexArray(0);

        buffer.clear();
    }

    // -----------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------

    void Renderer2D::Init()
    {
        // ------------------------------------------------------------------
        // SEGFAULT SOURCE 1: calling any GL function when GLAD hasn't loaded
        // the function pointers (no active GL context).  Check every pointer
        // before touching the driver at all.
        // ------------------------------------------------------------------
        s_GLAvailable = CheckGLFunctionsLoaded();

        if (!s_GLAvailable)
        {
            // No GL context — mark as "initialised" for the state machine so
            // that BeginScene/EndScene/Draw* all return silently instead of
            // crashing.  Flush() also checks s_GLAvailable and bails out.
            s_Initialized = true;
            return;
        }

        // ------------------------------------------------------------------
        // SEGFAULT SOURCE 2: Double_Init() — calling Init() twice without
        // Shutdown() in between leaks the old VAO/VBO/shader and leaves stale
        // IDs.  Delete any existing objects first.
        // ------------------------------------------------------------------
        if (s_VAO != 0)           { glDeleteVertexArrays(1, &s_VAO); s_VAO = 0; }
        if (s_VBO != 0)           { glDeleteBuffers(1, &s_VBO);      s_VBO = 0; }
        if (s_ShaderProgram != 0) { glDeleteProgram(s_ShaderProgram); s_ShaderProgram = 0; }

        // Create VAO + VBO
        glGenVertexArrays(1, &s_VAO);
        glGenBuffers(1, &s_VBO);

        // ------------------------------------------------------------------
        // SEGFAULT SOURCE 3: using a null/zero VAO or VBO in later draw calls.
        // If GL object creation failed (driver out of resources) the IDs stay
        // 0 and every subsequent bind would invoke undefined behaviour.
        // ------------------------------------------------------------------
        if (s_VAO == 0 || s_VBO == 0)
        {
            printf("Renderer2D::Init — failed to allocate GL objects\n");
            if (s_VAO != 0) { glDeleteVertexArrays(1, &s_VAO); s_VAO = 0; }
            if (s_VBO != 0) { glDeleteBuffers(1, &s_VBO);      s_VBO = 0; }
            // Leave s_Initialized false so all draw calls become no-ops.
            return;
        }

        glBindVertexArray(s_VAO);
        glBindBuffer(GL_ARRAY_BUFFER, s_VBO);

        // Position: vec2 at offset 0
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE,
                              sizeof(Vertex), reinterpret_cast<void*>(0));
        glEnableVertexAttribArray(0);

        // Color: vec3 at offsetof(Vertex, r)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                              sizeof(Vertex),
                              reinterpret_cast<void*>(offsetof(Vertex, r)));
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);

        // Build shader — result may be 0 if compilation/linking fails.
        // Draw calls check s_ShaderProgram != 0 before calling glUseProgram.
        s_ShaderProgram = CreateShaderProgram(s_VertexShaderSource,
                                               s_FragmentShaderSource);

        s_Initialized = true;
    }

    void Renderer2D::Shutdown()
    {
        // ------------------------------------------------------------------
        // SEGFAULT SOURCE 4: Double_Shutdown() — calling Shutdown() twice.
        // Without zeroing IDs after deletion the second call tries to delete
        // already-freed objects, which is undefined behaviour.
        // The != 0 guards below make every extra Shutdown() a safe no-op.
        // ------------------------------------------------------------------

        if (s_GLAvailable)
        {
            if (s_VAO != 0)
            {
                glDeleteVertexArrays(1, &s_VAO);
                s_VAO = 0;
            }
            if (s_VBO != 0)
            {
                glDeleteBuffers(1, &s_VBO);
                s_VBO = 0;
            }
            if (s_ShaderProgram != 0)
            {
                glDeleteProgram(s_ShaderProgram);
                s_ShaderProgram = 0;
            }
        }

        s_QuadVertexBuffer.clear();
        s_LineVertexBuffer.clear();
        s_Initialized   = false;
        s_GLAvailable   = false;
    }

    void Renderer2D::BeginScene()
    {
        // Safe even without a GL context — just clears CPU-side vectors.
        s_QuadVertexBuffer.clear();
        s_LineVertexBuffer.clear();
    }

    void Renderer2D::EndScene()
    {
        Flush();
    }

    void Renderer2D::DrawQuad(const Math::Vector2& position,
                               const Math::Vector2& size,
                               const Math::Vector2& origin,
                               const Math::Vector3& color)
    {
        // ------------------------------------------------------------------
        // SEGFAULT SOURCE 5: DrawQuad called before Init() or when GL is
        // unavailable.  Both cases are covered by this single guard.
        // ------------------------------------------------------------------
        if (!s_Initialized) return;

        // Auto-flush if the quad buffer would overflow MaxVertices.
        if (s_QuadVertexBuffer.size() + 6 > MaxVertices)
        {
            if (s_GLAvailable)
            {
                BindShaderWithUniforms();
                FlushBuffer(s_QuadVertexBuffer, GL_TRIANGLES);
            }
            else
            {
                s_QuadVertexBuffer.clear();
            }
        }

        const float x = position.x - origin.x;
        const float y = position.y - origin.y;
        const float w = size.x;
        const float h = size.y;
        const float cr = color.x, cg = color.y, cb = color.z;

        // Two triangles, six vertices, wound counter-clockwise
        s_QuadVertexBuffer.push_back({ x,     y + h, cr, cg, cb }); // tri1 BL
        s_QuadVertexBuffer.push_back({ x + w, y,     cr, cg, cb }); // tri1 TR
        s_QuadVertexBuffer.push_back({ x,     y,     cr, cg, cb }); // tri1 TL

        s_QuadVertexBuffer.push_back({ x,     y + h, cr, cg, cb }); // tri2 BL
        s_QuadVertexBuffer.push_back({ x + w, y + h, cr, cg, cb }); // tri2 BR
        s_QuadVertexBuffer.push_back({ x + w, y,     cr, cg, cb }); // tri2 TR
    }

    void Renderer2D::DrawLine(const Math::Vector2& start,
                               const Math::Vector2& end,
                               const Math::Vector3& color)
    {
        // ------------------------------------------------------------------
        // SEGFAULT SOURCE 6: same as DrawQuad — must guard on s_Initialized.
        // ------------------------------------------------------------------
        if (!s_Initialized) return;

        // Auto-flush if the line buffer would overflow MaxVertices.
        if (s_LineVertexBuffer.size() + 2 > MaxVertices)
        {
            if (s_GLAvailable)
            {
                BindShaderWithUniforms();
                FlushBuffer(s_LineVertexBuffer, GL_LINES);
            }
            else
            {
                s_LineVertexBuffer.clear();
            }
        }

        const float cr = color.x, cg = color.y, cb = color.z;
        s_LineVertexBuffer.push_back({ start.x, start.y, cr, cg, cb });
        s_LineVertexBuffer.push_back({ end.x,   end.y,   cr, cg, cb });
    }

    void Renderer2D::Flush()
    {
        // ------------------------------------------------------------------
        // SEGFAULT SOURCE 7: Flush() called with no GL context.
        // This is the original crash — EndScene() → Flush() → glBufferData()
        // with null function pointers = SIGSEGV.
        //
        // Also covers:
        //   • Flush() called before Init()           (s_Initialized false)
        //   • Flush() called after Shutdown()        (s_Initialized false)
        //   • Flush() when GL context never loaded   (s_GLAvailable false)
        //   • s_VAO/s_VBO somehow 0 after Init()     (object creation failed)
        //   • s_ShaderProgram 0 (shader compile fail) — BindShaderWithUniforms
        //     already guards on this, but we skip the whole path for clarity.
        // ------------------------------------------------------------------
        if (!s_Initialized || !s_GLAvailable) return;
        if (s_VAO == 0 || s_VBO == 0)        return;

        if (s_QuadVertexBuffer.empty() && s_LineVertexBuffer.empty()) return;

        BindShaderWithUniforms();

        // Quads and lines require different primitive types — separate calls.
        FlushBuffer(s_QuadVertexBuffer, GL_TRIANGLES);
        FlushBuffer(s_LineVertexBuffer, GL_LINES);
    }

} // namespace Nyon::Graphics