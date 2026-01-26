#include "nyon/graphics/Renderer2D.h"
#include <glad/glad.h>
#include <vector>
#include <cstdio>

namespace Nyon::Graphics
{
    static std::vector<Vertex> s_VertexBuffer;
    static unsigned int s_VAO = 0;
    static unsigned int s_VBO = 0;
    static bool s_RendererInitialized = false;

    // Simple shader sources
    const char* vertexShaderSource = R"(
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

    const char* fragmentShaderSource = R"(
        #version 460 core
        in vec3 v_Color;
        out vec4 fragColor;
        void main()
        {
            fragColor = vec4(v_Color, 1.0);
        }
    )";

    static unsigned int CompileShader(unsigned int type, const char* source)
    {
        unsigned int id = glCreateShader(type);
        glShaderSource(id, 1, &source, nullptr);
        glCompileShader(id);

        int result;
        glGetShaderiv(id, GL_COMPILE_STATUS, &result);
        if (result == GL_FALSE)
        {
            int length;
            glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
            char* message = (char*)alloca(length * sizeof(char));
            glGetShaderInfoLog(id, length, &length, message);
            printf("Failed to compile %s shader!\n", type == GL_VERTEX_SHADER ? "vertex" : "fragment");
            printf("%s\n", message);
            glDeleteShader(id);
            return 0;
        }

        return id;
    }

    static unsigned int CreateShader(const char* vertexShader, const char* fragmentShader)
    {
        unsigned int program = glCreateProgram();
        unsigned int vs = CompileShader(GL_VERTEX_SHADER, vertexShader);
        unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fragmentShader);

        glAttachShader(program, vs);
        glAttachShader(program, fs);
        glLinkProgram(program);

        int result;
        glGetProgramiv(program, GL_LINK_STATUS, &result);
        if (result == GL_FALSE)
        {
            int length;
            glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
            char* message = (char*)alloca(length * sizeof(char));
            glGetProgramInfoLog(program, length, &length, message);
            printf("Failed to link shader program!\n");
            printf("%s\n", message);
            glDeleteProgram(program);
            return 0;
        }

        glValidateProgram(program);

        glDeleteShader(vs);
        glDeleteShader(fs);

        return program;
    }

    void Renderer2D::Init()
    {
        // Create buffers
        glGenVertexArrays(1, &s_VAO);
        glGenBuffers(1, &s_VBO);

        glBindVertexArray(s_VAO);
        glBindBuffer(GL_ARRAY_BUFFER, s_VBO);

        // Position attribute
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
        glEnableVertexAttribArray(0);

        // Color attribute
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, r));
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);

        s_RendererInitialized = true;
    }

    void Renderer2D::Shutdown()
    {
        if (s_VAO != 0) glDeleteVertexArrays(1, &s_VAO);
        if (s_VBO != 0) glDeleteBuffers(1, &s_VBO);
        s_RendererInitialized = false;
    }

    void Renderer2D::BeginScene()
    {
        s_VertexBuffer.clear();
    }

    void Renderer2D::EndScene()
    {
        Flush();
    }

    void Renderer2D::DrawQuad(const Math::Vector2& position, const Math::Vector2& size, const Math::Vector2& origin, const Math::Vector3& color)
    {
        // Calculate quad vertices relative to position and origin
        float x = position.x - origin.x;
        float y = position.y - origin.y;
        float w = size.x;
        float h = size.y;

        // Define the four corners of the quad
        Vertex vertices[6];

        // Triangle 1
        vertices[0] = { x, y + h, color.x, color.y, color.z };      // Bottom-left
        vertices[1] = { x + w, y, color.x, color.y, color.z };     // Top-right
        vertices[2] = { x, y, color.x, color.y, color.z };         // Top-left

        // Triangle 2
        vertices[3] = { x, y + h, color.x, color.y, color.z };     // Bottom-left
        vertices[4] = { x + w, y + h, color.x, color.y, color.z }; // Bottom-right
        vertices[5] = { x + w, y, color.x, color.y, color.z };     // Top-right

        // Add vertices to buffer
        for (int i = 0; i < 6; i++)
        {
            s_VertexBuffer.push_back(vertices[i]);
        }
    }

    void Renderer2D::DrawLine(const Math::Vector2& start, const Math::Vector2& end, const Math::Vector3& color)
    {
        Vertex vertices[2];
        vertices[0] = { start.x, start.y, color.x, color.y, color.z };
        vertices[1] = { end.x, end.y, color.x, color.y, color.z };

        for (int i = 0; i < 2; i++)
        {
            s_VertexBuffer.push_back(vertices[i]);
        }
    }

    void Renderer2D::Flush()
    {
        if (s_VertexBuffer.empty()) return;

        // Bind buffers and upload vertex data
        glBindBuffer(GL_ARRAY_BUFFER, s_VBO);
        glBufferData(GL_ARRAY_BUFFER, s_VertexBuffer.size() * sizeof(Vertex), s_VertexBuffer.data(), GL_DYNAMIC_DRAW);

        // Use shader program (simple shader for now)
        static unsigned int shaderProgram = 0;
        if (shaderProgram == 0)
        {
            shaderProgram = CreateShader(vertexShaderSource, fragmentShaderSource);
        }

        glUseProgram(shaderProgram);

        // Set up orthographic projection matrix
        float left = 0.0f;
        float right = 1280.0f;
        float bottom = 720.0f;
        float top = 0.0f;
        float near = -1.0f;
        float far = 1.0f;

        float projectionMatrix[16] = {
            2.0f / (right - left), 0.0f, 0.0f, 0.0f,
            0.0f, 2.0f / (top - bottom), 0.0f, 0.0f,
            0.0f, 0.0f, -2.0f / (far - near), 0.0f,
            -(right + left) / (right - left), -(top + bottom) / (top - bottom), -(far + near) / (far - near), 1.0f
        };

        // Set the projection matrix uniform
        int projLocation = glGetUniformLocation(shaderProgram, "projection");
        glUniformMatrix4fv(projLocation, 1, GL_FALSE, projectionMatrix);

        // Set up view matrix (identity for now)
        float viewMatrix[16] = {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        };

        int viewLocation = glGetUniformLocation(shaderProgram, "view");
        glUniformMatrix4fv(viewLocation, 1, GL_FALSE, viewMatrix);

        // Draw
        glBindVertexArray(s_VAO);
        glDrawArrays(GL_TRIANGLES, 0, s_VertexBuffer.size());

        glBindVertexArray(0);
    }
}