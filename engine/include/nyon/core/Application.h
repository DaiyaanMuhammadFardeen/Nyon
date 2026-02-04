#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <memory>

namespace Nyon
{
    class Application
    {
    public:
        Application(const char* title, int width, int height);
        virtual ~Application();

        void Run();
        void Close();

        static Application& Get() { return *s_Instance; }
        GLFWwindow* GetWindow() const { return m_Window; }

    protected:
        // Methods that can be overridden by games
        virtual void OnStart() {}
        virtual void OnUpdate(float deltaTime) {} // For backward compatibility
        virtual void OnFixedUpdate(float deltaTime) { OnUpdate(deltaTime); } // Fixed timestep update for physics
        virtual void OnRender() {} // For backward compatibility
        virtual void OnInterpolateAndRender(float alpha) { OnRender(); } // Render with interpolation
        virtual void OnEvent() {}

    private:
        void Init();
        void ProcessInput();

    private:
        GLFWwindow* m_Window;
        bool m_Running;
        float m_LastFrameTime;
        const char* m_Title;
        int m_Width, m_Height;
        
        // Variables for fixed timestep game loop with interpolation
        double m_CurrentTime;
        double m_Accumulator;
        static constexpr double FIXED_TIMESTEP = 1.0 / 60.0;  // 60 Hz physics updates
        static constexpr double MAX_FRAME_TIME = 0.25;       // Max 0.25s catch up to prevent spiral of death

        static Application* s_Instance;
    };
}