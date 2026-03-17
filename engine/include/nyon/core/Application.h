#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <memory>
#include "nyon/EngineConstants.h"

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
        virtual void OnUpdate(float deltaTime) {} // For backward compatibility - per-frame user logic
        virtual void OnFixedUpdate(float deltaTime) {} // Fixed timestep update for physics - no default delegation
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

        static Application* s_Instance;
    };
}