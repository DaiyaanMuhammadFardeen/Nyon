#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <memory>
namespace Nyon {
    class Application {
    public:
        Application(const char* title, int width, int height);
        virtual ~Application();
        void Run();
        void Close();
        static Application& Get() { return *s_Instance; }
        GLFWwindow* GetWindow() const { return m_Window; }
    protected:
        virtual void OnStart() {}
        virtual void OnUpdate(float deltaTime) {}  
        virtual void OnFixedUpdate(float deltaTime) {}  
        virtual void OnRender() {}  
        virtual void OnInterpolateAndRender(float alpha) { OnRender(); }  
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
        double m_CurrentTime;
        double m_Accumulator;
        static constexpr double FIXED_TIMESTEP = 1.0 / 60.0;   
        static constexpr double MAX_FRAME_TIME = 0.25;        
        static Application* s_Instance; }; }