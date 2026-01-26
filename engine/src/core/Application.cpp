#include "nyon/core/Application.h"
#include <iostream>

namespace Nyon
{
    Application* Application::s_Instance = nullptr;

    Application::Application(const char* title, int width, int height)
        : m_Window(nullptr), m_Running(true), m_LastFrameTime(0.0f), 
          m_Title(title), m_Width(width), m_Height(height)
    {
        std::cerr << "[DEBUG] Application constructor called" << std::endl;
        s_Instance = this;
        Init();
    }

    Application::~Application()
    {
        std::cerr << "[DEBUG] Application destructor called" << std::endl;
        glfwDestroyWindow(m_Window);
        glfwTerminate();
    }

    void Application::Init()
    {
        std::cerr << "[DEBUG] Application::Init() called" << std::endl;
        if (!glfwInit())
        {
            std::cerr << "Failed to initialize GLFW!" << std::endl;
            return;
        }

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        m_Window = glfwCreateWindow(m_Width, m_Height, m_Title, nullptr, nullptr);
        if (m_Window == nullptr)
        {
            std::cerr << "Failed to create GLFW window!" << std::endl;
            glfwTerminate();
            return;
        }

        std::cerr << "[DEBUG] GLFW window created successfully, pointer: " << m_Window << std::endl;

        glfwMakeContextCurrent(m_Window);

        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        {
            std::cerr << "Failed to initialize GLAD!" << std::endl;
            return;
        }

        glViewport(0, 0, m_Width, m_Height);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        std::cout << "Nyon Engine initialized successfully!" << std::endl;
    }

    void Application::Run()
    {
        std::cerr << "[DEBUG] Application::Run() called" << std::endl;
        
        // Call OnStart once before the main loop
        OnStart();
        
        float lastFrame = static_cast<float>(glfwGetTime());
        std::cerr << "[DEBUG] Initial lastFrame time: " << lastFrame << std::endl;

        while (!glfwWindowShouldClose(m_Window) && m_Running)
        {
            float currentFrame = static_cast<float>(glfwGetTime());
            float deltaTime = currentFrame - lastFrame;
            m_LastFrameTime = currentFrame;

            std::cerr << "[DEBUG] Processing frame with delta time: " << deltaTime << std::endl;
            
            ProcessInput();
            OnUpdate(deltaTime);
            OnRender();

            glfwSwapBuffers(m_Window);
            glfwPollEvents();
        }
        std::cerr << "[DEBUG] Application::Run() ended" << std::endl;
    }

    void Application::ProcessInput()
    {
        if (m_Window != nullptr && glfwGetKey(m_Window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            Close();
    }

    void Application::Close()
    {
        std::cerr << "[DEBUG] Application::Close() called" << std::endl;
        m_Running = false;
    }
}