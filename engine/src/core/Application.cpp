#include "nyon/core/Application.h"
#include "nyon/graphics/Renderer2D.h"
#include <iostream>
#ifdef _DEBUG
#define NYON_DEBUG_LOG(x) std::cerr << x << std::endl
#else
#define NYON_DEBUG_LOG(x)
#endif
namespace Nyon {
    Application* Application::s_Instance = nullptr;
    Application::Application(const char* title, int width, int height)
        : m_Window(nullptr), m_Running(true), m_LastFrameTime(0.0f), 
          m_Title(title), m_Width(width), m_Height(height)
    {
#ifdef _DEBUG
        std::cerr << "[DEBUG] Application constructor called" << std::endl;
#endif
        s_Instance = this;
        Init();
    }
    Application::~Application()
    {
#ifdef _DEBUG
        std::cerr << "[DEBUG] Application destructor called" << std::endl;
#endif
        Graphics::Renderer2D::Shutdown();
        glfwDestroyWindow(m_Window);
        glfwTerminate();
    }
    void Application::Init()
    {
#ifdef _DEBUG
        std::cerr << "[DEBUG] Application::Init() called" << std::endl;
#endif
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
#ifdef _DEBUG
        std::cerr << "[DEBUG] GLFW window created successfully" << std::endl;
#endif
        glfwMakeContextCurrent(m_Window);
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        {
            std::cerr << "Failed to initialize GLAD!" << std::endl;
            return;
        }
        glViewport(0, 0, m_Width, m_Height);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        Graphics::Renderer2D::Init();
#ifdef _DEBUG
        std::cerr << "Nyon Engine initialized successfully!" << std::endl;
#endif
    }
    void Application::Run()
    {
#ifdef _DEBUG
        std::cerr << "[DEBUG] Application::Run() called" << std::endl;
#endif
        OnStart();
        m_CurrentTime = glfwGetTime();
        m_Accumulator = 0.0;
#ifdef _DEBUG
        std::cerr << "[DEBUG] Initial current time: " << m_CurrentTime << std::endl;
#endif
        while (!glfwWindowShouldClose(m_Window) && m_Running)
        {
            double newTime = glfwGetTime();
            double frameTime = newTime - m_CurrentTime;
            m_CurrentTime = newTime;
            if (frameTime > MAX_FRAME_TIME)
                frameTime = MAX_FRAME_TIME;
            m_Accumulator += frameTime;
            ProcessInput();
            while (m_Accumulator >= FIXED_TIMESTEP)
            {
                OnFixedUpdate(static_cast<float>(FIXED_TIMESTEP));
                m_Accumulator -= FIXED_TIMESTEP;
            }
            double alpha = m_Accumulator / FIXED_TIMESTEP;
            OnInterpolateAndRender(static_cast<float>(alpha));
            glfwSwapBuffers(m_Window);
            glfwPollEvents();
        }
#ifdef _DEBUG
        std::cerr << "[DEBUG] Application::Run() ended" << std::endl;
#endif
    }
    void Application::ProcessInput()
    {
        if (m_Window != nullptr && glfwGetKey(m_Window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            Close();
    }
    void Application::Close()
    {
#ifdef _DEBUG
        std::cerr << "[DEBUG] Application::Close() called" << std::endl;
#endif
        m_Running = false;
    }
}