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
        
        m_CurrentTime = glfwGetTime();
        m_Accumulator = 0.0;
        std::cerr << "[DEBUG] Initial current time: " << m_CurrentTime << std::endl;

        while (!glfwWindowShouldClose(m_Window) && m_Running)
        {
            double newTime = glfwGetTime();
            double frameTime = newTime - m_CurrentTime;
            m_CurrentTime = newTime;

            // Prevent spiral of death: cap the frame time
            if (frameTime > MAX_FRAME_TIME)
                frameTime = MAX_FRAME_TIME;

            m_Accumulator += frameTime;

            // --- PHYSICS UPDATE LOOP ---
            // Consumes time from the accumulator in fixed chunks
            while (m_Accumulator >= FIXED_TIMESTEP)
            {
                ProcessInput();
                OnFixedUpdate(static_cast<float>(FIXED_TIMESTEP));
                
                // Advance simulation time
                m_Accumulator -= FIXED_TIMESTEP;
            }

            // --- RENDER ---
            // Calculate 'alpha': how far are we into the *next* physics frame?
            // 0.0 = exactly at prev state, 1.0 = exactly at current state
            double alpha = m_Accumulator / FIXED_TIMESTEP;

            OnInterpolateAndRender(static_cast<float>(alpha));

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