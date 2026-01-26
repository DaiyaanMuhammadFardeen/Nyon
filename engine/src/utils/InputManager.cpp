#include "nyon/utils/InputManager.h"
#include <cstring>
#include <iostream>

namespace Nyon::Utils
{
    GLFWwindow* InputManager::s_Window = nullptr;
    bool InputManager::s_CurrentKeys[GLFW_KEY_LAST] = {};
    bool InputManager::s_PreviousKeys[GLFW_KEY_LAST] = {};
    bool InputManager::s_CurrentMouseButtons[GLFW_MOUSE_BUTTON_LAST] = {};
    bool InputManager::s_PreviousMouseButtons[GLFW_MOUSE_BUTTON_LAST] = {};

    void InputManager::Init(GLFWwindow* window)
    {
        std::cerr << "[DEBUG] InputManager::Init() called with window: " << window << std::endl;
        s_Window = window;
    }

    void InputManager::Update()
    {
        if (s_Window == nullptr) {
            std::cerr << "[ERROR] InputManager::Update() called with null window!" << std::endl;
            return;
        }
        
        std::cerr << "[DEBUG] InputManager::Update() called" << std::endl;
        
        // Copy current state to previous state
        memcpy(s_PreviousKeys, s_CurrentKeys, sizeof(s_CurrentKeys));
        memcpy(s_PreviousMouseButtons, s_CurrentMouseButtons, sizeof(s_CurrentMouseButtons));

        // Update current state
        for (int i = 0; i < GLFW_KEY_LAST; i++)
        {
            s_CurrentKeys[i] = (glfwGetKey(s_Window, i) == GLFW_PRESS);
        }

        for (int i = 0; i < GLFW_MOUSE_BUTTON_LAST; i++)
        {
            s_CurrentMouseButtons[i] = (glfwGetMouseButton(s_Window, i) == GLFW_PRESS);
        }
    }

    bool InputManager::IsKeyPressed(int key)
    {
        if (s_Window == nullptr) {
            std::cerr << "[ERROR] InputManager::IsKeyPressed() called with null window!" << std::endl;
            return false;
        }
        return s_CurrentKeys[key] && !s_PreviousKeys[key];
    }

    bool InputManager::IsKeyDown(int key)
    {
        if (s_Window == nullptr) {
            std::cerr << "[ERROR] InputManager::IsKeyDown() called with null window!" << std::endl;
            return false;
        }
        return s_CurrentKeys[key];
    }

    bool InputManager::IsKeyUp(int key)
    {
        if (s_Window == nullptr) {
            std::cerr << "[ERROR] InputManager::IsKeyUp() called with null window!" << std::endl;
            return false;
        }
        return !s_CurrentKeys[key];
    }

    bool InputManager::IsMousePressed(int button)
    {
        if (s_Window == nullptr) {
            std::cerr << "[ERROR] InputManager::IsMousePressed() called with null window!" << std::endl;
            return false;
        }
        return s_CurrentMouseButtons[button] && !s_PreviousMouseButtons[button];
    }

    bool InputManager::IsMouseDown(int button)
    {
        if (s_Window == nullptr) {
            std::cerr << "[ERROR] InputManager::IsMouseDown() called with null window!" << std::endl;
            return false;
        }
        return s_CurrentMouseButtons[button];
    }

    bool InputManager::IsMouseUp(int button)
    {
        if (s_Window == nullptr) {
            std::cerr << "[ERROR] InputManager::IsMouseUp() called with null window!" << std::endl;
            return false;
        }
        return !s_CurrentMouseButtons[button];
    }

    void InputManager::GetMousePosition(double& x, double& y)
    {
        if (s_Window == nullptr) {
            std::cerr << "[ERROR] InputManager::GetMousePosition() called with null window!" << std::endl;
            x = 0.0;
            y = 0.0;
            return;
        }
        glfwGetCursorPos(s_Window, &x, &y);
    }
}