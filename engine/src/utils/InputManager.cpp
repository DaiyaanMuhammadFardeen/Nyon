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
    
    // Callback-based input tracking
    std::unordered_set<int> InputManager::s_ActiveKeys;
    std::unordered_set<int> InputManager::s_ActiveMouseButtons;
    bool InputManager::s_InputDirty = false;

    void InputManager::Init(GLFWwindow* window)
    {
        s_Window = window;
        
        if (window != nullptr) {
            // Register GLFW callbacks for efficient input handling
            glfwSetKeyCallback(window, KeyCallback);
            glfwSetMouseButtonCallback(window, MouseButtonCallback);
            
            // Initialize active sets with currently pressed keys
            for (int i = 0; i < GLFW_KEY_LAST; ++i) {
                if (glfwGetKey(window, i) == GLFW_PRESS) {
                    s_ActiveKeys.insert(i);
                    s_CurrentKeys[i] = true;
                }
            }
            
            for (int i = 0; i < GLFW_MOUSE_BUTTON_LAST; ++i) {
                if (glfwGetMouseButton(window, i) == GLFW_PRESS) {
                    s_ActiveMouseButtons.insert(i);
                    s_CurrentMouseButtons[i] = true;
                }
            }
        }
    }
    
    // GLFW key callback - only called when key state changes
    void InputManager::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
    {
        if (key >= 0 && key < GLFW_KEY_LAST) {
            if (action == GLFW_PRESS) {
                s_ActiveKeys.insert(key);
                s_CurrentKeys[key] = true;
            } else if (action == GLFW_RELEASE) {
                s_ActiveKeys.erase(key);
                s_CurrentKeys[key] = false;
            }
            s_InputDirty = true;
        }
    }
    
    // GLFW mouse button callback - only called when button state changes
    void InputManager::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
    {
        if (button >= 0 && button < GLFW_MOUSE_BUTTON_LAST) {
            if (action == GLFW_PRESS) {
                s_ActiveMouseButtons.insert(button);
                s_CurrentMouseButtons[button] = true;
            } else if (action == GLFW_RELEASE) {
                s_ActiveMouseButtons.erase(button);
                s_CurrentMouseButtons[button] = false;
            }
            s_InputDirty = true;
        }
    }

    void InputManager::Update()
    {
        if (s_Window == nullptr) {
            return;
        }
        
        // Only update state if input has changed (callback triggered)
        if (s_InputDirty) {
            // Copy current state to previous state
            memcpy(s_PreviousKeys, s_CurrentKeys, sizeof(s_CurrentKeys));
            memcpy(s_PreviousMouseButtons, s_CurrentMouseButtons, sizeof(s_CurrentMouseButtons));
            
            // Reset dirty flag
            s_InputDirty = false;
        }
        
        // Mouse position is still polled every frame as it can change without button events
        // This is acceptable since it's a single call vs 348 key checks
    }

    bool InputManager::IsKeyPressed(int key)
    {
        if (s_Window == nullptr || key < 0 || key >= GLFW_KEY_LAST) {
            return false;
        }
        return s_CurrentKeys[key] && !s_PreviousKeys[key];
    }

    bool InputManager::IsKeyDown(int key)
    {
        if (s_Window == nullptr || key < 0 || key >= GLFW_KEY_LAST) {
            return false;
        }
        return s_CurrentKeys[key];
    }

    bool InputManager::IsKeyUp(int key)
    {
        if (key < 0 || key >= GLFW_KEY_LAST) {
            return false;
        }
        // When no window is available, treat all keys as up
        if (s_Window == nullptr) {
            return true;
        }
        return !s_CurrentKeys[key];
    }

    bool InputManager::IsMousePressed(int button)
    {
        if (s_Window == nullptr || button < 0 || button >= GLFW_MOUSE_BUTTON_LAST) {
            return false;
        }
        return s_CurrentMouseButtons[button] && !s_PreviousMouseButtons[button];
    }

    bool InputManager::IsMouseDown(int button)
    {
        if (s_Window == nullptr || button < 0 || button >= GLFW_MOUSE_BUTTON_LAST) {
            return false;
        }
        return s_CurrentMouseButtons[button];
    }

    bool InputManager::IsMouseUp(int button)
    {
        if (button < 0 || button >= GLFW_MOUSE_BUTTON_LAST) {
            return false;
        }
        // When no window is available, treat all buttons as up
        if (s_Window == nullptr) {
            return true;
        }
        return !s_CurrentMouseButtons[button];
    }

    void InputManager::GetMousePosition(double& x, double& y)
    {
        if (s_Window == nullptr) {
            x = 0.0;
            y = 0.0;
            return;
        }
        glfwGetCursorPos(s_Window, &x, &y);
    }
}