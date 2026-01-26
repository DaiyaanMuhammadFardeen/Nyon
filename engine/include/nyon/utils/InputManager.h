#pragma once

#include <GLFW/glfw3.h>

namespace Nyon::Utils
{
    class InputManager
    {
    public:
        static void Init(GLFWwindow* window);
        static void Update();
        
        // Keyboard input
        static bool IsKeyPressed(int key);
        static bool IsKeyDown(int key);
        static bool IsKeyUp(int key);
        
        // Mouse input
        static bool IsMousePressed(int button);
        static bool IsMouseDown(int button);
        static bool IsMouseUp(int button);
        static void GetMousePosition(double& x, double& y);
        
    private:
        static GLFWwindow* s_Window;
        static bool s_CurrentKeys[GLFW_KEY_LAST];
        static bool s_PreviousKeys[GLFW_KEY_LAST];
        static bool s_CurrentMouseButtons[GLFW_MOUSE_BUTTON_LAST];
        static bool s_PreviousMouseButtons[GLFW_MOUSE_BUTTON_LAST];
    };
}