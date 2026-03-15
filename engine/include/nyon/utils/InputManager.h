#pragma once
#include <GLFW/glfw3.h>
#include <unordered_set>
namespace Nyon::Utils {
    class InputManager {
    public:
        static void Init(GLFWwindow* window);
        static void Update();
        static bool IsKeyPressed(int key);
        static bool IsKeyDown(int key);
        static bool IsKeyUp(int key);
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
        static std::unordered_set<int> s_ActiveKeys;
        static std::unordered_set<int> s_ActiveMouseButtons;
        static bool s_InputDirty;
        static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
        static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods); }; }