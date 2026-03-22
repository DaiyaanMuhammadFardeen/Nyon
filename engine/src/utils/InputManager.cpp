#include "nyon/utils/InputManager.h"
#include <cstring>
#include <iostream>
namespace Nyon::Utils {
    GLFWwindow* InputManager::s_Window = nullptr;
    bool InputManager::s_CurrentKeys[GLFW_KEY_LAST] = {};
    bool InputManager::s_PreviousKeys[GLFW_KEY_LAST] = {};
    bool InputManager::s_CurrentMouseButtons[GLFW_MOUSE_BUTTON_LAST] = {};
    bool InputManager::s_PreviousMouseButtons[GLFW_MOUSE_BUTTON_LAST] = {};
    std::unordered_set<int> InputManager::s_ActiveKeys;
    std::unordered_set<int> InputManager::s_ActiveMouseButtons;
    bool InputManager::s_InputDirty = false;
    void InputManager::Init(GLFWwindow* window) {
        s_Window = window;
        if (window != nullptr) {
            glfwSetKeyCallback(window, KeyCallback);
            glfwSetMouseButtonCallback(window, MouseButtonCallback); } }
    void InputManager::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
        if (key >= 0 && key < GLFW_KEY_LAST) {
            if (action == GLFW_PRESS) {
                s_ActiveKeys.insert(key);
                s_CurrentKeys[key] = true; } else if (action == GLFW_RELEASE) {
                s_ActiveKeys.erase(key);
                s_CurrentKeys[key] = false; }
            s_InputDirty = true; } }
    void InputManager::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
        if (button >= 0 && button < GLFW_MOUSE_BUTTON_LAST) {
            if (action == GLFW_PRESS) {
                s_ActiveMouseButtons.insert(button);
                s_CurrentMouseButtons[button] = true; } else if (action == GLFW_RELEASE) {
                s_ActiveMouseButtons.erase(button);
                s_CurrentMouseButtons[button] = false; }
            s_InputDirty = true; } }
    void InputManager::Update() {
        if (s_Window == nullptr) {
            return; }
        memcpy(s_PreviousKeys, s_CurrentKeys, sizeof(s_CurrentKeys));
        memcpy(s_PreviousMouseButtons, s_CurrentMouseButtons, sizeof(s_CurrentMouseButtons));
        s_InputDirty = false; }
    bool InputManager::IsKeyPressed(int key) {
        if (s_Window == nullptr || key < 0 || key >= GLFW_KEY_LAST) {
            return false; }
        return s_CurrentKeys[key] && !s_PreviousKeys[key]; }
    bool InputManager::IsKeyDown(int key) {
        if (s_Window == nullptr || key < 0 || key >= GLFW_KEY_LAST) {
            return false; }
        return s_CurrentKeys[key]; }
    bool InputManager::IsKeyUp(int key) {
        if (key < 0 || key >= GLFW_KEY_LAST) {
            return false; }
        if (s_Window == nullptr) {
            return true; }
        return !s_CurrentKeys[key]; }
    bool InputManager::IsMousePressed(int button) {
        if (s_Window == nullptr || button < 0 || button >= GLFW_MOUSE_BUTTON_LAST) {
            return false; }
        return s_CurrentMouseButtons[button] && !s_PreviousMouseButtons[button]; }
    bool InputManager::IsMouseDown(int button) {
        if (s_Window == nullptr || button < 0 || button >= GLFW_MOUSE_BUTTON_LAST) {
            return false; }
        return s_CurrentMouseButtons[button]; }
    bool InputManager::IsMouseUp(int button) {
        if (button < 0 || button >= GLFW_MOUSE_BUTTON_LAST) {
            return false; }
        if (s_Window == nullptr) {
            return true; }
        return !s_CurrentMouseButtons[button]; }
    void InputManager::GetMousePosition(double& x, double& y) {
        if (s_Window == nullptr) {
            x = 0.0;
            y = 0.0;
            return; }
        glfwGetCursorPos(s_Window, &x, &y); } }