#pragma once
#include <string>
#include <fstream>
#include <sstream>
#include <stdexcept>
namespace Nyon::Graphics {
static std::string LoadShaderSource(const std::string& filepath) {
    std::ifstream file(filepath, std::ios::in | std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open shader file: " + filepath); }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str(); } }  
