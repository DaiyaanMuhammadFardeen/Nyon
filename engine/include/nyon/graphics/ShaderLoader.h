#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace Nyon::Graphics {

/**
 * @brief Load shader source code from a file
 * @param filepath Path to the shader file
 * @return Shader source code as string
 */
static std::string LoadShaderSource(const std::string& filepath)
{
    std::ifstream file(filepath, std::ios::in | std::ios::binary);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open shader file: " + filepath);
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

} // namespace Nyon::Graphics
