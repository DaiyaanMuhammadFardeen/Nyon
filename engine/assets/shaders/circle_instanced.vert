#version 460 core

layout(location = 0) in vec2 a_LocalPos;       // unit circle mesh
layout(location = 1) in vec2 a_InstPos;         // instance: world position
layout(location = 2) in vec2 a_InstAngleRadius; // instance: (angle, radius)
layout(location = 3) in vec3 a_InstColor;        // instance: rgb
layout(location = 4) in float a_InstAspect;      // instance: aspect ratio

uniform mat4 u_VP;

out vec3 v_Color;
out vec2 v_LocalPos;

void main() {
    float angle  = a_InstAngleRadius.x;
    float radius = a_InstAngleRadius.y;

    float c = cos(angle);
    float s = sin(angle);
    // Scale local pos by radius and aspect, then rotate, then translate
    vec2 scaled = vec2(a_LocalPos.x * radius * a_InstAspect,
                       a_LocalPos.y * radius);
    vec2 rotated = vec2(scaled.x * c - scaled.y * s,
                        scaled.x * s + scaled.y * c);
    vec2 world = a_InstPos + rotated;

    gl_Position = u_VP * vec4(world, 0.0, 1.0);
    v_Color     = a_InstColor;
    v_LocalPos  = a_LocalPos;
}