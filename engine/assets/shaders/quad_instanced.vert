#version 460 core

layout(location = 0) in vec2 a_LocalPos;       // unit quad: (-1,-1) to (1,1)
layout(location = 1) in vec2 a_InstPos;
layout(location = 2) in vec2 a_InstAngleRadius;
layout(location = 3) in vec3 a_InstColor;
layout(location = 4) in float a_InstAspect;

uniform mat4 u_VP;
out vec3 v_Color;

void main() {
    float angle  = a_InstAngleRadius.x;
    float radius = a_InstAngleRadius.y;

    float c = cos(angle); float s = sin(angle);
    vec2 scaled  = vec2(a_LocalPos.x * radius * a_InstAspect, a_LocalPos.y * radius);
    vec2 rotated = vec2(scaled.x*c - scaled.y*s, scaled.x*s + scaled.y*c);

    gl_Position = u_VP * vec4(a_InstPos + rotated, 0.0, 1.0);
    v_Color = a_InstColor;
}