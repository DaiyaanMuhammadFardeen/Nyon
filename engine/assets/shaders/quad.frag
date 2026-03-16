#version 460 core

// quad.frag — opaque solid fill, color from vertex stage

in vec3 v_Color;
out vec4 fragColor;

void main()
{
    fragColor = vec4(v_Color, 1.0);
}
