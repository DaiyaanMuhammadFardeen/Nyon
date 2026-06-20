#version 460 core

// polygon.frag — opaque solid, shared by filled and outline polygon passes

in vec3 v_Color;
out vec4 fragColor;

void main()
{
    fragColor = vec4(v_Color, 1.0);
}
