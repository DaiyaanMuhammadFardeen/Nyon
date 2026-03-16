#version 460 core

// capsule.frag
// The capsule body and caps are a solid filled mesh so this is just an
// opaque solid. The outline path (DrawCapsule) uses the PolyLine buffer
// with the polygon shader instead of this pipeline.

in vec3  v_Color;
in float v_Outlined;   // currently unused for the solid fill path

out vec4 fragColor;

void main()
{
    fragColor = vec4(v_Color, 1.0);
}
