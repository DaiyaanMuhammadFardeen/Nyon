#version 460 core

// -----------------------------------------------------------------------
// circle.vert
// Instanced circle renderer.  The static mesh is a 16-sided triangle fan
// in unit-circle space.  The fragment shader uses SDF to produce a perfect
// circle regardless of polygon approximation.
//
// Mesh attrib (per vertex):
//   layout 0 — a_LocalPos: vec2 on the unit circle  (|a_LocalPos| ≤ 1)
//
// Instance attribs (divisor=1):
//   layout 1 — a_Center:   world center  (vec2)
//   layout 2 — a_Radius:   radius        (float)
//   layout 3 — a_Color:    rgb           (vec3)
//   layout 4 — a_Outlined: 0=filled, 1=outline ring (float)
// -----------------------------------------------------------------------

layout(location = 0) in vec2  a_LocalPos;
layout(location = 1) in vec2  a_Center;
layout(location = 2) in float a_Radius;
layout(location = 3) in vec3  a_Color;
layout(location = 4) in float a_Outlined;

uniform mat4 u_VP;

out vec3  v_Color;
out vec2  v_LocalPos;    // passed to frag for SDF evaluation
out float v_Outlined;

void main()
{
    // Scale unit circle by radius and translate to world center.
    vec2 world = a_Center + a_LocalPos * a_Radius;

    gl_Position = u_VP * vec4(world, 0.0, 1.0);

    v_Color    = a_Color;
    v_LocalPos = a_LocalPos;   // still in [-1,1] unit space
    v_Outlined = a_Outlined;
}
