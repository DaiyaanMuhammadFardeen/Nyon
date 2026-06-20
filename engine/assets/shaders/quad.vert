#version 460 core

// -----------------------------------------------------------------------
// quad.vert
// Instanced quad renderer.
//
// Mesh attrib (per vertex):
//   layout 0 — a_LocalPos: vec2 in [0,1]×[0,1] local space
//
// Instance attribs (per instance, divisor=1):
//   layout 1 — a_Pos:    world pivot position  (vec2)
//   layout 2 — a_Size:   width, height          (vec2)
//   layout 3 — a_Origin: pivot offset           (vec2)
//   layout 4 — a_Angle:  rotation in radians    (float)
//   layout 5 — a_Color:  rgb                    (vec3)
// -----------------------------------------------------------------------

layout(location = 0) in vec2  a_LocalPos;
layout(location = 1) in vec2  a_Pos;
layout(location = 2) in vec2  a_Size;
layout(location = 3) in vec2  a_Origin;
layout(location = 4) in float a_Angle;
layout(location = 5) in vec3  a_Color;

uniform mat4 u_VP;

out vec3 v_Color;

void main()
{
    // Map local [0,1]x[0,1] → corner offset relative to pivot:
    //   local (0,0) → corner at -origin
    //   local (1,1) → corner at size - origin
    vec2 local = a_LocalPos * a_Size - a_Origin;

    // Rotate the corner offset around the pivot by a_Angle
    float c = cos(a_Angle);
    float s = sin(a_Angle);
    vec2 rotated = vec2(local.x * c - local.y * s,
                        local.x * s + local.y * c);

    // Translate to world space
    vec2 world = a_Pos + rotated;

    gl_Position = u_VP * vec4(world, 0.0, 1.0);
    v_Color = a_Color;
}
