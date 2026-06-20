#version 460 core

// -----------------------------------------------------------------------
// capsule.vert
// Instanced capsule renderer.
//
// The static mesh encodes each vertex as (localX, localY, type):
//
//   type 0.0  →  left-cap vertex  relative to c1
//                world = c1 + (axisDir×localX + perpDir×localY) × radius
//
//   type 1.0  →  right-cap vertex relative to c2
//                world = c2 + (axisDir×localX + perpDir×localY) × radius
//
//   type 2.0  →  body-rectangle vertex
//                world = mix(c1, c2, localX) + perpDir × (localY × radius)
//
// This encoding lets the vertex shader orient and scale the capsule to any
// arbitrary pair of world-space endpoints purely from per-instance data,
// with no CPU involvement.
//
// Mesh attrib (per vertex):
//   layout 0 — a_MeshPos: (localX, localY, type)  vec3
//
// Instance attribs (divisor=1):
//   layout 1 — a_C1:       center1   (vec2)
//   layout 2 — a_C2:       center2   (vec2)
//   layout 3 — a_Radius:   radius    (float)
//   layout 4 — a_Color:    rgb       (vec3)
//   layout 5 — a_Outlined: 0=filled, 1=outline (float, used by frag)
// -----------------------------------------------------------------------

layout(location = 0) in vec3  a_MeshPos;
layout(location = 1) in vec2  a_C1;
layout(location = 2) in vec2  a_C2;
layout(location = 3) in float a_Radius;
layout(location = 4) in vec3  a_Color;
layout(location = 5) in float a_Outlined;

uniform mat4 u_VP;

out vec3  v_Color;
out float v_Outlined;

void main()
{
    float localX = a_MeshPos.x;
    float localY = a_MeshPos.y;
    float type   = a_MeshPos.z;

    // Build capsule orientation frame from the two instance endpoints
    vec2  axis    = a_C2 - a_C1;
    float len     = length(axis);
    vec2  axisDir = (len > 0.0001) ? axis / len : vec2(1.0, 0.0);
    vec2  perpDir = vec2(-axisDir.y, axisDir.x);

    vec2 world;

    if (type < 0.5)
    {
        // Left cap — relative to c1
        // localX, localY are cos(a) and sin(a) on the unit semicircle
        world = a_C1
              + axisDir * (localX * a_Radius)
              + perpDir * (localY * a_Radius);
    }
    else if (type < 1.5)
    {
        // Right cap — relative to c2
        world = a_C2
              + axisDir * (localX * a_Radius)
              + perpDir * (localY * a_Radius);
    }
    else
    {
        // Body rectangle
        //   localX ∈ [0,1] parametrises along the c1→c2 axis
        //   localY ∈ [-1,+1] parametrises the lateral extent × radius
        world = mix(a_C1, a_C2, localX)
              + perpDir * (localY * a_Radius);
    }

    gl_Position = u_VP * vec4(world, 0.0, 1.0);
    v_Color    = a_Color;
    v_Outlined = a_Outlined;
}
