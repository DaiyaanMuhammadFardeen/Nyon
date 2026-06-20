#version 460 core

// -----------------------------------------------------------------------
// line.vert
// Instanced line renderer.  Replaces both thin (GL_LINES) and thick
// (rectangle) paths from the original CPU renderer with a single pipeline.
//
// The static mesh is a unit rectangle:
//   X ∈ [0,1]     — parametric position along the line (a_Start → a_End)
//   Y ∈ [-0.5, +0.5] — lateral position (scaled by thickness)
//
// Instance attribs (divisor=1):
//   layout 1 — a_Start:     world start point  (vec2)
//   layout 2 — a_End:       world end point    (vec2)
//   layout 3 — a_Color:     rgb                (vec3)
//   layout 4 — a_Thickness: world-space width  (float)
// -----------------------------------------------------------------------

layout(location = 0) in vec2  a_LocalPos;   // (x ∈ [0,1], y ∈ [-0.5,+0.5])
layout(location = 1) in vec2  a_Start;
layout(location = 2) in vec2  a_End;
layout(location = 3) in vec3  a_Color;
layout(location = 4) in float a_Thickness;

uniform mat4 u_VP;

out vec3 v_Color;

void main()
{
    vec2  dir     = a_End - a_Start;
    float len     = length(dir);
    vec2  axisDir = (len > 0.0001) ? dir / len : vec2(1.0, 0.0);
    vec2  perpDir = vec2(-axisDir.y, axisDir.x);

    // Map local (x, y) to world space:
    //   x=0 → a_Start, x=1 → a_End  (along the line)
    //   y    → lateral offset scaled by thickness
    vec2 world = a_Start
               + axisDir * (a_LocalPos.x * len)
               + perpDir * (a_LocalPos.y * a_Thickness);

    gl_Position = u_VP * vec4(world, 0.0, 1.0);
    v_Color = a_Color;
}
