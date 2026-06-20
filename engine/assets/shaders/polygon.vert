#version 460 core

// -----------------------------------------------------------------------
// polygon.vert
// Passthrough shader for the CPU-tessellated polygon buffers.
// Used by both PolyFill (GL_TRIANGLES) and PolyLine (GL_LINES) draw calls.
//
// World-space Vertex structs are written directly into a persistently mapped
// GPU buffer by the CPU. No instancing — one vertex attribute per vertex.
//
// Vertex layout matches Nyon::Graphics::Vertex:
//   layout 0 — position  vec2  (x, y)        offset 0
//   layout 1 — color     vec3  (r, g, b)      offset 8
//   (u, v, nx, ny are present in the struct but not used by this shader)
// -----------------------------------------------------------------------

layout(location = 0) in vec2 a_Position;
layout(location = 1) in vec3 a_Color;

uniform mat4 u_VP;

out vec3 v_Color;

void main()
{
    gl_Position = u_VP * vec4(a_Position, 0.0, 1.0);
    v_Color = a_Color;
}
