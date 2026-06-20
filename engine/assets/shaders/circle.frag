#version 460 core

// -----------------------------------------------------------------------
// circle.frag
// SDF-based circle renderer.
//
// v_LocalPos is the local coordinate interpolated from the vertex stage.
// For the triangle fan mesh its range is [-1,1] (the perimeter vertices
// are exactly on the unit circle; the centre is at (0,0)).
//
// The SDF is simply dist = length(v_LocalPos).
//   dist > 1.0          → outside the circle      → discard
//   dist 0.90 .. 1.00   → anti-aliased edge (filled)
//   dist 0.78 .. 0.82   → inner edge of outline ring
//   dist 0.93 .. 1.00   → outer edge of outline ring
// -----------------------------------------------------------------------

in vec3  v_Color;
in vec2  v_LocalPos;
in float v_Outlined;

out vec4 fragColor;

void main()
{
    float dist = length(v_LocalPos);

    // Hard cull — all fragments outside the circle are discarded.
    // This converts the 16-sided polygon mesh into a mathematically
    // perfect circle at subpixel precision.
    if (dist > 1.0) discard;

    float alpha;

    if (v_Outlined > 0.5)
    {
        // Outline ring: render only pixels near the perimeter.
        // Ring inner boundary at ~0.80, outer at ~1.00.
        float inner = smoothstep(0.78, 0.82, dist);   // fade in at inner edge
        float outer = 1.0 - smoothstep(0.93, 1.0, dist); // fade out at outer edge
        alpha = inner * outer;
        if (alpha < 0.01) discard;
    }
    else
    {
        // Filled circle with anti-aliased soft edge.
        alpha = 1.0 - smoothstep(0.90, 1.0, dist);
    }

    fragColor = vec4(v_Color, alpha);
}
