#version 460 core

in vec3 v_Color;
in vec2 v_LocalPos;
out vec4 fragColor;

void main() {
    float d     = length(v_LocalPos);
    if (d > 1.0) discard;
    float alpha = 1.0 - smoothstep(0.85, 1.0, d);
    // Optional: add specular highlight for 3D look
    float light = 1.0 - 0.3 * d;
    fragColor = vec4(v_Color * light, alpha);
}