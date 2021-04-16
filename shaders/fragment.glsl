#version 330 core

in vec3 point_color;
in vec2 uv;

uniform bool use_texture;
uniform sampler2D tex;

out vec4 color;

void main() {
    if (use_texture) {
        color = texture(tex, uv);
    } else {
        color = vec4(point_color, 1.0);
    }
    
}
