#version 410 core
out vec2 v_TexCoord;

void main() {
    vec2 vertices[3] = vec2[3](vec2(-1,-1), vec2(3,-1), vec2(-1, 3));
    gl_Position = vec4(vertices[gl_VertexID], 0, 1);
    v_TexCoord = 0.5 * gl_Position.xy + 0.5;
}