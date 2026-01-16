#version 410 core

layout(location = 0) in  vec3 a_Position;
layout(location = 1) in  vec3 a_Velocity;

layout(location = 0) out vec3 v_Position;
layout(location = 1) out vec3 v_Velocity;

uniform mat4  u_Projection;
uniform mat4  u_View;
uniform vec3  u_Color;

void main() {
    v_Position  = a_Position;
    v_Velocity  = a_Velocity;
    gl_Position = u_Projection * u_View * vec4(v_Position, 1.);
}
