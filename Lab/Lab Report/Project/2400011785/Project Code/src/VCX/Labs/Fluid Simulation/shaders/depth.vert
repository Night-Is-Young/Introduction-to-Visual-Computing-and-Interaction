#version 410 core
layout(location = 0) in vec3 position;

uniform mat4 u_View;
uniform mat4 u_Projection;
uniform float u_PointRadius;
uniform float u_ScreenHeight;
uniform float minDensity;

out vec3 v_ViewPos;

void main() {
    vec4 viewPos = u_View * vec4(position, 1.0);
    v_ViewPos = viewPos.xyz;
    gl_Position = u_Projection * viewPos;
    
    gl_PointSize = u_ScreenHeight * u_Projection[1][1] * u_PointRadius / gl_Position.w;
}