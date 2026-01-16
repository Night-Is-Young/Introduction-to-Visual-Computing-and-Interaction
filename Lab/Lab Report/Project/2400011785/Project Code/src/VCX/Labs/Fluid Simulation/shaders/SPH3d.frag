#version 410 core

layout(location = 0) in  vec3 v_Position;
layout(location = 1) in  vec3 v_Velocity;

layout(location = 0) out vec4 f_Color;

uniform float u_MaxSpeed;

vec3 colorMap(float t) {
    if (t < 0.25)      return mix(vec3(0,0,1), vec3(0,1,1), t/0.25);
    else if (t < 0.5)  return mix(vec3(0,1,1), vec3(0,1,0), (t-0.25)/0.25);
    else if (t < 0.75) return mix(vec3(0,1,0), vec3(1,1,0), (t-0.5)/0.25);
    else               return mix(vec3(1,1,0), vec3(1,0,0), (t-0.75)/0.25);
}

void main()
{
    vec2 c = gl_PointCoord * 2.0 - 1.0;
    float r2 = dot(c, c);
    if (r2 > 1.0)
        discard;
    float speed = length(v_Velocity);
    float t = clamp(speed / u_MaxSpeed, 0.0, 1.0);
    vec3 col = colorMap(t);
    f_Color = vec4(col, 1.);
}
