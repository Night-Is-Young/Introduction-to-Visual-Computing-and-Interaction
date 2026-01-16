#version 410 core
in vec3 v_ViewPos;

uniform float u_PointRadius;
uniform mat4 u_Projection;

void main() {
    vec2 coord = gl_PointCoord * 2.0 - 1.0;
    float mag = dot(coord, coord);
    
    if (mag > 1.0) discard;
    
    vec3 normal = vec3(coord, sqrt(1.0 - mag));
    
    vec3 spherePos = v_ViewPos + normal * u_PointRadius;
    vec4 clipPos = u_Projection * vec4(spherePos, 1.0);
    
    gl_FragDepth = clipPos.z / clipPos.w;
}