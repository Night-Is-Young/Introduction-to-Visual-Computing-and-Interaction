#version 410 core
in vec2 v_TexCoord;

uniform sampler2D u_SmoothDepthMap;
uniform mat4 u_InvProjection;
uniform vec3 u_LightDir;
uniform vec3 u_LightColor;
uniform vec3 u_DarkColor;

out vec4 f_Color;

vec3 ViewPosFromDepth(vec2 texCoord, float depth) {
    vec4 clipPos = vec4(texCoord * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
    vec4 viewPos = u_InvProjection * clipPos;
    return viewPos.xyz / viewPos.w;
}

void main() {
    float depth = texture(u_SmoothDepthMap, v_TexCoord).r;
    
    if (depth >= 1.0) discard;

    vec3 viewPos = ViewPosFromDepth(v_TexCoord, depth);

    vec2 texelSize = 1.0 / vec2(textureSize(u_SmoothDepthMap, 0));
    
    float depthR = texture(u_SmoothDepthMap, v_TexCoord + vec2(texelSize.x, 0.0)).r;
    float depthL = texture(u_SmoothDepthMap, v_TexCoord - vec2(texelSize.x, 0.0)).r;
    float depthU = texture(u_SmoothDepthMap, v_TexCoord + vec2(0.0, texelSize.y)).r;
    float depthD = texture(u_SmoothDepthMap, v_TexCoord - vec2(0.0, texelSize.y)).r;

    if (depthR >= 1.0) depthR = depth;
    if (depthL >= 1.0) depthL = depth;
    if (depthU >= 1.0) depthU = depth;
    if (depthD >= 1.0) depthD = depth;

    vec3 posR = ViewPosFromDepth(v_TexCoord + vec2(texelSize.x, 0.0), depthR);
    vec3 posL = ViewPosFromDepth(v_TexCoord - vec2(texelSize.x, 0.0), depthL);
    vec3 posU = ViewPosFromDepth(v_TexCoord + vec2(0.0, texelSize.y), depthU);
    vec3 posD = ViewPosFromDepth(v_TexCoord - vec2(0.0, texelSize.y), depthD);

    vec3 ddx = posR - posL;
    vec3 ddy = posU - posD;
    vec3 normal = normalize(cross(ddx, ddy));

    vec3 lightDir = normalize(u_LightDir);
    vec3 viewDir = normalize(-viewPos);

    float diff = max(dot(normal, lightDir), 0.0);

    if (diff < 0.125) diff = 0;
    else if (diff < 0.375) diff = 0.25;
    else if (diff < 0.625) diff = 0.5;
    else if (diff < 0.875) diff = 0.75;
    else diff = 1.0;

    f_Color = vec4(diff*u_LightColor+(1-diff)*u_DarkColor,1.0);
}