#version 410 core
in vec2 v_TexCoord;

uniform sampler2D u_DepthMap;
uniform vec2 u_TexelSize;
uniform float u_BlurScale;
uniform float u_BlurDepthFalloff;

out float o_Depth;

void main() {
    
    float depth = texture(u_DepthMap, v_TexCoord).r;
    
    if (depth >= 1.0) { 
        o_Depth = 1.0; 
        return; 
    }

    float sum = 0.0;
    float wsum = 0.0;
    
    int r = 10;
    for(int x = -r; x <= r; ++x) {
        for(int y = -r; y <= r; ++y) {
            vec2 offset = vec2(float(x), float(y)) * u_TexelSize;
            float sampleDepth = texture(u_DepthMap, v_TexCoord + offset).r;
            
            if(sampleDepth >= 1.0) continue;

            float r2 = float(x*x + y*y);
            float wSpatial = exp(-r2 * u_BlurScale);

            float dDiff = sampleDepth - depth;
            float wRange = exp(-dDiff * dDiff * u_BlurDepthFalloff);

            float w = wSpatial * wRange;
            sum += sampleDepth * w;
            wsum += w;
        }
    }

    if (wsum > 0.0)
        o_Depth = sum / wsum;
    else
        o_Depth = depth;
}