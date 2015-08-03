#version 120
#extension GL_ARB_texture_rectangle : enable

uniform sampler2DRect texture;

void main() {
    vec4 texColor = texture2DRect(texture, gl_TexCoord[0].st);
    //texColor.rg = texColor.rg * vec2(0.5) + vec2(0.5);
    //texColor.b = texColor.b + 0.5 + 0.5;
    texColor.b *= -1.0;
    texColor.rgb = texColor.rgb * vec3(0.5) + vec3(0.5);
    gl_FragColor = texColor * gl_Color;
}
