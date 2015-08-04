#version 120
#extension GL_ARB_texture_rectangle : enable

uniform sampler2DRect texture;

void main() {
    vec4 texColor = texture2DRect(texture, gl_TexCoord[0].st);
    texColor.rgb = texColor.rgb * vec3(25);
    gl_FragColor = texColor * gl_Color;
}
