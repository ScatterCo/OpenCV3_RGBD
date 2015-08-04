#version 120
#extension GL_ARB_texture_rectangle : enable

uniform sampler2DRect texture;

void main() {
    gl_Position = ftransform();
    gl_TexCoord[0] = gl_MultiTexCoord0; 
    gl_FrontColor = gl_Color;
}
