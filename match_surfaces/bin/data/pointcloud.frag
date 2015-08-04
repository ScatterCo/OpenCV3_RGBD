#version 120
#extension GL_ARB_texture_rectangle : enable

const float epsilon = 1e-6;
uniform sampler2D sprite;

void main()
{

    gl_FragColor = texture2D(sprite, gl_TexCoord[0].st) * gl_Color;
    
}

