#version 120
#extension GL_ARB_texture_rectangle : enable

uniform vec2 dimensions;
uniform sampler2DRect texture;
uniform vec2 pp;
uniform vec2 fov;
uniform mat4 calibration;

const float epsilon = 1e-6;

void main(void)
{
    //align to texture
    vec2 halfvec = vec2(.5,.5);
    float depth = texture2DRect(texture, floor(gl_Vertex.xy) + halfvec).b * 255.0;
    vec4 pos = vec4(gl_Vertex.xy, depth, 1.0);
    pos = calibration * pos;
    gl_Position = gl_ProjectionMatrix *  gl_ModelViewMatrix * pos;
    gl_FrontColor = gl_Color;
    gl_PointSize = 4.0;
    gl_TexCoord[0] = gl_MultiTexCoord0;
}

