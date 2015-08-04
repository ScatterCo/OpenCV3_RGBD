#version 120
#extension GL_ARB_texture_rectangle : enable

uniform vec2 dimensions;
uniform sampler2DRect texture;
uniform sampler2DRect normalMap;
uniform vec2 pp;
uniform vec2 fov;
uniform mat4 calibration;

const float epsilon = 1e-6;

varying vec3 v;
varying vec3 N;
varying vec3 r;

varying vec3 localNormal;

void main(void)
{
    //align to texture
    vec2 halfvec = vec2(.5,.5);
    vec3 posTexel = texture2DRect(texture, floor(gl_Vertex.xy) + halfvec).rgb;
    vec4 pos = vec4(posTexel * vec3(500.0), 1.0);
    pos = calibration * pos;
    gl_Position = gl_ProjectionMatrix *  gl_ModelViewMatrix * pos;

    v = pos.xyz;

    vec3 normalTexel = texture2DRect(normalMap, floor(gl_Vertex.xy) + halfvec).xyz;
    localNormal = normalTexel;
    localNormal.z *= -1;
    N = normalize(gl_NormalMatrix * normalTexel);

    r = reflect(v, N);

    gl_FrontColor = gl_Color;
    gl_PointSize = 4.0;
    gl_TexCoord[0] = gl_MultiTexCoord0;
}

