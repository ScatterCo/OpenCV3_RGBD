#version 120
#extension GL_ARB_texture_rectangle : enable

const float epsilon = 1e-6;
uniform sampler2D sprite;
uniform int drawNormals;

varying vec3 v;
varying vec3 N;
varying vec3 r;

varying vec3 localNormal;

uniform float t;

void main()
{

    if(drawNormals == 1.0) {
        gl_FragColor = vec4(localNormal * vec3(0.5) + vec3(0.5), 1.0);
        return;
    }

    const vec4 ambient = vec4(0.1, 0.1, 0.1, 1);
    const vec4 specular = vec4(0.7, 0.7, 0.4, 1);
    const float shinyness = 1.0;
    vec3 lightPos = vec3(cos(t) * 1000.0, sin(t * 1.25) * 1000.0, 0.0);
    
    vec3 L = normalize(lightPos - v);   
    vec3 E = normalize(-v); 
    vec3 R = normalize(-reflect(L,N));  

    // ambient term 
    vec4 Iamb = ambient;    

    // diffuse term
    vec4 Idiff = texture2D(sprite, gl_TexCoord[0].st) * gl_Color;
    Idiff *= max(dot(N,L), 0.0);
    Idiff = clamp(Idiff, 0.0, 1.0);     

    // specular term
    vec4 Ispec = specular; 
    Ispec *= pow(max(dot(R,E),0.0), shinyness);
    Ispec = clamp(Ispec, 0.0, 1.0); 

    // reflection term (fake ground reflection)
    float m = 2.0 * sqrt(r.x * r.x + r.y * r.y + (r.z + 1.0) * (r.z + 1.0) );
    float f = pow(0.5 - r.y / m, 5.0);
    vec4 Irefl = vec4(f, f, f, 1.0) * specular;

    // final color 
    gl_FragColor = Iamb + Irefl + Idiff + Ispec;    
}
