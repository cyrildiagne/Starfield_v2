#version 150

// these are for the programmable pipeline system and are passed in
// by default from OpenFrameworks
uniform mat4 modelViewMatrix;
uniform mat4 projectionMatrix;
uniform mat4 textureMatrix;
uniform mat4 modelViewProjectionMatrix;

in vec4 position;
in vec4 color;
in vec4 normal;
in vec2 texcoord;
// this is the end of the default functionality

uniform vec3 pos[500];

// this is something we're creating for this shader
out vec2 varyingtexcoord;
out vec3 test;

void main() {
    
    // here we move the texture coordinates
    varyingtexcoord = vec2(texcoord.x, texcoord.y);
    
    vec4 p = position;
    p.xyz += pos[gl_InstanceID];
//    p.x = p.x + float(gl_InstanceID % 5) * 128.0;
//    p.y = p.y + float(gl_InstanceID / 5) * 128.0;
    gl_Position = modelViewProjectionMatrix * p;
}