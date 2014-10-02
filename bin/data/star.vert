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

uniform float starSize;

//out vec2 varyingtexcoord;

void main() {
    
    // here we move the texture coordinates
//    varyingtexcoord = vec2(texcoord.x, texcoord.y);
    
	vec4 eyeCoord = modelViewMatrix * position;
	gl_Position = projectionMatrix * eyeCoord;
    
	float dist = sqrt(eyeCoord.x*eyeCoord.x + eyeCoord.y*eyeCoord.y + eyeCoord.z*eyeCoord.z);
	float att	 = 600.0 / dist;
    
	gl_PointSize = starSize * att - 1.5;
}