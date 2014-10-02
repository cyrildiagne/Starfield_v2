#version 150

uniform sampler2DRect tex0;
in vec2 varyingtexcoord;
out vec4 outputColor;

in vec3 test;

void main (void) {
    
    outputColor = texture(tex0, varyingtexcoord);
    outputColor.a *= 0.6;
//    color.rgb *= test;
}