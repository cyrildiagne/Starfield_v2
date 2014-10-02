#version 150

uniform sampler2DRect tex0;

//in vec2 varyingtexcoord;

out vec4 outputColor;

void main (void) {
    
//    gl_FragColor = texture2D(tex, gl_TexCoord[0].st);
    outputColor = texture(tex0, gl_PointCoord*64);
//    outputColor = vec4(gl_PointCoord,1.0,1.0);
}