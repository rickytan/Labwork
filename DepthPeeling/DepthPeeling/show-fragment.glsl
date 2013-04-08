uniform samplerRECT finalTex;

void main(void)
{
	gl_FragColor = textureRect(finalTex,gl_FragCoord.xy);
	//gl_FragColor = gl_Color;
	//gl_FragColor = vec4(0.7,0.7,0.7,1.0)*gl_FragCoord.x / 1024.;
	//gl_FragColor = gl_FragCoord;
}
