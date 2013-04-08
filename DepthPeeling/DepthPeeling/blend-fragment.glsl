uniform samplerRECT TempTex;

void main(void)
{
	gl_FragColor = textureRect(TempTex, gl_FragCoord.xy);
}