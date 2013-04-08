uniform samplerRECT lastDepth;
uniform samplerRECT currentDepth;

void main(void)
{
	float layer0 = textureRect(lastDepth, gl_FragCoord.xy).r;
	float layer1 = textureRect(currentDepth, gl_FragCoord.xy).r;
	gl_FragData[0] = vec4(vec3(layer1 - layer0),1.0);
	//gl_FragData[1] = vec4(.8);
	return;
	if (layer1 >= layer0)
		gl_FragColor = vec4(layer1 - layer0);
	else
		gl_FragColor = vec4(.5,1.0,0.7,1.0);
}