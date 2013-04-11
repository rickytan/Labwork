uniform samplerRECT lastDepth;
uniform samplerRECT currentDepth;

uniform float zNear = 0.1;
uniform float zFar = 8.0;

uniform float roleHeight = 0.0;

varying vec4 vertex;

void main(void)
{
	float layer0 = textureRect(lastDepth, gl_FragCoord.xy).r;
	float layer1 = textureRect(currentDepth, gl_FragCoord.xy).r;
	float fn = zNear * zFar;
	float f_n = zFar - zNear;
	float depth0 = fn / (zFar - layer0 * f_n);
	float depth1 = fn / (zFar - layer1 * f_n);
	gl_FragData[0] = vec4(vertex.xy, -depth0, 1.0);
	//gl_FragData[0] = vec4(.5);
	gl_FragData[1] = vec4(vertex.xy, -depth1, 1.0);
}