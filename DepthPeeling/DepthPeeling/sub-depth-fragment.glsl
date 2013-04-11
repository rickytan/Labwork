uniform samplerRECT lastDepth;
uniform samplerRECT currentDepth;

uniform float zNear = 0.1;	// 相机最近切平面
uniform float zFar = 8.0;	// 相机最远切平面

uniform float zDist = 5.0;	// 相机到原点的距离

uniform float roleHeight = 0.0;	// 角色的高度

varying vec4 vertex;

void main(void)
{
	float layer0 = textureRect(lastDepth, gl_FragCoord.xy).r;
	float layer1 = textureRect(currentDepth, gl_FragCoord.xy).r;
	float fn = zNear * zFar;
	float f_n = zFar - zNear;
	float depth0 = fn / (zFar - layer0 * f_n);
	float depth1 = fn / (zFar - layer1 * f_n);

	gl_FragData[0] = vec4(vertex.xy, (zNear + zFar) / 2 - depth0, 1.0);
	//gl_FragData[0] = vertex;
	gl_FragData[1] = vec4(vertex.xy, (zNear + zFar) / 2 - depth1, 1.0);
}