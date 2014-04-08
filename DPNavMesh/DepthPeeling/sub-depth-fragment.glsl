uniform samplerRECT lastDepth;
uniform samplerRECT currentDepth;

uniform float zNear = 0.1;	// 相机最近切平面
uniform float zFar = 8.0;	// 相机最远切平面

uniform float roleHeight = 0.0;	// 角色的高度

varying vec4 vertex;

void main(void)
{
	float layer0 = textureRect(lastDepth, gl_FragCoord.xy).z;
	float layer1 = textureRect(currentDepth, gl_FragCoord.xy).z;

	
	/*float fn = zNear * zFar;
	float f_n = zFar - zNear;
	float depth0 = fn / (zFar - layer0 * f_n);
	float depth1 = fn / (zFar - layer1 * f_n);
	gl_FragData[0] = vec4(vertex.xy, - depth0, 1.0);
	gl_FragData[1] = vec4(vertex.xy, - depth1, 1.0);*/
	 
	gl_FragData[0] = vec4(vertex.xy, - (zNear + (zFar - zNear) * layer0), 1.0);
	gl_FragData[1] = vec4(vertex.xy, - (zNear + (zFar - zNear) * layer1), 1.0);
	
	/*float z_n0 = 2.0 * layer0 - 1.0;
	float z_e0 = 2.0 * zNear * zFar / (zFar + zNear - z_n0 * (zFar - zNear));
	float z_n1 = 2.0 * layer0 - 1.0;
	float z_e1 = 2.0 * zNear * zFar / (zFar + zNear - z_n1 * (zFar - zNear));

	gl_FragData[0] = vec4(vertex.xy, - z_e0, 1.0);
	gl_FragData[1] = vec4(vertex.xy, - z_e1, 1.0);*/
}