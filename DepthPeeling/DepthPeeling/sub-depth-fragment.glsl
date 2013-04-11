uniform samplerRECT lastDepth;
uniform samplerRECT currentDepth;

uniform float zNear = 0.1;	// ��������ƽ��
uniform float zFar = 8.0;	// �����Զ��ƽ��

uniform float zDist = 5.0;	// �����ԭ��ľ���

uniform float roleHeight = 0.0;	// ��ɫ�ĸ߶�

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