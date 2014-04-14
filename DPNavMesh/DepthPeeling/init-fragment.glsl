precision highp float;

uniform float scale = 1.0;
uniform float znear = 1.0;
uniform float zfar  = 10.0;

varying vec4 vertex;

void main()
{
	vec3 col = vec3(1.);
	float xWorldPos = gl_TexCoord[0].x;
	float yWorldPos = gl_TexCoord[0].y;
	float diffuse = gl_TexCoord[0].z;

	float i = floor(xWorldPos * 4. * scale);
	float j = floor(yWorldPos * 4. * scale);
	col = (fmod(i, 2.0) == 0.0 )? vec3(.4,.85,.0) : vec3(1.0);
	gl_FragData[0] = vec4(col*gl_TexCoord[0].z,1.0);
	gl_FragData[1] = vec4(vertex.xy, -(znear + (zfar - znear) * gl_FragCoord.z), 1.0);
}
