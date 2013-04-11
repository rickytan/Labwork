vec3 ShadeVertex()
{
	float diffuse = abs(normalize(gl_NormalMatrix * gl_Normal).z);
	return vec3(gl_Vertex.xy, diffuse);
}

varying vec4 vertex;

void main(void)
{
	gl_Position = ftransform();
	gl_TexCoord[0].xyz = ShadeVertex();

	vertex = gl_Vertex;
}