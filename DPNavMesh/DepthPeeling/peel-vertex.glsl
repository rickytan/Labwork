vec3 ShadeVertex()
{
	float diffuse = 0.8*abs(normalize(gl_NormalMatrix * gl_Normal).z);
	return vec3(gl_Vertex.xy, diffuse + 0.2);
}

varying vec4 vertex;

void main(void)
{
	gl_Position = ftransform();
	gl_TexCoord[0].xyz = ShadeVertex();
	vertex = gl_Position;
}
