void main()
{
	//gl_Position = ftransform();
	//gl_Position = gl_ModelViewMatrix * gl_Vertex;
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	//gl_TexCoord[0].xy = gl_Vertex.xy;
}