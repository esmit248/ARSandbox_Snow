/*
Shader to update quantity(water amount) based on the bathymetry height and critical Height
*/

#extension GL_ARB_texture_rectangle : enable

uniform float criticalHeight;
uniform sampler2DRect quantitySampler;
uniform sampler2DRect bathymetrySampler;

void main()
{
	// Get current bathymetry height and quantity value
	float B=(texture2DRect(bathymetrySampler,vec2(gl_FragCoord.x-1.0,gl_FragCoord.y-1.0)).r+
	         texture2DRect(bathymetrySampler,vec2(gl_FragCoord.x,gl_FragCoord.y-1.0)).r+
	         texture2DRect(bathymetrySampler,vec2(gl_FragCoord.x-1.0,gl_FragCoord.y)).r+
	         texture2DRect(bathymetrySampler,vec2(gl_FragCoord.xy)).r)*0.25;


	vec3 Q = texture2DRect(quantitySampler,gl_FragCoord.xy).rgb;
	vec3 noHu;

	gl_FragColor=vec4(Q,0.0);

	if(B >= criticalHeight)
	{  // above critial height
		noHu = vec3(B,0.0,0.0); //Removes water from quantity where snow is present
		gl_FragColor=vec4(noHu,0.0);
	}
}
