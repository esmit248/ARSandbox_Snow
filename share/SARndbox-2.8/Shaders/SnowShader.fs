/*Shader to add the amount of snow to the snow shader from the quantity shader*/
 
#extension GL_ARB_texture_rectangle : enable

uniform float criticalHeight;
uniform float meltRate;
uniform sampler2DRect snowSampler;
uniform sampler2DRect quantitySampler;
uniform sampler2DRect bathymetrySampler;


void main()
	{
	/*Calculate if surface height is higher than a given value*/
	vec3 snowAmt = (0.0,0.0,0.0);
	//float holdMelt;

	float B=(texture2DRect(bathymetrySampler,vec2(gl_FragCoord.x-1.0,gl_FragCoord.y-1.0)).r+
	         texture2DRect(bathymetrySampler,vec2(gl_FragCoord.x,gl_FragCoord.y-1.0)).r+
	         texture2DRect(bathymetrySampler,vec2(gl_FragCoord.x-1.0,gl_FragCoord.y)).r+
	         texture2DRect(bathymetrySampler,vec2(gl_FragCoord.xy)).r)*0.25;
	
	vec3 q=texture2DRect(quantitySampler,gl_FragCoord.xy).rgb;
	float w = q.r;
	vec3 snow = texture2DRect(snowSampler, gl_FragCoord.xy).rgb;
	float snowMelt = meltRate/100000;

	if(B >= criticalHeight){  // Above critical height
		snowAmt.r = snow.r + (w-B);
		}
	else{  // Below critial height
		snowAmt.r = snow.r; 

		//Melt if statement
		if(snowAmt.r > 0.0001)
		{
			snowAmt.b = snow.r *snowMelt; //0.00005;
			snowAmt.g = snow.g + snowAmt.b;
			snowAmt.r = snowAmt.r - snowAmt.g;
		}
		else
		{
			snowAmt.r = 0.0; //sets to 0 to prevent infintely small snow
			//snowAmt.g = 0.0;
		}
		}
	
	gl_FragColor=vec4(snowAmt,0.0);
	}