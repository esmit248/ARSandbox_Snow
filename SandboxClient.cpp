/***********************************************************************
SandboxClient - Vrui application connect to a remote AR Sandbox and
render its bathymetry and water level.
Copyright (c) 2019-2021 Oliver Kreylos

This file is part of the Augmented Reality Sandbox (SARndbox).

The Augmented Reality Sandbox is free software; you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The Augmented Reality Sandbox is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Augmented Reality Sandbox; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#include "SandboxClient.h"

#include <string>
#include <stdexcept>
#include <iostream>
#include <Misc/SizedTypes.h>
#include <Misc/PrintInteger.h>
#include <Misc/FunctionCalls.h>
#include <Comm/TCPPipe.h>
#include <Math/Math.h>
#include <Geometry/LinearUnit.h>
#include <GL/gl.h>
#include <GL/GLMaterialTemplates.h>
#include <GL/GLLightTracker.h>
#include <GL/GLContextData.h>
#include <GL/Extensions/GLARBMultitexture.h>
#include <GL/Extensions/GLARBTextureRectangle.h>
#include <GL/Extensions/GLARBTextureFloat.h>
#include <GL/Extensions/GLARBTextureRg.h>
#include <GL/Extensions/GLARBVertexBufferObject.h>
#include <GL/Extensions/GLARBVertexShader.h>
#include <GL/Extensions/GLARBFragmentShader.h>
#include <GL/GLModels.h>
#include <GL/GLGeometryWrappers.h>
#include <Vrui/Viewer.h>
#include <Vrui/CoordinateManager.h>
#include <Vrui/Lightsource.h>
#include <Vrui/LightsourceManager.h>
#include <Vrui/ToolManager.h>

/****************************************************
Static eleemnts of class SandboxClient::TeleportTool:
****************************************************/

SandboxClient::TeleportToolFactory* SandboxClient::TeleportTool::factory=0;

/********************************************
Methods of class SandboxClient::TeleportTool:
********************************************/

void SandboxClient::TeleportTool::applyNavState(void) const
	{
	/* Compose and apply the navigation transformation: */
	Vrui::NavTransform nav=physicalFrame;
	nav*=Vrui::NavTransform::rotate(Vrui::Rotation::rotateZ(azimuth));
	nav*=Geometry::invert(surfaceFrame);
	Vrui::setNavigationTransformation(nav);
	}

void SandboxClient::TeleportTool::initNavState(void)
	{
	/* Calculate the main viewer's current head and foot positions: */
	Point headPos=Vrui::getMainViewer()->getHeadPosition();
	footPos=Vrui::calcFloorPoint(headPos);
	headHeight=Geometry::dist(headPos,footPos);
	
	/* Set up a physical navigation frame around the main viewer's current head position: */
	calcPhysicalFrame(headPos);
	
	/* Calculate the initial environment-aligned surface frame in navigation coordinates: */
	surfaceFrame=Vrui::getInverseNavigationTransformation()*physicalFrame;
	Vrui::NavTransform newSurfaceFrame=surfaceFrame;
	
	/* Align the initial frame with the application's surface and calculate Euler angles: */
	AlignmentData ad(surfaceFrame,newSurfaceFrame,Vrui::getMeterFactor()*Scalar(0.25),Vrui::getMeterFactor());
	Scalar elevation,roll;
	align(ad,azimuth,elevation,roll);
	
	/* Move the physical frame to the foot position, and adjust the surface frame accordingly: */
	newSurfaceFrame*=Geometry::invert(physicalFrame)*Vrui::NavTransform::translate(footPos-headPos)*physicalFrame;
	physicalFrame.leftMultiply(Vrui::NavTransform::translate(footPos-headPos));
	
	/* Apply the initial navigation state: */
	surfaceFrame=newSurfaceFrame;
	applyNavState();
	}

void SandboxClient::TeleportTool::initClass(void)
	{
	/* Create a factory object for the teleporting tool class: */
	factory=new TeleportToolFactory("TeleportTool","Teleport",Vrui::getToolManager()->loadClass("SurfaceNavigationTool"),*Vrui::getToolManager());
	
	/* Set the teleport tool class' input layout: */
	factory->setNumButtons(2);
	factory->setButtonFunction(0,"Toggle");
	factory->setButtonFunction(1,"Teleport");
	
	/* Register the teleport tool class with Vrui's tool manager: */
	Vrui::getToolManager()->addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	}

SandboxClient::TeleportTool::TeleportTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::SurfaceNavigationTool(factory,inputAssignment),
	 cast(false)
	{
	sphereRenderer.setVariableRadius();
	cylinderRenderer.setVariableRadius();
	}

SandboxClient::TeleportTool::~TeleportTool(void)
	{
	}

const Vrui::ToolFactory* SandboxClient::TeleportTool::getFactory(void) const
	{
	return factory;
	}

void SandboxClient::TeleportTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	switch(buttonSlotIndex)
		{
		case 0:
			if(cbData->newButtonState) // Button has just been pressed
				{
				/* Act depending on this tool's current state: */
				if(isActive())
					{
					if(!cast)
						{
						/* Deactivate this tool: */
						deactivate();
						}
					}
				else
					{
					/* Try activating this tool: */
					if(activate())
						{
						/* Initialize the navigation state: */
						initNavState();
						}
					}
				}
			
			break;
		
		case 1:
			if(isActive())
				{
				if(cbData->newButtonState)
					cast=true;
				else
					{
					/* Teleport to the end of the cast arc: */
					surfaceFrame.leftMultiply(Vrui::NavTransform::translate(castArc.back()-surfaceFrame.getOrigin()));
					
					cast=false;
					}
				}
			
			break;
		}
	}

void SandboxClient::TeleportTool::frame(void)
	{
	if(isActive())
		{
		/* Calculate the new head and foot positions: */
		Point newHead=Vrui::getMainViewer()->getHeadPosition();
		Point newFootPos=Vrui::calcFloorPoint(newHead);
		headHeight=Geometry::dist(newHead,newFootPos);
		
		/* Create a physical navigation frame around the new foot position: */
		calcPhysicalFrame(newFootPos);
		
		/* Calculate the movement from walking: */
		Vector move=newFootPos-footPos;
		footPos=newFootPos;
		
		/* Transform the movement vector from physical space to the physical navigation frame: */
		move=physicalFrame.inverseTransform(move);
		
		/* Rotate by the current azimuth angle: */
		move=Vrui::Rotation::rotateZ(-azimuth).transform(move);
		
		/* Move the surface frame: */
		Vrui::NavTransform newSurfaceFrame=surfaceFrame;
		newSurfaceFrame*=Vrui::NavTransform::translate(move);
		
		/* Re-align the surface frame with the surface: */
		AlignmentData ad(surfaceFrame,newSurfaceFrame,Vrui::getMeterFactor()*Scalar(0.25),Vrui::getMeterFactor());
		align(ad);
		
		/* Apply the newly aligned surface frame: */
		surfaceFrame=newSurfaceFrame;
		applyNavState();
		
		if(cast)
			{
			/* Cast an arc from the current input device position: */
			castArc.clear();
			Point cp=Vrui::getInverseNavigationTransformation().transform(getButtonDevicePosition(1));
			Vector cv=Vrui::getInverseNavigationTransformation().transform(getButtonDeviceRayDirection(1)*(Vrui::getMeterFactor()*Scalar(15)));
			Vector ca(0,0,-Vrui::getInverseNavigationTransformation().getScaling()*Vrui::getMeterFactor()*Scalar(9.81));
			for(int i=0;i<100;++i)
				{
				castArc.push_back(cp);
				
				Point cpn=cp+cv*Scalar(0.05);
				cv+=ca*Scalar(0.05);
				Scalar lambda=application->intersectLine(cp,cpn);
				if(lambda<Scalar(1))
					{
					castArc.push_back(Geometry::affineCombination(cp,cpn,lambda));
					break;
					}
				
				cp=cpn;
				}
			}
		}
	}

void SandboxClient::TeleportTool::display(GLContextData& contextData) const
	{
	if(isActive()&&cast)
		{
		/* Draw the cast arc: */
		Vrui::goToNavigationalSpace(contextData);
		Scalar radius=Vrui::getInchFactor()*Scalar(1)*Vrui::getInverseNavigationTransformation().getScaling();
		
		glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(0.0f,1.0f,0.0f));
		glMaterialSpecular(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(0.333f,0.333f,0.333f));
		glMaterialShininess(GLMaterialEnums::FRONT,32.0f);
		glMaterialEmission(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(1.0f,0.0f,0.0f));
		
		sphereRenderer.enable(Vrui::getNavigationTransformation().getScaling(),contextData);
		glBegin(GL_POINTS);
		for(std::vector<Point>::const_iterator caIt=castArc.begin();caIt!=castArc.end();++caIt)
			glVertex4f((*caIt)[0],(*caIt)[1],(*caIt)[2],radius);
		glVertex4f(castArc.back()[0],castArc.back()[1],castArc.back()[2],Vrui::getMeterFactor()*Scalar(0.125)*Vrui::getInverseNavigationTransformation().getScaling());
		glEnd();
		sphereRenderer.disable(contextData);
		
		cylinderRenderer.enable(Vrui::getNavigationTransformation().getScaling(),contextData);
		glBegin(GL_LINE_STRIP);
		for(std::vector<Point>::const_iterator caIt=castArc.begin();caIt!=castArc.end();++caIt)
			glVertex4f((*caIt)[0],(*caIt)[1],(*caIt)[2],radius);
		glEnd();
		cylinderRenderer.disable(contextData);
		
		glPopMatrix();
		}
	}

/****************************************
Methods of class SandboxClient::DataItem:
****************************************/

SandboxClient::DataItem::DataItem(void)
	:bathymetryTexture(0),waterTexture(0),textureVersion(0),
	 bathymetryVertexBuffer(0),bathymetryIndexBuffer(0),
	 waterVertexBuffer(0),waterIndexBuffer(0),
	 bathymetryVertexShader(0),bathymetryFragmentShader(0),bathymetryShaderProgram(0),
	 waterVertexShader(0),waterFragmentShader(0),waterShaderProgram(0)
	{
	/* Initialize required OpenGL extensions: */
	GLARBMultitexture::initExtension();
	GLARBTextureRectangle::initExtension();
	GLARBTextureFloat::initExtension();
	GLARBTextureRg::initExtension();
	GLARBVertexBufferObject::initExtension();
	GLARBShaderObjects::initExtension();
	GLARBVertexShader::initExtension();
	GLARBFragmentShader::initExtension();
	
	/* Create texture objects: */
	glGenTextures(1,&bathymetryTexture);
	glGenTextures(1,&waterTexture);
	
	/* Create buffer objects: */
	glGenBuffersARB(1,&bathymetryVertexBuffer);
	glGenBuffersARB(1,&bathymetryIndexBuffer);
	glGenBuffersARB(1,&waterVertexBuffer);
	glGenBuffersARB(1,&waterIndexBuffer);
	
	/* Create shader objects: */
	bathymetryVertexShader=glCreateShaderObjectARB(GL_VERTEX_SHADER_ARB);
	bathymetryFragmentShader=glCreateShaderObjectARB(GL_FRAGMENT_SHADER_ARB);
	bathymetryShaderProgram=glCreateProgramObjectARB();
	waterVertexShader=glCreateShaderObjectARB(GL_VERTEX_SHADER_ARB);
	waterFragmentShader=glCreateShaderObjectARB(GL_FRAGMENT_SHADER_ARB);
	waterShaderProgram=glCreateProgramObjectARB();
	
	/* Attach shader objects to the shader program: */
	glAttachObjectARB(bathymetryShaderProgram,bathymetryVertexShader);
	glAttachObjectARB(bathymetryShaderProgram,bathymetryFragmentShader);
	glAttachObjectARB(waterShaderProgram,waterVertexShader);
	glAttachObjectARB(waterShaderProgram,waterFragmentShader);
	}

SandboxClient::DataItem::~DataItem(void)
	{
	/* Destroy objects: */
	glDeleteTextures(1,&bathymetryTexture);
	glDeleteTextures(1,&waterTexture);
	glDeleteBuffersARB(1,&bathymetryVertexBuffer);
	glDeleteBuffersARB(1,&bathymetryIndexBuffer);
	glDeleteBuffersARB(1,&waterVertexBuffer);
	glDeleteBuffersARB(1,&waterIndexBuffer);
	glDeleteObjectARB(bathymetryVertexShader);
	glDeleteObjectARB(bathymetryFragmentShader);
	glDeleteObjectARB(bathymetryShaderProgram);
	glDeleteObjectARB(waterVertexShader);
	glDeleteObjectARB(waterFragmentShader);
	glDeleteObjectARB(waterShaderProgram);
	}

/******************************
Methods of class SandboxClient:
******************************/

void SandboxClient::readGrids(void)
	{
	/* Start a new set of grids: */
	GridBuffers& gb=grids.startNewValue();
	
	/* Calculate elevation quantization factors: */
	GLfloat eScale=(elevationRange[1]-elevationRange[0])/65535.0f;
	GLfloat eOffset=elevationRange[0];
	
	/* Receive the bathymetry grid: */
	GLfloat* bPtr=gb.bathymetry;
	for(GLsizei y=0;y<gridSize[1]-1;++y)
		for(GLsizei x=0;x<gridSize[0]-1;++x,++bPtr)
			*bPtr=GLfloat(pipe->read<Misc::UInt16>())*eScale+eOffset;
	
	/* Receive the water level grid: */
	GLfloat* wlPtr=gb.waterLevel;
	for(GLsizei y=0;y<gridSize[1];++y)
		for(GLsizei x=0;x<gridSize[0];++x,++wlPtr)
			*wlPtr=GLfloat(pipe->read<Misc::UInt16>())*eScale+eOffset;
	
	/* Post the new set of grids: */
	grids.postNewValue();
	}

SandboxClient::Scalar SandboxClient::intersectLine(const SandboxClient::Point& p0,const SandboxClient::Point& p1) const
	{
	/* Convert the points to grid coordinates: */
	Point gp0(p0[0]/Scalar(cellSize[0])-Scalar(0.5),p0[1]/Scalar(cellSize[1])-Scalar(0.5),p0[2]);
	Point gp1(p1[0]/Scalar(cellSize[0])-Scalar(0.5),p1[1]/Scalar(cellSize[1])-Scalar(0.5),p1[2]);
	Vector gd=gp1-gp0;
	
	/* Clip the line segment against the grid's boundaries: */
	Scalar l0(0);
	Scalar l1(1);
	for(int i=0;i<2;++i)
		{
		/* Clip against the lower boundary: */
		Scalar b(0);
		if(gp0[i]<b)
			{
			if(gp1[i]>b)
				l0=Math::max(l0,(b-gp0[i])/gd[i]);
			else
				return Scalar(1);
			}
		else if(gp1[i]<b)
			{
			if(gp0[i]>b)
				l1=Math::min(l1,(b-gp0[i])/gd[i]);
			else
				return Scalar(1);
			}
		
		/* Clip against the upper boundary: */
		b=Scalar(gridSize[i]-2);
		if(gp0[i]>b)
			{
			if(gp1[i]<b)
				l0=Math::max(l0,(b-gp0[i])/gd[i]);
			else
				return Scalar(1);
			}
		else if(gp1[i]>b)
			{
			if(gp0[i]<b)
				l1=Math::min(l1,(b-gp0[i])/gd[i]);
			else
				return Scalar(1);
			}
		}
	if(l0>=l1)
		return Scalar(1);
	
	/* Find the grid cell containing the first point: */
	Point gp=Geometry::affineCombination(gp0,gp1,l0);
	GLsizei cp[2];
	for(int i=0;i<2;++i)
		cp[i]=Math::clamp(GLsizei(Math::floor(gp[i])),GLsizei(0),gridSize[i]-3);
	Scalar cl0=l0;
	while(cl0<l1)
		{
		/* Calculate the line parameter where the line segment leaves the current cell: */
		Scalar cl1=l1;
		int exit=-1;
		for(int i=0;i<2;++i)
			{
			Scalar el=cl1;
			if(gp0[i]<gp1[i])
				el=(Scalar(cp[i]+1)-gp0[i])/gd[i];
			else if(gp0[i]>gp1[i])
				el=(Scalar(cp[i])-gp0[i])/gd[i];
			if(cl1>el)
				{
				cl1=el;
				exit=i;
				}
			}
		
		/* Intersect the line segment with the surface inside the current cell: */
		const GLfloat* cell=grids.getLockedValue().bathymetry+(cp[1]*(gridSize[0]-1)+cp[0]);
		Scalar c0=cell[0];
		Scalar c1=cell[1];
		Scalar c2=cell[gridSize[0]-1];
		Scalar c3=cell[gridSize[0]];
		Scalar cx0=Scalar(cp[0]);
		Scalar cx1=Scalar(cp[0]+1);
		Scalar cy0=Scalar(cp[1]);
		Scalar cy1=Scalar(cp[1]+1);
		Scalar fxy=c0-c1+c3-c2;
		Scalar fx=(c1-c0)*cy1-(c3-c2)*cy0;
		Scalar fy=(c2-c0)*cx1-(c3-c1)*cx0;
		Scalar f=(c0*cx1-c1*cx0)*cy1-(c2*cx1-c3*cx0)*cy0;
		Scalar a=fxy*gd[0]*gd[1];
		Scalar bc0=(fxy*gp0[1]+fx);
		Scalar bc1=(fxy*gp0[0]+fy);
		Scalar b=bc0*gd[0]+bc1*gd[1]-gd[2];
		Scalar c=bc0*gp0[0]+bc1*gp0[1]-gp0[2]-fxy*gp0[0]*gp0[1]+f;
		Scalar il=cl1;
		if(a!=Scalar(0))
			{
			/* Solve the quadratic equation and use the smaller valid solution: */
			Scalar det=b*b-Scalar(4)*a*c;
			if(det>=Scalar(0))
				{
				det=Math::sqrt(det);
				if(a>Scalar(0))
					{
					/* Test the smaller intersection first: */
					il=b>=Scalar(0)?(-b-det)/(Scalar(2)*a):(Scalar(2)*c)/(-b+det);
					if(il<cl0)
						il=b>=Scalar(0)?(Scalar(2)*c)/(-b-det):(-b+det)/(Scalar(2)*a);
					}
				else
					{
					/* Test the smaller intersection first: */
					il=b>=Scalar(0)?(Scalar(2)*c)/(-b-det):(-b+det)/(Scalar(2)*a);
					if(il<cl0)
						il=b>=Scalar(0)?(-b-det)/(Scalar(2)*a):(Scalar(2)*c)/(-b+det);
					}
				}
			}
		else
			{
			/* Solve the linear equation: */
			il=-c/b;
			}
		
		/* Check if the intersection is valid: */
		if(il>=cl0&&il<cl1)
			return il;
		
		/* Go to the next cell: */
		if(exit>=0)
			{
			if(gd[exit]<Scalar(0))
				--cp[exit];
			else
				++cp[exit];
			}
		cl0=cl1;
		}
	
	return Scalar(1);
	}

bool SandboxClient::serverMessageCallback(Threads::EventDispatcher::ListenerKey eventKey,int eventType,void* userData)
	{
	SandboxClient* thisPtr=static_cast<SandboxClient*>(userData);
	
	try
		{
		/* Read a new set of grids: */
		thisPtr->readGrids();
		
		/* Wake up the main thread: */
		Vrui::requestUpdate();
		}
	catch(const std::runtime_error& err)
		{
		}
	
	return false;
	}

void* SandboxClient::communicationThreadMethod(void)
	{
	/* Wait for messages from the remote AR Sandbox until interrupted: */
	while(dispatcher.dispatchNextEvent())
		{
		}
	
	return 0;
	}

void SandboxClient::alignSurfaceFrame(Vrui::SurfaceNavigationTool::AlignmentData& alignmentData)
	{
	/* Get the frame's base point: */
	Point base=alignmentData.surfaceFrame.getOrigin();
	
	/* Snap the base point to the terrain: */
	GLfloat* bathymetry=grids.getLockedValue().bathymetry;
	Scalar dx=base[0]/Scalar(cellSize[0])-Scalar(0.5);
	GLsizei gx=Math::clamp(GLsizei(Math::floor(dx)),GLsizei(0),gridSize[0]-GLsizei(3));
	dx-=gx;
	Scalar dy=base[1]/Scalar(cellSize[1])-Scalar(0.5);
	GLsizei gy=Math::clamp(GLsizei(Math::floor(dy)),GLsizei(0),gridSize[1]-GLsizei(3));
	dy-=gy;
	GLfloat* cell=bathymetry+(gy*(gridSize[0]-1)+gx);
	Scalar b0=cell[0]*(Scalar(1)-dx)+cell[1]*dx;
	cell+=gridSize[0]-1;
	Scalar b1=cell[0]*(Scalar(1)-dx)+cell[1]*dx;
	base[2]=b0*(Scalar(1)-dy)+b1*dy;
	
	/* Align the frame with the bathymetry surface's x and y directions: */
	alignmentData.surfaceFrame=Vrui::NavTransform(base-Point::origin,Vrui::Rotation::identity,alignmentData.surfaceFrame.getScaling());
	}

void SandboxClient::compileShaders(SandboxClient::DataItem* dataItem,const GLLightTracker& lightTracker) const
	{
	/* Create the bathymetry vertex shader source code: */
	std::string bathymetryVertexShaderDefines="\
	#extension GL_ARB_texture_rectangle : enable\n";
	std::string bathymetryVertexShaderFunctions;
	std::string bathymetryVertexShaderUniforms="\
	uniform sampler2DRect bathymetrySampler; // Sampler for the bathymetry texture\n\
	uniform vec2 bathymetryCellSize; // Cell size of the bathymetry grid\n";
	std::string bathymetryVertexShaderVaryings="\
	varying float dist; // Eye-space distance to vertex for fogging\n";
	std::string bathymetryVertexShaderMain="\
	void main()\n\
		{\n\
		/* Get the vertex's grid-space z coordinate from the bathymetry texture: */\n\
		vec4 vertexGc=gl_Vertex;\n\
		vertexGc.z=texture2DRect(bathymetrySampler,vertexGc.xy).r;\n\
		\n\
		/* Calculate the vertex's grid-space normal vector: */\n\
		vec3 normalGc;\n\
		normalGc.x=(texture2DRect(bathymetrySampler,vec2(vertexGc.x-1.0,vertexGc.y)).r-texture2DRect(bathymetrySampler,vec2(vertexGc.x+1.0,vertexGc.y)).r)*bathymetryCellSize.y;\n\
		normalGc.y=(texture2DRect(bathymetrySampler,vec2(vertexGc.x,vertexGc.y-1.0)).r-texture2DRect(bathymetrySampler,vec2(vertexGc.x,vertexGc.y+1.0)).r)*bathymetryCellSize.x;\n\
		normalGc.z=2.0*bathymetryCellSize.x*bathymetryCellSize.y;\n\
		\n\
		/* Transform the vertex and its normal vector from grid space to eye space for illumination: */\n\
		vertexGc.x*=bathymetryCellSize.x;\n\
		vertexGc.y*=bathymetryCellSize.y;\n\
		vec4 vertexEc=gl_ModelViewMatrix*vertexGc;\n\
		vec3 normalEc=normalize(gl_NormalMatrix*normalGc);\n\
		\n\
		/* Initialize the vertex color accumulators: */\n\
		vec4 ambDiff=gl_LightModel.ambient*gl_FrontMaterial.ambient;\n\
		vec4 spec=vec4(0.0,0.0,0.0,0.0);\n\
		\n\
		/* Accumulate all enabled light sources: */\n";
	
	/* Create light application functions for all enabled light sources: */
	for(int lightIndex=0;lightIndex<lightTracker.getMaxNumLights();++lightIndex)
		if(lightTracker.getLightState(lightIndex).isEnabled())
			{
			/* Create the light accumulation function: */
			bathymetryVertexShaderFunctions+=lightTracker.createAccumulateLightFunction(lightIndex);
			
			/* Call the light application function from the bathymetry vertex shader's main function: */
			bathymetryVertexShaderMain+="\
			accumulateLight";
			char liBuffer[12];
			bathymetryVertexShaderMain.append(Misc::print(lightIndex,liBuffer+11));
			bathymetryVertexShaderMain+="(vertexEc,normalEc,gl_FrontMaterial.ambient,gl_FrontMaterial.diffuse,gl_FrontMaterial.specular,gl_FrontMaterial.shininess,ambDiff,spec);\n";
			}
	
	/* Finalize the bathymetry vertex shader's main function: */
	bathymetryVertexShaderMain+="\
		dist=length(vertexEc.xyz);\n\
		gl_FrontColor=ambDiff+spec;\n\
		gl_Position=gl_ModelViewProjectionMatrix*vertexGc;\n\
		}\n";
	
	/* Compile the bathymetry vertex shader: */
	glCompileShaderFromStrings(dataItem->bathymetryVertexShader,5,bathymetryVertexShaderDefines.c_str(),bathymetryVertexShaderFunctions.c_str(),bathymetryVertexShaderUniforms.c_str(),bathymetryVertexShaderVaryings.c_str(),bathymetryVertexShaderMain.c_str());
	
	/* Create the bathymetry fragment shader source code: */
	std::string bathymetryFragmentShaderMain="\
	uniform vec4 waterColor; // Color of water surface for fogging\n\
	uniform float waterOpacity; // Opacity of water for fogging\n\
	\n\
	varying float dist; // Eye-space distance to vertex for fogging\n\
	\n\
	void main()\n\
		{\n\
		gl_FragColor=mix(waterColor,gl_Color,exp(-dist*waterOpacity));\n\
		}\n";
	
	/* Compile the bathymetry fragment shader: */
	glCompileShaderFromString(dataItem->bathymetryFragmentShader,bathymetryFragmentShaderMain.c_str());
	
	/* Link the bathymetry shader program: */
	glLinkAndTestShader(dataItem->bathymetryShaderProgram);
	
	/* Retrieve the bathymetry shader program's uniform variable locations: */
	dataItem->bathymetryShaderUniforms[0]=glGetUniformLocationARB(dataItem->bathymetryShaderProgram,"bathymetrySampler");
	dataItem->bathymetryShaderUniforms[1]=glGetUniformLocationARB(dataItem->bathymetryShaderProgram,"bathymetryCellSize");
	dataItem->bathymetryShaderUniforms[2]=glGetUniformLocationARB(dataItem->bathymetryShaderProgram,"waterColor");
	dataItem->bathymetryShaderUniforms[3]=glGetUniformLocationARB(dataItem->bathymetryShaderProgram,"waterOpacity");
	
	/* Create the water surface vertex shader source code: */
	std::string waterVertexShaderDefines="\
	#extension GL_ARB_texture_rectangle : enable\n";
	std::string waterVertexShaderFunctions;
	std::string waterVertexShaderUniforms="\
	uniform sampler2DRect bathymetrySampler; // Sampler for the bathymetry texture\n\
	uniform sampler2DRect waterSampler; // Sampler for the water surface texture\n\
	uniform vec2 waterCellSize; // Cell size of the water surface grid\n";
	std::string waterVertexShaderMain="\
	void main()\n\
		{\n\
		/* Get the vertex's grid-space z coordinate from the water surface texture: */\n\
		vec4 vertexGc=gl_Vertex;\n\
		vertexGc.z=texture2DRect(waterSampler,vertexGc.xy).r;\n\
		\n\
		/* Get the bathymetry elevation at the same location: */\n\
		float bathy=(texture2DRect(bathymetrySampler,vertexGc.xy-vec2(1.0,1.0)).r\n\
		            +texture2DRect(bathymetrySampler,vertexGc.xy-vec2(1.0,0.0)).r\n\
		            +texture2DRect(bathymetrySampler,vertexGc.xy-vec2(0.0,1.0)).r\n\
		            +texture2DRect(bathymetrySampler,vertexGc.xy-vec2(0.0,0.0)).r)*0.25;\n\
		\n\
		/* Calculate the vertex's grid-space normal vector: */\n\
		vec3 normalGc;\n\
		normalGc.x=(texture2DRect(waterSampler,vec2(vertexGc.x-1.0,vertexGc.y)).r-texture2DRect(waterSampler,vec2(vertexGc.x+1.0,vertexGc.y)).r)*waterCellSize.y;\n\
		normalGc.y=(texture2DRect(waterSampler,vec2(vertexGc.x,vertexGc.y-1.0)).r-texture2DRect(waterSampler,vec2(vertexGc.x,vertexGc.y+1.0)).r)*waterCellSize.x;\n\
		normalGc.z=1.0*waterCellSize.x*waterCellSize.y;\n\
		\n\
		/* Transform the vertex and its normal vector from grid space to eye space for illumination: */\n\
		vertexGc.x=(vertexGc.x-0.5)*waterCellSize.x;\n\
		vertexGc.y=(vertexGc.y-0.5)*waterCellSize.y;\n\
		vec4 vertexEc=gl_ModelViewMatrix*vertexGc;\n\
		vec3 normalEc=normalize(gl_NormalMatrix*normalGc);\n\
		\n\
		/* Initialize the vertex color accumulators: */\n\
		vec4 ambDiff=gl_LightModel.ambient*gl_FrontMaterial.ambient;\n\
		vec4 spec=vec4(0.0,0.0,0.0,0.0);\n\
		\n\
		/* Accumulate all enabled light sources: */\n";
	
	/* Create light application functions for all enabled light sources: */
	for(int lightIndex=0;lightIndex<lightTracker.getMaxNumLights();++lightIndex)
		if(lightTracker.getLightState(lightIndex).isEnabled())
			{
			/* Create the light accumulation function: */
			waterVertexShaderFunctions+=lightTracker.createAccumulateLightFunction(lightIndex);
			
			/* Call the light application function from the bathymetry vertex shader's main function: */
			waterVertexShaderMain+="\
			accumulateLight";
			char liBuffer[12];
			waterVertexShaderMain.append(Misc::print(lightIndex,liBuffer+11));
			waterVertexShaderMain+="(vertexEc,normalEc,gl_FrontMaterial.ambient,gl_FrontMaterial.diffuse,gl_FrontMaterial.specular,gl_FrontMaterial.shininess,ambDiff,spec);\n";
			}
	
	/* Finalize the water vertex shader's main function: */
	waterVertexShaderMain+="\
		gl_FrontColor=vec4(ambDiff.xyz+spec.xyz,(vertexGc.z-bathy)*2.0);\n\
		gl_BackColor=gl_FrontColor;\n\
		gl_Position=gl_ModelViewProjectionMatrix*vertexGc;\n\
		}\n";
	
	/* Compile the water vertex shader: */
	glCompileShaderFromStrings(dataItem->waterVertexShader,4,waterVertexShaderDefines.c_str(),waterVertexShaderFunctions.c_str(),waterVertexShaderUniforms.c_str(),waterVertexShaderMain.c_str());
	
	/* Create the water fragment shader source code: */
	std::string waterFragmentShaderMain="\
	void main()\n\
		{\n\
		//if(gl_Color.a<0.005)\n\
		//	discard;\n\
		gl_FragColor=gl_Color;\n\
		}\n";
	
	/* Compile the water fragment shader: */
	glCompileShaderFromString(dataItem->waterFragmentShader,waterFragmentShaderMain.c_str());
	
	/* Link the water shader program: */
	glLinkAndTestShader(dataItem->waterShaderProgram);
	
	/* Retrieve the water shader program's uniform variable locations: */
	dataItem->waterShaderUniforms[0]=glGetUniformLocationARB(dataItem->waterShaderProgram,"bathymetrySampler");
	dataItem->waterShaderUniforms[1]=glGetUniformLocationARB(dataItem->waterShaderProgram,"waterSampler");
	dataItem->waterShaderUniforms[2]=glGetUniformLocationARB(dataItem->waterShaderProgram,"waterCellSize");
	
	/* Mark the bathymetry shader as up-to-date: */
	dataItem->lightStateVersion=lightTracker.getVersion();
	}

SandboxClient::SandboxClient(int& argc,char**& argv)
	:Vrui::Application(argc,argv),
	 pipe(0),
	 gridVersion(0),
	 sun(0),underwater(false)
	{
	/* Parse the command line: */
	const char* serverName=0;
	int serverPortId=26000;
	for(int argi=1;argi<argc;++argi)
		{
		if(argv[argi][0]=='-')
			{
			std::cerr<<"SandboxClient: Ignoring command line option "<<argv[argi]<<std::endl;
			}
		else if(serverName==0)
			serverName=argv[argi];
		else
			std::cerr<<"SandboxClient: Ignoring command line argument "<<argv[argi]<<std::endl;
		}
	if(serverName==0)
		throw std::runtime_error("SandboxClient: No server name provided");
	
	/* Connect to the AR Sandbox server: */
	pipe=new Comm::TCPPipe(serverName,serverPortId);
	
	/* Send an endianness token to the server: */
	pipe->write<Misc::UInt32>(0x12345678U);
	pipe->flush();
	
	/* Receive an endianness token from the server: */
	Misc::UInt32 token=pipe->read<Misc::UInt32>();
	if(token==0x78563412U)
		pipe->setSwapOnRead(true);
	else if(token!=0x12345678U)
		{
		delete pipe;
		throw std::runtime_error("SandboxClient: Invalid response from remote AR Sandbox");
		}
	
	try
		{
		/* Receive the remote AR Sandbox's water table grid size, cell size, and elevation range: */
		for(int i=0;i<2;++i)
			{
			gridSize[i]=pipe->read<Misc::UInt32>();
			cellSize[i]=pipe->read<Misc::Float32>();
			}
		for(int i=0;i<2;++i)
			elevationRange[i]=pipe->read<Misc::Float32>();
		
		/* Initialize the grid buffers: */
		for(int i=0;i<3;++i)
			grids.getBuffer(i).init(gridSize);
		
		/* Read the initial set of grids: */
		readGrids();
		}
	catch(const std::runtime_error& err)
		{
		/* Disconnect from the remote AR Sandbox: */
		delete pipe;
		
		/* Re-throw the exception: */
		throw;
		}
	
	/* Start listening on the TCP pipe: */
	dispatcher.addIOEventListener(pipe->getFd(),Threads::EventDispatcher::Read,serverMessageCallback,this);
	communicationThread.start(this,&SandboxClient::communicationThreadMethod);
	
	/* Set the linear unit to scale the AR Sandbox 1:100: */
	Vrui::getCoordinateManager()->setUnit(Geometry::LinearUnit(Geometry::LinearUnit::METER,0.01));
	
	/* Create a light source and disable all viewers' headlights: */
	sun=Vrui::getLightsourceManager()->createLightsource(false);
	sun->enable();
	sun->getLight().position=GLLight::Position(-0.2,0.3,1.0,0.0);
	for(int i=0;i<Vrui::getNumViewers();++i)
		Vrui::getViewer(i)->setHeadlightState(false);
	
	/* Create tool classes: */
	TeleportTool::initClass();
	}

SandboxClient::~SandboxClient(void)
	{
	/* Disconnect from the remote AR Sandbox: */
	dispatcher.stop();
	communicationThread.join();
	delete pipe;
	}

void SandboxClient::toolCreationCallback(Vrui::ToolManager::ToolCreationCallbackData* cbData)
	{
	/* Check if the new tool is a surface navigation tool: */
	Vrui::SurfaceNavigationTool* surfaceNavigationTool=dynamic_cast<Vrui::SurfaceNavigationTool*>(cbData->tool);
	if(surfaceNavigationTool!=0)
		{
		/* Set the new tool's alignment function: */
		surfaceNavigationTool->setAlignFunction(Misc::createFunctionCall(this,&SandboxClient::alignSurfaceFrame));
		}
	
	/* Call the base class method: */
	Vrui::Application::toolCreationCallback(cbData);
	}

void SandboxClient::frame(void)
	{
	/* Lock the most recent grid buffers: */
	if(grids.lockNewValue())
		++gridVersion;
	
	/* Calculate the position of the main viewer's head in grid space: */
	Point head=Vrui::getHeadPosition();
	GLfloat* waterLevel=grids.getLockedValue().waterLevel;
	Scalar dx=head[0]/Scalar(cellSize[0]);
	GLsizei gx=GLsizei(Math::floor(dx));
	dx-=gx;
	Scalar dy=head[1]/Scalar(cellSize[1]);
	GLsizei gy=GLsizei(Math::floor(dy));
	dy-=gy;
	if(gx>=0&&gx<gridSize[0]-1&&gy>=0&&gy<gridSize[1]-1)
		{
		GLfloat* cell=waterLevel+(gy*gridSize[0]+gx);
		Scalar b0=cell[0]*(Scalar(1)-dx)+cell[1]*dx;
		cell+=gridSize[0];
		Scalar b1=cell[0]*(Scalar(1)-dx)+cell[1]*dx;
		Scalar water=b0*(Scalar(1)-dy)+b1*dy;
		underwater=head[2]<=water;
		}
	else
		underwater=false;
	
	/* Send the current head position to the remote AR Sandbox: */
	Geometry::Point<Misc::Float32,3> fhead(head);
	pipe->write<Misc::UInt16>(0);
	pipe->write(fhead.getComponents(),3);
	Geometry::Vector<Misc::Float32,3> fview(Vrui::getViewDirection());
	pipe->write(fview.getComponents(),3);
	pipe->flush();
	}

void SandboxClient::display(GLContextData& contextData) const
	{
	/* Retrieve the context data item: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	/* Set up OpenGL state: */
	glPushAttrib(GL_ENABLE_BIT);
	
	/* Update the shader programs if necessary: */
	const GLLightTracker& lightTracker=*contextData.getLightTracker();
	if(dataItem->lightStateVersion!=lightTracker.getVersion())
		compileShaders(dataItem,lightTracker);
	
	/* Activate the bathymetry shader: */
	glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(0.6f,0.4f,0.1f));
	glMaterialSpecular(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(1.0f,1.0f,1.0f));
	glMaterialShininess(GLMaterialEnums::FRONT,32.0f);
	glUseProgramObjectARB(dataItem->bathymetryShaderProgram);
	
	/* Render the locked bathymetry grid: */
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,dataItem->bathymetryTexture);
	if(dataItem->textureVersion!=gridVersion)
		{
		/* Upload the new bathymetry grid: */
		glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB,0,0,0,gridSize[0]-1,gridSize[1]-1,GL_RED,GL_FLOAT,grids.getLockedValue().bathymetry);
		}
	glUniform1iARB(dataItem->bathymetryShaderUniforms[0],0);
	
	/* Bind the vertex and index buffers: */
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->bathymetryVertexBuffer);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->bathymetryIndexBuffer);
	
	glUniform2fARB(dataItem->bathymetryShaderUniforms[1],cellSize[0],cellSize[1]);
	glUniform4fARB(dataItem->bathymetryShaderUniforms[2],0.2f,0.5f,0.8f,1.0f);
	glUniform1fARB(dataItem->bathymetryShaderUniforms[3],underwater?0.1f:0.0f);
	
	/* Draw the bathymetry: */
	{
	GLVertexArrayParts::enable(Vertex::getPartsMask());
	glVertexPointer(static_cast<const Vertex*>(0));
	GLuint* indexPtr=0;
	for(GLsizei y=1;y<gridSize[1]-1;++y,indexPtr+=(gridSize[0]-1)*2)
		glDrawElements(GL_QUAD_STRIP,(gridSize[0]-1)*2,GL_UNSIGNED_INT,indexPtr);
	GLVertexArrayParts::disable(Vertex::getPartsMask());
	}
	
	/* Activate the water surface shader: */
	glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(0.2f,0.5f,0.8f));
	glMaterialSpecular(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(1.0f,1.0f,1.0f));
	glMaterialShininess(GLMaterialEnums::FRONT,64.0f);
	glUseProgramObjectARB(dataItem->waterShaderProgram);
	
	/* Render the locked water surface grid: */
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,dataItem->waterTexture);
	if(dataItem->textureVersion!=gridVersion)
		{
		/* Upload the new water surface grid: */
		glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB,0,0,0,gridSize[0],gridSize[1],GL_RED,GL_FLOAT,grids.getLockedValue().waterLevel);
		}
	glUniform1iARB(dataItem->waterShaderUniforms[1],1);
	
	/* Bind the vertex and index buffers: */
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->waterVertexBuffer);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->waterIndexBuffer);
	
	glUniform2fARB(dataItem->waterShaderUniforms[2],cellSize[0],cellSize[1]);
	
	if(underwater)
		glCullFace(GL_FRONT);
	else
		{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		}
	
	/* Draw the water surface: */
	{
	GLVertexArrayParts::enable(Vertex::getPartsMask());
	glVertexPointer(static_cast<const Vertex*>(0));
	GLuint* indexPtr=0;
	for(GLsizei y=1;y<gridSize[1];++y,indexPtr+=gridSize[0]*2)
		glDrawElements(GL_QUAD_STRIP,gridSize[0]*2,GL_UNSIGNED_INT,indexPtr);
	GLVertexArrayParts::disable(Vertex::getPartsMask());
	}
	
	if(underwater)
		glCullFace(GL_BACK);
	else
		glDisable(GL_BLEND);
	
	/* Protect the buffers and textures and deactivate the shaders: */
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,0);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,0);
	glUseProgramObjectARB(0);
	
	/* Mark the textures as up-to-date: */
	dataItem->textureVersion=gridVersion;
	
	/* Restore OpenGL state: */
	glPopAttrib();
	}

void SandboxClient::resetNavigation(void)
	{
	}

void SandboxClient::initContext(GLContextData& contextData) const
	{
	/* Create context data item and store it in the GLContextData object: */
	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);
	
	/* Create the bathymetry elevation texture: */
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,dataItem->bathymetryTexture);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_WRAP_S,GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_WRAP_T,GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB,0,GL_R32F,gridSize[0]-1,gridSize[1]-1,0,GL_RED,GL_FLOAT,0);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,0);
	
	/* Create the water surface elevation texture: */
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,dataItem->waterTexture);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_WRAP_S,GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_WRAP_T,GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB,0,GL_R32F,gridSize[0],gridSize[1],0,GL_RED,GL_FLOAT,0);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,0);
	
	/* Upload the grid of bathymetry template vertices into the vertex buffer: */
	{
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->bathymetryVertexBuffer);
	glBufferDataARB(GL_ARRAY_BUFFER_ARB,(gridSize[1]-1)*(gridSize[0]-1)*sizeof(Vertex),0,GL_STATIC_DRAW_ARB);
	Vertex* vPtr=static_cast<Vertex*>(glMapBufferARB(GL_ARRAY_BUFFER_ARB,GL_WRITE_ONLY_ARB));
	for(GLsizei y=0;y<gridSize[1]-1;++y)
		for(GLsizei x=0;x<gridSize[0]-1;++x,++vPtr)
			{
			/* Set the template vertex' position to the pixel center's position: */
			vPtr->position[0]=GLfloat(x)+0.5f;
			vPtr->position[1]=GLfloat(y)+0.5f;
			}
	glUnmapBufferARB(GL_ARRAY_BUFFER_ARB);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);
	
	/* Upload the bathymetry's triangle indices into the index buffer: */
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->bathymetryIndexBuffer);
	glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,(gridSize[1]-2)*(gridSize[0]-1)*2*sizeof(GLuint),0,GL_STATIC_DRAW_ARB);
	GLuint* iPtr=static_cast<GLuint*>(glMapBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,GL_WRITE_ONLY_ARB));
	for(GLsizei y=1;y<gridSize[1]-1;++y)
		for(GLsizei x=0;x<gridSize[0]-1;++x,iPtr+=2)
			{
			iPtr[0]=GLuint(y*(gridSize[0]-1)+x);
			iPtr[1]=GLuint((y-1)*(gridSize[0]-1)+x);
			}
	glUnmapBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
	}
	
	/* Upload the grid of water surface template vertices into the vertex buffer: */
	{
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->waterVertexBuffer);
	glBufferDataARB(GL_ARRAY_BUFFER_ARB,gridSize[1]*gridSize[0]*sizeof(Vertex),0,GL_STATIC_DRAW_ARB);
	Vertex* vPtr=static_cast<Vertex*>(glMapBufferARB(GL_ARRAY_BUFFER_ARB,GL_WRITE_ONLY_ARB));
	for(GLsizei y=0;y<gridSize[1];++y)
		for(GLsizei x=0;x<gridSize[0];++x,++vPtr)
			{
			/* Set the template vertex' position to the pixel center's position: */
			vPtr->position[0]=GLfloat(x)+0.5f;
			vPtr->position[1]=GLfloat(y)+0.5f;
			}
	glUnmapBufferARB(GL_ARRAY_BUFFER_ARB);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);
	
	/* Upload the water surface's triangle indices into the index buffer: */
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->waterIndexBuffer);
	glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,(gridSize[1]-1)*gridSize[0]*2*sizeof(GLuint),0,GL_STATIC_DRAW_ARB);
	GLuint* iPtr=static_cast<GLuint*>(glMapBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,GL_WRITE_ONLY_ARB));
	for(GLsizei y=1;y<gridSize[1];++y)
		for(GLsizei x=0;x<gridSize[0];++x,iPtr+=2)
			{
			iPtr[0]=GLuint(y*gridSize[0]+x);
			iPtr[1]=GLuint((y-1)*gridSize[0]+x);
			}
	glUnmapBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
	}
	
	/* Create the initial bathymetry and water surface shader programs: */
	compileShaders(dataItem,*contextData.getLightTracker());
	}

/*************
Main function:
*************/

VRUI_APPLICATION_RUN(SandboxClient)
