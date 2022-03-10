/***********************************************************************
SandboxClient - Vrui application connect to a remote AR Sandbox and
render its bathymetry and water level.
Copyright (c) 2019 Oliver Kreylos

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

#ifndef SANDBOXCLIENT_INCLUDED
#define SANDBOXCLIENT_INCLUDED

#include <Threads/Thread.h>
#include <Threads/TripleBuffer.h>
#include <Threads/EventDispatcher.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/OrthogonalTransformation.h>
#include <GL/gl.h>
#include <GL/GLObject.h>
#include <GL/Extensions/GLARBShaderObjects.h>
#include <GL/GLSphereRenderer.h>
#include <GL/GLCylinderRenderer.h>
#include <GL/GLGeometryVertex.h>
#include <Vrui/Application.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/SurfaceNavigationTool.h>

/* Forward declarations: */
namespace Comm {
class TCPPipe;
}
class GLLightTracker;
namespace Vrui {
class Lightsource;
}

class SandboxClient:public Vrui::Application,public GLObject
	{
	/* Embedded classes: */
	private:
	typedef Vrui::Scalar Scalar;
	typedef Vrui::Point Point;
	typedef Vrui::Vector Vector;
	
	struct GridBuffers // Structure representing a pair of grids
		{
		/* Elements: */
		public:
		GLfloat* bathymetry;
		GLfloat* waterLevel;
		
		/* Constructors and destructors: */
		GridBuffers(void)
			:bathymetry(0),waterLevel(0)
			{
			}
		~GridBuffers(void)
			{
			delete[] bathymetry;
			delete[] waterLevel;
			}
		
		/* Methods: */
		void init(const GLsizei gridSize[2]) // Initializes the grids
			{
			bathymetry=new GLfloat[(gridSize[1]-1)*(gridSize[0]-1)];
			waterLevel=new GLfloat[gridSize[1]*gridSize[0]];
			}
		};
	
	class TeleportTool;
	typedef Vrui::GenericToolFactory<TeleportTool> TeleportToolFactory;
	
	class TeleportTool:public Vrui::SurfaceNavigationTool,public Vrui::Application::Tool<SandboxClient>
		{
		friend class Vrui::GenericToolFactory<TeleportTool>;
		
		/* Elements: */
		private:
		static TeleportToolFactory* factory; // Pointer to the factory object for this class
		
		/* Transient navigation state: */
		Point footPos; // Position of the main viewer's foot on the last frame
		Scalar headHeight; // Height of viewer's head above the foot point
		Vrui::NavTransform surfaceFrame; // Current local coordinate frame aligned to the surface in navigation coordinates
		Scalar azimuth; // Current azimuth of view relative to local coordinate frame
		
		bool cast; // Flag if the teleport button is currently pressed
		std::vector<Point> castArc; // The cast arc
		GLSphereRenderer sphereRenderer;
		GLCylinderRenderer cylinderRenderer;
		
		/* Private methods: */
		void applyNavState(void) const; // Sets the navigation transformation based on the tool's current navigation state
		void initNavState(void); // Initializes the tool's navigation state when it is activated
		
			/* Constructors and destructors: */
		public:
		static void initClass(void); // Initializes the teleport tool class's factory class
		TeleportTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
		virtual ~TeleportTool(void);
		
		/* Methods from class Vrui::Tool: */
		virtual const Vrui::ToolFactory* getFactory(void) const;
		virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
		virtual void frame(void);
		virtual void display(GLContextData& contextData) const;
		};
	
	typedef GLGeometry::Vertex<void,0,void,0,void,GLfloat,2> Vertex; // Type for grid rendering template vertices
	
	struct DataItem:public GLObject::DataItem
		{
		/* Elements: */
		public:
		GLuint bathymetryTexture; // ID of texture object holding bathymetry vertex elevations
		GLuint waterTexture; // ID of texture object holding water surface vertex elevations
		unsigned int textureVersion; // Version number of bathymetry and water grids stored in textures
		GLuint bathymetryVertexBuffer; // ID of vertex buffer object holding bathymetry's template vertices
		GLuint bathymetryIndexBuffer; // ID of index buffer object holding bathymetry's triangles
		GLuint waterVertexBuffer; // ID of vertex buffer object holding water surface's template vertices
		GLuint waterIndexBuffer; // ID of index buffer object holding water surface's triangles
		GLhandleARB bathymetryVertexShader; // Vertex shader to render the bathymetry
		GLhandleARB bathymetryFragmentShader; // Fragment shader to render the bathymetry
		GLhandleARB bathymetryShaderProgram; // Shader program to render the bathymetry
		GLint bathymetryShaderUniforms[4]; // Locations of the bathymetry shader's uniform variables
		GLhandleARB waterVertexShader; // Vertex shader to render the water surface
		GLhandleARB waterFragmentShader; // Fragment shader to render the water surface
		GLhandleARB waterShaderProgram; // Shader program to render the water surface
		GLint waterShaderUniforms[3]; // Locations of the water surface shader's uniform variables
		unsigned int lightStateVersion; // Version number for current lighting state reflected in the bathymetry and water surface shader programs
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	/* Elements: */
	Comm::TCPPipe* pipe; // TCP pipe connected to the remote AR Sandbox
	GLsizei gridSize[2]; // Width and height of the water table's cell-centered quantity grid
	GLfloat cellSize[2]; // Width and height of each water table cell
	GLfloat elevationRange[2]; // Minimum and maximum valid elevations
	Threads::EventDispatcher dispatcher; // Dispatcher for events on the TCP pipe
	Threads::Thread communicationThread; // Thread to handle communication with the remote AR Sandbox in the background
	Threads::TripleBuffer<GridBuffers> grids; // Triple buffer of bathymetry and water level grids
	unsigned int gridVersion; // Version number of currently locked grids
	Vrui::Lightsource* sun; // Light source representing the sun
	bool underwater; // Flag if the main viewer's head is currently under water
	
	/* Private methods: */
	void readGrids(void); // Reads a new set of bathymetry and water level grids from the remote AR Sandbox
	Scalar intersectLine(const Point& p0,const Point& p1) const; // Returns the intersection parameter of a line segment with the bathymetry; returns 1.0 if there is no intersection
	static bool serverMessageCallback(Threads::EventDispatcher::ListenerKey eventKey,int eventType,void* userData); // Callback called when a message arrives from the remote AR Sandbox
	void* communicationThreadMethod(void); // Method handling communication with the remote AR Sandbox in the background
	void alignSurfaceFrame(Vrui::SurfaceNavigationTool::AlignmentData& alignmentData); // Aligns the surface frame of a surface navigation tool with the bathymetry surface
	void compileShaders(DataItem* dataItem,const GLLightTracker& lightTracker) const; // Compiles the bathymetry and water surface shader programs based on current lighting state
	
	/* Constructors and destructors: */
	public:
	SandboxClient(int& argc,char**& argv);
	virtual ~SandboxClient(void);
	
	/* Methods from class Vrui::Application: */
	virtual void toolCreationCallback(Vrui::ToolManager::ToolCreationCallbackData* cbData);
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	virtual void resetNavigation(void);
	
	/* Methods from class GLObject: */
	virtual void initContext(GLContextData& contextData) const;
	};

#endif
