#ifndef APP_H
#define APP_H

#include <api/MinVR.h>
using namespace MinVR;

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>

#ifdef _WIN32
#include "GL/glew.h"
#include "GL/wglew.h"
#elif (!defined(__APPLE__))
#include "GL/glxew.h"
#endif

// OpenGL Headers
#if defined(WIN32)
#define NOMINMAX
#include <windows.h>
#include <GL/gl.h>
#elif defined(__APPLE__)
#define GL_GLEXT_PROTOTYPES
#include <OpenGL/gl3.h>
#include <OpenGL/glext.h>
#else
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#endif

#include <BasicGraphics.h>

#include <btBulletDynamicsCommon.h>

#include "PhysicsShape.h"
#include "GroundPlane.h"
#include "VRMultithreadedApp.h"
#include "CatmullRomSpline.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <limits>
#include <mutex>


class App : public VRMultithreadedApp {
public:
    
    /** The constructor passes argc, argv, and a MinVR config file on to VRApp.
     */
	App(int argc, char** argv);
    virtual ~App();

    
    /** USER INTERFACE CALLBACKS **/
    virtual void onButtonDown(const VRButtonEvent &state);
    virtual void onButtonUp(const VRButtonEvent &state);
    virtual void onTrackerMove(const VRTrackerEvent &state);

	virtual void updateWorld(double currentTime);
    /** RENDERING CALLBACKS **/
    virtual void onRenderGraphicsScene(const VRGraphicsState& state);
    virtual void onRenderGraphicsContext(const VRGraphicsState& state);
    
private:

    // The names of events that will trigger right hand tracking, left hand
    // tracking, grabbing a shape, or creating a shape.  These are different depending upon
    // whether you are running in a Cave, Vive, Oculus, zSpace, desktop, etc.
    std::string _rHandTrackerEvent;
    std::string _lHandTrackerEvent;
    std::string _startLineEvent;
    std::string _endLineEvent;
    std::string _creatingEvent;
	std::string _startingCreation;
	CatmullRomSpline<glm::dvec3> _spline;
	std::vector<glm::vec3> _calcSpline;
	std::vector<CatmullRomSpline<glm::dmat4>> _splineArray;

	std::vector<std::vector<vec4> > _colorArray;
	
	//Booleans for event execution
	bool _aggregate;
	bool _held;
	int _physicsCounter;
	bool _released;
	int controlCount;
	double resamplePoints;
	
	//May be depreciated
	//int boxNum;

	bool constraintAssigned;
    // Transformation matrices for the left hand and right hand
    glm::mat4 _lhand;
    glm::mat4 _rhand;
	
	shared_ptr<basicgraphics::Line> _sharedLine;
	std::vector<std::unique_ptr<basicgraphics::Sphere>> _testSphere;
	double _lastTime;
	double _curFrameTime;
	glm::vec4 color;
	int numWindows;
    virtual void reloadShaders(int windowId);
	
	
	
	
	
	//Custom Shaders for ease of use 
	std::vector<std::shared_ptr<basicgraphics::GLSLProgram> >_ezShader;
	std::vector<std::shared_ptr<basicgraphics::GLSLProgram> > _shader;
	
	
	//Ground plane
	std::vector<std::shared_ptr<GroundPlane> > _ground;
    
	
	//User input devices
	std::vector<std::unique_ptr<basicgraphics::Sphere> > _rHandCursor;
    std::vector<std::unique_ptr<basicgraphics::Sphere> > _lHandCursor;
	
	std::vector<std::shared_ptr<basicgraphics::Sphere> > _drawnShapes;
	

	//p1 p2 for frame by frame drawing positions
	
	glm::vec3  p1;
	glm::vec3  p2;
	//Corrisponding pointers.

	std::shared_ptr<glm::vec3> _p1;
	std::shared_ptr<glm::vec3> _p2;
	

	// We'll use one object as the graphical representation for each physics object
	
	
	//So is this an array or is this just a singular mesh? if so why is it in a vector?
	std::vector<std::unique_ptr<basicgraphics::Box> > _boxMesh;
	
	//CatmullRomSpline _spline = new CatmullRomSpline();
	
	std::vector<vec3>  _positionArray;
	
	// Use one object as the graphical representation of each line object;
	std::vector<std::unique_ptr<basicgraphics::Sphere> > _sphereMesh;
	
	//Stores each line in each window context. Unsure if unique is the best type of pointer. 
	std::vector<std::unique_ptr<basicgraphics::Line> > _LineMesh;
	

	//Stores each different line into the _LineArray so that it can be looped through effectivly. Basically maintains the line in this source for comparison later.
	//this is a double array due to the glcontext for each window. 
	std::vector<std::vector<std::vector<glm::vec3>> >_LineArray;

	//2020 Attempt at new line drawing implementation in order to complete this stupid thing that I am putting way too much effort into.
	
	std::vector <glm::vec3>  _NLineArray;
	std::vector <std::vector<glm::vec3>> _NWindowLineArray;
	std::vector<std::shared_ptr<basicgraphics::Line>> _NLineMeshArray;
	
	//to be added to the line drawing array
	std::shared_ptr<basicgraphics::Line> _FinalSplineMesh;

	//To be looped through
	std::vector<std::shared_ptr<basicgraphics::Line>> _FSpineMeshArray;
    

	void setControlPoints(std::vector<glm::vec3> _LineArray, int controlCount);

		//Here we set 4 control points for our line to convert into a spline.
	// objects necessary to define the physics engine configuration


    btDiscreteDynamicsWorld* _dynamicsWorld;
    btDefaultCollisionConfiguration* _collisionConfiguration;
    btCollisionDispatcher* _dispatcher;
    btBroadphaseInterface* _overlappingPairCache;
    btSequentialImpulseConstraintSolver* _solver;
    

    // Finds the closest physics shape to position and changes its color to the highlight color if it is within maxDist.
    // Resets all the other boxes' colors to white
    // returns the index into the _physicsShapes vector for the closest box  or -1 if one is not found within maxDist.
	


	std::vector<glm::vec3> aggregateLinesOfSameColor(std::vector<std::vector<glm::vec3> >positionArray, std::vector<glm::vec4> colorArray, glm::vec4 color);

	//Function call for addition into spline
	void splineEval( int maxPoints);
};


#endif //APP_H
