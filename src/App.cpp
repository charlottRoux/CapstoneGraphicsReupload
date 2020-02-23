#include "App.h"
#include "CatmullRomSpline.h"
#include <glm.hpp>
//#include <epsilon.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/epsilon.hpp>
//https://github.com/g-truc/glm/issues/11
//troubleshooting 
#include <glm/detail/type_vec.hpp>


#include <config/VRDataIndex.h>
#include <glm/gtx/orthonormalize.hpp>

using namespace basicgraphics;
using namespace std;
using namespace glm;
//Citation for general help learning how to do C++ again http://www.cplusplus.com/reference/memory/shared_ptr/operator=/
App::App(int argc, char** argv) : VRMultithreadedApp(argc, argv)
{
	// This is an important idiom for programming with MinVR.  We want to write
	// our program so that it will work across multiple VR setups (i.e. you should
	// be able to run the same application on your desktop or in a VR cave. Each setup
	// will have some form of trackers and buttons, but the events they generate
	// will be named something different depending on whether we are running in
	// a Cave or Vive or in desktop mode.  So, here in the constructor, we check
	// the name of the current VR setup file and then set the name of the event to
	// listen for accordingly.

	std::string name = getVRSetupName();
	


	if (name.find("MacCave") != string::npos) {
		// Mac Cave mode	
		_rHandTrackerEvent = "RedStylus_Move";
		_lHandTrackerEvent = "BlueStylus_Move";
		_startLineEvent = "Aimon_Joystick_Y_Down";
		_endLineEvent = "Aimon_Joystick_Y_Up";
		_creatingEvent = "Aimon_04_Up";
		_startingCreation = "Aimon_Joystick_04_Down";
		//For model manipulation consider changing starting creation into event executor button?
	}


	else if (name.find("Desktop") != string::npos) {
		// Desktop mode
		_rHandTrackerEvent = "RHandTracker_Move";
		_lHandTrackerEvent = "LHandTracker_Move";
		_startLineEvent = "KbdSpace_Down";
		_endLineEvent = "KbdSpace_Up";
		_creatingEvent = "MouseBtnLeft_Up";
		_startingCreation = "MouseBtnLeft_Down";
	}
	
	
	else {
		std::cout << "Unrecognized VR setup `" << getVRSetupName() <<
			"` -- using Cave mode events." << std::endl;
		// Assume Cave mode
		_rHandTrackerEvent = "RedStylus_Move";
		_lHandTrackerEvent = "BlueStylus_Move";
		_startLineEvent = "Aimon_Joystick_Y_Down";
		_endLineEvent = "Aimon_Joystick_Y_Up";
		_creatingEvent = "Aimon_04_Up";
		_startingCreation = "Aimon_04_Down";
	}
	double resamplePoints = 100;
	glm::vec3 * _p1, * _p2;
	int maxCount = 100;
	CatmullRomSpline<glm::vec3> _spline;
	
	_held = FALSE;
	constraintAssigned = FALSE;
	_lastTime = 0.0;
	p1 = glm::vec3(0.0, 0.0, 0.0);
	p2 = glm::vec3(0.0, 0.0, 0.0);
	
	_p1 = &p1;
	_p2 = &p2;

	_curFrameTime = 0.0;
	_lhand = mat4(1.0);
	color = vec4(1.0, 1.0, 1.0, 1.0);
	
	_aggregate = FALSE;


	//line1 = (Line(vec3(0.0, 0.0, 0.0), vec3(0.0, 0.0, 1.0), vec3(0.0, 0.0, 1.0), 2.0, vec4(1.0, 1.0, 1.0, 1.0)));
	//_testSphere.reset(new Sphere(vec3(1.0, 1.0, 1.0), 1.0, vec4(1.0, 1.0, 1.0, 1.0)));
	//Initialize the physics engine (not an important step for our project but many objects refer to the bulletDynamics World variable
	//Cave change to rebuild

	
	_collisionConfiguration = new btDefaultCollisionConfiguration();
	_dispatcher = new btCollisionDispatcher(_collisionConfiguration);
	_overlappingPairCache = new btDbvtBroadphase();
	_solver = new btSequentialImpulseConstraintSolver;
	_dynamicsWorld = new btDiscreteDynamicsWorld(_dispatcher, _overlappingPairCache, _solver, _collisionConfiguration);
	_dynamicsWorld->setGravity(btVector3(0, -32.1522, 0)); 


	// Needs to be in ft/s because the vr cave measures in ft.
	// When rendering in the VR Cave, each projector (there are 8) displays a separate window.
	// To be efficient, each window runs in its own thread and has its own opengl context (https://www.khronos.org/opengl/wiki/OpenGL_Context)
	// That means anytime  we create data in gpu memory, we need to be careful that we have separate
	// versions for each context. Here, we get the number of windows (1 if running on a desktop, more 
	// if running in the cave). For each graphics object (shaders, Meshes, Models, etc.) we create a vector
	// to hold each thread/context specific version

	numWindows = getNumWindows();
	_shader.resize(numWindows);
	_ground.resize(numWindows);
	_rHandCursor.resize(numWindows);
	_lHandCursor.resize(numWindows);
	_testSphere.resize(numWindows);
	
	//If box mesh is resized in the windows then I think all meshes should be resized. 
	_boxMesh.resize(numWindows);
	_LineMesh.resize(numWindows);
	_LineArray.resize(numWindows);
	_colorArray.resize(numWindows);
	
	
	//Modern context sensative shared ptr line array. Vs individual _LineArray Unique ptr
	_FSpineMeshArray.resize(numWindows);
	_NWindowLineArray.resize(numWindows);
	
	
	
	//_positionArray.resize(numWindows);
	//_positionArray.resize(numWindows);

}

App::~App()
{
	delete _dynamicsWorld;
	delete _solver;
	delete _overlappingPairCache;
	delete _dispatcher;
	delete _collisionConfiguration;
	shutdown();
}

void App::onButtonDown(const VRButtonEvent &event) {
	// This routine is called for all Button_Down events.  Check event.getName()
	// to see exactly which button has been pressed down

	//This is only Called once when the button is pressed Not the most useful thing for this tbh
	// You should use the event aliases that were setup in the constructor
	// (e.g. _grabOnEvent) when comparing against the event name. That way
	// when you run in the cave this method does not need to change to still
	// work with different button events (you don't have access to a keyboard or mouse
	// in the cave.
	
	cout << event.getName();
	//Changes the Colors
	color = vec4(1.0, 1.0, 1.0, 1.0);

	if (event.getName() == _startLineEvent) {
		p1 = vec3(column(_lhand, 3));
		//Maybe set a pointer here in order to change p1 on button press, Shared ptr?
		_held = TRUE;    
		
	 }
	if (event.getName() == "Aimon_Joystick_X_Down") {
		//Just call the function here?
		_aggregate = TRUE;
	}
	if (event.getName() == "Aimon_04_Down") {
		//Setsm color for user inputs 
		color = vec4(1.0, 1.0, 1.0, 1.0);
		cout << "White";
	}
	if (event.getName() == "Aimon_05_Down") {
		color = vec4(1.0, 0, 0, 1.0);
		cout << "Red";
	}
	if (event.getName() == "Aimon_06_Down") {
		color = vec4(0, 0, 1.0, 1.0);
		cout << "Blue";
	}
	if (event.getName() == "Aimon_07_Down") {
		color = vec4(0, 1.0, 0, 1.0);
		cout << "Green";
	}
	//Manual PC clear
	if (event.getName() == "KbdT_Down") {
		for (int i = 0; i < numWindows; i++) {
			_LineArray[i].clear();
			_colorArray[i].clear();
		}
	}


}




void App::onButtonUp(const VRButtonEvent &event) {
	// This routine is called for all Button_Up events.  Check event.getName()
	// to see exactly which button has been released.
	// Again, use event aliases!
	if (event.getName() == _endLineEvent && _held == TRUE) {
		//_sharedLine.reset(new Line(p1, p2, vec3(0, 1, 0), 5.0, vec4(1.0, 1.0, 1.0, 1.0)));
       //		_drawnShapes.push_back(_sharedLine);
		_held = FALSE;
		//p2 = vec3(column(_lhand, 3));
		//_positionArray.push_back(p2);
		//_drawnShapes.push_back(_testSphere);
	}
}


void App::onTrackerMove(const VRTrackerEvent &event) {
	// This routine is called for all Tracker_Move events.  Check event.getName()
	// to see exactly which tracker has moved, and then access the tracker's new
	// 4x4 transformation matrix with event->getTransform().

	if (event.getName() == _lHandTrackerEvent) {
		// This updates the left hand transform based on the tracker event.
		_lhand = glm::make_mat4(event.getTransform());
		
	}

}

void App::updateWorld(double currentTime)
{
	// This method is called once per frame independently of how many windows/threads are
	// rendering.

	//Update world is where things should be happening so it seems. 
	_lastTime = _curFrameTime;
	_curFrameTime = currentTime;

	//Build the position array while drawing the line objects.
	if (_held == TRUE) {
		_positionArray.push_back(p1);
		p2 = vec3(column(_lhand, 3));
		_positionArray.push_back(p2);
		p1 = p2;




		//Pushed the points of the previously drawn line into the Line array.
		//Also stores the relevent color informatiion. 
		if (_held == FALSE && _positionArray.size() != 0) {
			//Push back the given positionArray into the line array;


			//for (int i = 0; i < numWindows; i++) {
			//	_LineArray[i].push_back(_positionArray);
			//	_colorArray[i].push_back(color);
			//}




			//Worked out all this jank so maybe it will work an ok amount or at least try to work.
			double time = 0;
			for (int i = 0; i < _positionArray.size(); i++) {
				// Make sure we don't double up
				if (i == 0 || glm::epsilonEqual(_positionArray[i], _positionArray[i - 1], (float)0.0001) != glm::tvec3<bool>(true)) {
					if (i > 0) {
						time += glm::sqrt(glm::length((_positionArray[i] - _positionArray[i - 1])));
					}
					_spline.append(time, (dvec3)_positionArray[i]);
				}
				//setControlPoints(_positionArray, 3);
				//_positionArray.clear();
			}


			//Resample time
			double interval = _spline.totalTime() / (double)( resamplePoints - 1);

			// Begining and end might be snap points, so keep them the same to avoid floating point errors making them not match up any more
			std::vector<glm::vec3> centers;
			
			centers.push_back(_positionArray[0]);
			for (int i = 1; i < resamplePoints - 1; i++) {
				glm::dvec3 pt = _spline.evaluate(i*interval);
				centers.push_back(pt);
			}

			for (int i = 0; i < numWindows; i++) {
				_LineArray[i].push_back(centers);
				_colorArray[i].push_back(color);
			}

			_positionArray.clear();

			//Creates a new line called aggLine, which, if a user has decided to use the aggrigation function
			//(WIP) Aggline is now gonna hopefully be a spline. 

			//aggLine will be the new line that is calculated and used to place into the _Line array.
			//This overwrites all lines of the color that the user is currently selecting
			std::vector<vec3> aggLine;

			//https://www.geeksforgeeks.org/ways-copy-vector-c/
			//introduced the copy function





			if (_aggregate == TRUE) {
				aggLine = aggregateLinesOfSameColor(_LineArray[0], _colorArray[0], color);
				for (int i = 0; i < numWindows; i++) {
					for (int k = 0; k < _LineArray[i].size(); k++) {

						vec4 aggColor = _colorArray[i][k];
						if (aggColor == color) {
							_LineArray[i][k] = aggLine;
						}
					}
					_aggregate = FALSE;
				}
			}
		}
	}
}

//Some assistance with understanding how to construct the spline. 

//Sampling my lines and things
//std::vector<glm::vec3> App::resampleLines(std::vector<glm::vec3> _LineArray) {
//	double interval = _drawSpline.totalTime() / (double)(_LineArray.size() - 1);
//
//	// Begining and end might be snap points, so keep them the same to avoid floating point errors making them not match up any more
//	centers.push_back(_centerPoints[0]);
//
//	for (int i = 1; i < numSamples - 1; i++) {
//		glm::dvec3 pt = _drawSpline.evaluate(i*interval);
//		centers.push_back(pt);
//	}
//}
void App::onRenderGraphicsContext(const VRGraphicsState &renderState) {
	// This routine is called once per graphics context (i.e. once per window) at the start of the
	// rendering process.  So, this is the place to initialize textures,
	// load models, or do other operations that you only want to do once per
	// frame per window.

	// Is this the first frame that we are rendering after starting the app?
	if (renderState.isInitialRenderCall()) {

		//For windows, we need to initialize a few more things for it to recognize all of the
		// opengl calls.
#ifndef __APPLE__
		glewExperimental = GL_TRUE;
		GLenum err = glewInit();
		if (GLEW_OK != err)
		{
			std::cout << "Error initializing GLEW." << std::endl;
		}
#endif     
		glEnable(GL_DEPTH_TEST);
		glClearDepth(1.0f);
		glDepthFunc(GL_LEQUAL);
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		glEnable(GL_MULTISAMPLE);

		// This sets the background color that is used to clear the canvas
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		
		// Figure out which window is calling this method. We'll use the window id as an index for
		// the context specific graphics objects
		
		int windowId = renderState.getWindowId();

		//Initialize all needed objects based on the window ID
		reloadShaders(windowId);
		
		//_lHandCursor[windowId].reset(new Sphere(vec3(0.0), 0.1, vec4(0, 0, 1, 1)));
		
		_LineMesh[windowId].reset(new Line(vec3(0, 0, 0), vec3(0, 0, 0), vec3(1, 0, 0), 0.0, vec4(0, 0, 0, 0)));

	}
	int windowId = renderState.getWindowId();
	_lHandCursor[windowId].reset(new Sphere(vec3(0.0), 0.1, color));
	
}

void App::onRenderGraphicsScene(const VRGraphicsState &renderState) {
	// This routine is called once per eye. In VR this means this will be called
	// once for the left eye and once for the right eye per frame per window.
	// This is the place to actually draw the scene. 
	// Keep in mind that for VR, this will get called once per eye! You should not be making
	// new graphical objects here!
	// clear the canvas and other buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	// Setup the camera with a good initial position and view direction to see the table
	glm::mat4 model(1.0);

	int windowId = renderState.getWindowId();
	
	_shader[windowId]->use(); // Tell opengl we want to use this specific shader.

	_shader[windowId]->setUniform("view_mat", glm::make_mat4(renderState.getViewMatrix()));
	_shader[windowId]->setUniform("projection_mat", glm::make_mat4(renderState.getProjectionMatrix()));
	_shader[windowId]->setUniform("model_mat", model);
	_shader[windowId]->setUniform("normal_mat", mat3(transpose(inverse(model))));
	
	
	glm::vec3 eyePos = glm::make_vec3(renderState.getCameraPos());
	_shader[windowId]->setUniform("eye_world", eyePos);
	
	
	// Draw spheres for where the trackers are located

	_lHandCursor[windowId]->draw(*(_shader[windowId]), _lhand);
	
	
	//Draws all lines the user has previously drawn
	//This actually kind of needs to be done here even though its inefficiant. That is because the line is reset and not saved
	//As such, the code will need to reset the line at every instance when drawing in order to actually draw the correct Line.
	//If this was done a different way it could be placed in a more optimal position.

	for (int k = 0; k < _LineArray[windowId].size(); k++) {
		for (int i = 0; i < _LineArray[windowId][k].size(); i += 2) {
			//Defines the normal as the midpoint between the vertex positions. The normal is then drawn to the 
			//users eye Position.
			vec3 p1N = eyePos - _LineArray[windowId][k][i];
			vec3 p2N = eyePos - _LineArray[windowId][k][i+1];
			glm::vec3 normal = vec3((p1N.x + p2N.x) / 2, (p1N.y + p2N.y) / 2, (p1N.z + p2N.z) / 2);
			vec4 storedColor = _colorArray[windowId][k];
			_LineMesh[windowId].reset(new Line(_LineArray[windowId][k][i], _LineArray[windowId][k][i + 1], normal, .01, storedColor));
			_LineMesh[windowId]->draw(*(_shader[windowId]), model);
		}
	}
	//Draw Current Line
	if (_positionArray.size() > 0) {
		for (int i = 0; i < _positionArray.size(); i += 2) {
			vec3 p1N = eyePos - _positionArray[i];
			vec3 p2N = eyePos - _positionArray[i + 1];
			glm::vec3 normal = vec3((p1N.x + p2N.x) / 2, (p1N.y + p2N.y) / 2, (p1N.z + p2N.z) / 2);
			_LineMesh[windowId].reset(new Line(_positionArray[i], _positionArray[i + 1], normal, .01, color));
			_LineMesh[windowId]->draw(*(_shader[windowId]), model);
		}
	}
	

	}
void App::setControlPoints(std::vector<glm::vec3> _lineArray, int _sampleC) {
	//First steps in establishing a spline to use.
	int size = _lineArray.size();
	while(_lineArray.size()%4 !=0){
		_lineArray.push_back(_lineArray[_lineArray.size()-1]);
	}
	_spline.append(_lineArray[0]);
	_spline.append(_lineArray[size / 2]);
	_spline.append(_lineArray[size - 1]);
}


void App::reloadShaders(int windowId)
{
	_shader[windowId].reset(new GLSLProgram());
	_shader[windowId]->compileShader("texture.vert", GLSLShader::VERTEX);
	_shader[windowId]->compileShader("texture.frag", GLSLShader::FRAGMENT);
	_shader[windowId]->link();
	_shader[windowId]->use();
}

void splineAddition(std::vector<std::vector<glm::vec3>> positionArray) {

}
//Doing a simple average point comparison by distance with color matching for added depth


//for (int i = 0; i < closestPts.size(); i++) {
//	// Make sure we don't double up
//	if (i == 0 || glm::epsilonEqual(closestPts[i], closestPts[i - 1], 0.0001) != glm::detail::tvec3<bool>(true)) {
//		if (i > 0) {
//			time += glm::sqrt<double>(glm::length(closestPts[i] - closestPts[i - 1]));
//		}
//		drawSpline.append(time, closestPts[i]);
//	}

//Takes p1 and p2 and then adds them to the corrisponding line array
void lineAdd(glm::vec3 p1, glm::vec3 p2 , std::vector<std::vector<glm::vec3>> positionArray) {
}




void App::splineEval(int p) {
	//Calculates and places each point into the drawn line array position. 
	
   //for (int i = 0; i <= 2; i += .02) {}
	//	_calcSpline.push_back(_spline.evaluate(i));
	//}

}



std::vector<vec3> App::aggregateLinesOfSameColor(std::vector<std::vector<glm::vec3>> positionArray, std::vector<vec4> colorArray, glm::vec4 color)
{
	std::vector<vec4> aggregatePositionArray;
	int numOfSameColorLines = 0;
	for (int i = 0; i < positionArray.size(); i++) {
		if (color == colorArray[i] && numOfSameColorLines == 0){
			for (int k = 0; k < positionArray[i].size(); k++) {
				aggregatePositionArray.push_back(vec4 (positionArray[i][k], 1.0));
			}
			numOfSameColorLines += 1;
		}
		//Some more intense math here to go pointwise and generate an average. If there is no matching pointwise comparison,
		//Just add the points on to the end
		else if (color == colorArray[i] && numOfSameColorLines > 0) {
			//Now determine what array is bigger. The new or the old
			if (aggregatePositionArray.size() > positionArray[i].size()) {
				for (int g = 0; g < positionArray[i].size(); g++) {
					aggregatePositionArray[g] += vec4(positionArray[i][g], 1);
				}
			}
			else if (positionArray[i].size() > aggregatePositionArray.size()) {
				for (int g = 0; g < aggregatePositionArray.size(); g++) {
					aggregatePositionArray[g] += vec4(positionArray[i][g], 1);
				}
 				for (int g = aggregatePositionArray.size(); g < positionArray[i].size(); g++) {
					aggregatePositionArray.push_back(vec4(positionArray[i][g],1));
				}
			}
			else {
				for (int g = 0; g < aggregatePositionArray.size(); g++) {
					aggregatePositionArray[g] += vec4( positionArray[i][g], 1);
				}
			}
			numOfSameColorLines++;
			//First loop through the smallest one and add them together, then loop through the excess,
		}
	}
	//FinalStep is averaging all the added Points
	//Use the final term of the vec4 for this as it contains index occurances.
	std::vector <vec3> averageArray;
	for (int i = 0; i < aggregatePositionArray.size(); i++) {
		averageArray.push_back(vec3( vec3(aggregatePositionArray[i].x, aggregatePositionArray[i].y, aggregatePositionArray[i].z) / (float) aggregatePositionArray[i].w));
	}
	return averageArray;
}

