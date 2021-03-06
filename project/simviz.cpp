#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include <iostream>
#include <string>

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;

const string world_file = "./resources/world.urdf";
const string robot_file = "./resources/panda_arm_hand.urdf";
const string robot_name = "PANDA";
const string camera_name = "camera_fixed";

const vector<string> robot_names = {
	"PANDA1",
	"PANDA2",
};

int n_robots = 2;

int loopFreq = 1000;

// redis keys:
// - write:
const std::string JOINT_ANGLES_KEY_1 = "sai2::cs225a::panda_robot::sensors::q1";
const std::string JOINT_VELOCITIES_KEY_1 = "sai2::cs225a::panda_robot::sensors::dq1";
const std::string PIECE_POS_KEY = "sai2::cs225a::pieces::pos";
const std::string JOINT_ANGLES_KEY_2 = "sai2::cs225a::panda_robot::sensors::q2";
const std::string JOINT_VELOCITIES_KEY_2 = "sai2::cs225a::panda_robot::sensors::dq2";

// - read
const std::string TORQUES_COMMANDED_KEY_1 = "sai2::cs225a::panda_robot::actuators::fgc1";
const std::string TORQUES_COMMANDED_KEY_2 = "sai2::cs225a::panda_robot::actuators::fgc2";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";
const std::string PIECE_NAME_KEY = "sai2::cs225a::pieces::name";


// dynamic objects information
const vector<string> object_names = {
"WRook1", "WKnight1", "WBishop1", "WQueen", "WKing", "WBishop2", "WKnight2", "WRook2", 
"WPawn1", "WPawn2", "WPawn3", "WPawn4", "WPawn5", "WPawn6", "WPawn7", "WPawn8",
"BPawn1", "BPawn2", "BPawn3", "BPawn4", "BPawn5", "BPawn6", "BPawn7", "BPawn8",
"BRook1", "BKnight1", "BBishop1", "BQueen", "BKing", "BBishop2", "BKnight2", "BRook2"};
vector<Vector3d> object_pos;
vector<Vector3d> object_lin_vel;
vector<Quaterniond> object_ori;
vector<Vector3d> object_ang_vel;
const int n_objects = object_names.size();


RedisClient redis_client;

// simulation function prototype
void simulation(vector<Sai2Model::Sai2Model*> robots, Simulation::Sai2Simulation* sim);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load robots
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	vector<Sai2Model::Sai2Model*> robots_sim;
	for(int i=0; i<n_robots; i++) {
		robots_sim.push_back(new Sai2Model::Sai2Model(robot_file, false, sim->getRobotBaseTransform(robot_names[i])));
	}

	robots_sim[0]->updateKinematics();
	robots_sim[1]->updateKinematics();

	// read joint positions, velocities, update model
	for(int i=0; i<n_robots; i++) {
	sim->getJointPositions(robot_names[i], robots_sim[i]->_q);
	sim->getJointVelocities(robot_names[i], robots_sim[i]->_dq);
	robots_sim[i]->updateKinematics();
	}

	// set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(0.0);

    // set co-efficient of friction
    sim->setCoeffFrictionStatic(0.8);
    sim->setCoeffFrictionDynamic(0.8);


	// fill in object information 
	for (int i = 0; i < n_objects; ++i) {
		Eigen::Vector3d _object_pos, _object_lin_vel, _object_ang_vel;
		Eigen::Quaterniond _object_ori;
		cout << endl << object_names[i] << endl;
		sim->getObjectPosition(object_names[i], _object_pos, _object_ori);
		cout << endl << object_names[i] << endl;
		sim->getObjectVelocity(object_names[i], _object_lin_vel, _object_ang_vel);
		object_pos.push_back(_object_pos);
		object_lin_vel.push_back(_object_lin_vel);
		object_ori.push_back(_object_ori);
		object_ang_vel.push_back(_object_ang_vel);
	}


	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - PandaApplications", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// cache variables
	double last_cursorx, last_cursory;

	redis_client.set(CONTROLLER_RUNING_KEY, "0");
	redis_client.set(PIECE_NAME_KEY, "WKing");
	fSimulationRunning = true;
	thread sim_thread(simulation, robots_sim, sim);
	
	// while window is open:
	while (fSimulationRunning)
	{

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_names[0], robots_sim[0]);
		graphics->updateGraphics(robot_names[1], robots_sim[1]);

		for (int i = 0; i < n_objects; ++i) {
			graphics->updateObjectGraphics(object_names[i], object_pos[i], object_ori[i]);
		}
		
		graphics->render(camera_name, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

		// poll for events
		glfwPollEvents();

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();
		if (fTransXp) {
			camera_pos = camera_pos + 0.05*cam_roll_axis;
			camera_lookat = camera_lookat + 0.05*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.05*cam_roll_axis;
			camera_lookat = camera_lookat - 0.05*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05*cam_up_axis;
			camera_lookat = camera_lookat + 0.05*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05*cam_up_axis;
			camera_lookat = camera_lookat - 0.05*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}	    
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
		}
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwSetWindowShouldClose(window,GL_TRUE);
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(vector<Sai2Model::Sai2Model*> robots_sim, Simulation::Sai2Simulation* sim) {

	int dof = robots_sim[0]->dof();
	VectorXd command_torques1 = VectorXd::Zero(dof);
	VectorXd command_torques2 = VectorXd::Zero(dof);
	VectorXd gravity1 = VectorXd::Zero(dof);
	VectorXd gravity2 = VectorXd::Zero(dof);
	redis_client.setEigenMatrixJSON(TORQUES_COMMANDED_KEY_1, command_torques1);
	redis_client.setEigenMatrixJSON(TORQUES_COMMANDED_KEY_2, command_torques2);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(loopFreq); 
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long simulation_counter = 0;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		if(redis_client.get(CONTROLLER_RUNING_KEY) == "1")
		{
			gravity1.setZero();
			gravity2.setZero();
		}
		else
		{
			robots_sim[0]->gravityVector(gravity1);
			robots_sim[1]->gravityVector(gravity2);
		}
		

		// read arm torques from redis

		// set torques to simulation
		command_torques1 = redis_client.getEigenMatrixJSON(TORQUES_COMMANDED_KEY_1);
		command_torques2 = redis_client.getEigenMatrixJSON(TORQUES_COMMANDED_KEY_2);
		
		
		sim->setJointTorques(robot_names[0], command_torques1 + gravity1);
		sim->setJointTorques(robot_names[1], command_torques2 + gravity2);

		// integrate forward
		// double curr_time = timer.elapsedTime();
		// double loop_dt = curr_time - last_time; 
		sim->integrate(0.001);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_names[0], robots_sim[0]->_q);
		sim->getJointVelocities(robot_names[0], robots_sim[0]->_dq);
		
		sim->getJointPositions(robot_names[1], robots_sim[1]->_q);
		sim->getJointVelocities(robot_names[1], robots_sim[1]->_dq);
		
		robots_sim[0]->updateKinematics();
		robots_sim[1]->updateKinematics();
		

		// get dynamic object positions
		for (int i = 0; i < n_objects; ++i) {
			sim->getObjectPosition(object_names[i], object_pos[i], object_ori[i]);
			sim->getObjectVelocity(object_names[i], object_lin_vel[i], object_ang_vel[i]);
		}

		// write new robot state to redis
		redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY_1, robots_sim[0]->_q);
		redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY_1, robots_sim[0]->_dq);
		redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY_2, robots_sim[1]->_q);
		redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY_2, robots_sim[1]->_dq);

		std::string target_piece = redis_client.get(PIECE_NAME_KEY);
		Vector3d target_piece_pos;
		Quaterniond target_piece_ori;
		sim->getObjectPosition(target_piece, target_piece_pos, target_piece_ori);
		redis_client.setEigenMatrixJSON(PIECE_POS_KEY, target_piece_pos);

		//update last time
		// last_time = curr_time;

		simulation_counter++;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			fSimulationRunning = false;
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			//TODO: menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}
