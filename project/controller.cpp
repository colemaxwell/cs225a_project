#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

#include <iostream>
#include <fstream>
#include <string>

// definitions for terminal input parsing 
#define QUESTION_1A   	11
#define QUESTION_1C   	13
#define QUESTION_2D   	24
#define QUESTION_2E   	25
#define QUESTION_2F   	26
#define QUESTION_2G   	27
#define QUESTION_3    	30
#define QUESTION_4A   	41
#define QUESTION_4B   	42
#define QUESTION_5A   	51
#define QUESTION_5B   	52
#define QUESTION_5C   	53

// state machine
#define MOVING_DOWN		0
#define APPLY_FORCE		1
#define MOVING_CIRCLE	2

#define PI 3.14159265

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm_hand.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
const std::string EE_FORCE_KEY = "sai2::cs225a::panda_robot::sensors::force";
const std::string EE_MOMENT_KEY = "sai2::cs225a::panda_robot::sensors::moment";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

// helper function 
double sat(double x) {
	if (abs(x) <= 1.0) {
		return x;
	}
	else {
		return signbit(x);
	}
}

int main(int argc, char* argv[]) {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	const string ee_link_name = "link7";
	const Vector3d pos_in_ee_link = Vector3d(0, 0, 0.15);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd J0 = MatrixXd::Zero(6,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd Lambda0 = MatrixXd::Zero(6,6);
	// MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);
	MatrixXd N0 = MatrixXd::Zero(dof,dof);

	robot->Jv(Jv, ee_link_name, pos_in_ee_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	// robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);
	robot->nullspaceMatrix(N0, J0);

	// initialize force and moment on end-effector
	Eigen::Vector3d force, moment;
	// force = redis_client.getEigenMatrixJSON(EE_FORCE_KEY);
	// moment = redis_client.getEigenMatrixJSON(EE_MOMENT_KEY);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	redis_client.set(CONTROLLER_RUNING_KEY, "1");

	// for state machine
	unsigned state = MOVING_DOWN;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		Vector3d x, x_d, x_dot, x_d_dot, F, w;
		MatrixXd J_bar = MatrixXd::Zero(dof,3);
		MatrixXd J(6, dof);
		Matrix3d R;

		Eigen::VectorXd g(dof); // Empty Gravity Vector
		robot->gravityVector(g);
		Eigen::VectorXd cF(dof); // Empty Coriolis Vector
		robot->coriolisForce(cF);

		robot->Jv(Jv, ee_link_name, pos_in_ee_link);
		robot->J_0(J0, ee_link_name, pos_in_ee_link);


		robot->taskInertiaMatrix(Lambda, Jv);
		robot->taskInertiaMatrix(Lambda0, J0);

		robot->dynConsistentInverseJacobian(J_bar, Jv);
		robot->nullspaceMatrix(N, Jv);
		robot->nullspaceMatrix(N0, J0);

		robot->position(x,ee_link_name, pos_in_ee_link);
		robot->linearVelocity(x_dot,ee_link_name, pos_in_ee_link);

		robot->rotation(R,ee_link_name);
		robot->angularVelocity(w,ee_link_name, pos_in_ee_link);

		VectorXd q_desired(dof);
		q_desired <<0,0,0,0,0,0,0,0,0;


		// ---------------------------  question 1 ---------------------------------------
		
		double kp = 100;
		double kv = 20;
		double kpj = 50;
		double kvj = 14;

		x_d << (0.3 + 0.1*sin(PI * time)) , (0.1 + 0.1*cos(PI * time)) , 0.5;// set x_d

		x_d_dot << (0.1*PI*cos(PI * time)) , (-0.1*PI*sin(PI * time)) , 0;

		// calculate joint_task_torque

		// calculate F
		F = Lambda * (-kp*(x - x_d) - kv * (x_dot - x_d_dot ));

		// calculate command_torques
		command_torques = Jv.transpose() * F + N.transpose() * (-kp * (robot->_q-q_desired) -kv * (robot->_dq)) + g;
		

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
