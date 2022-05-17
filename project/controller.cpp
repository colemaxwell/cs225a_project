#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

#include <iostream>
#include <fstream>
#include <string>

// state machine
#define MOVING_TO_PIECE		0
#define PICKING_DOWN		1
#define PICKING				2
#define PICKING_UP			3
#define MOVING_PIECE		4
#define PLACING_DOWN		5
#define PLACING				6
#define PLACING_UP			7
#define RETURNING			8

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
const std::string PIECE_POS_KEY = "sai2::cs225a::pieces::pos";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";
const std::string PIECE_NAME_KEY = "sai2::cs225a::pieces::name";

unsigned long long controller_counter = 0;

double grab_height = 0.04;
double over_height = 0.2;
const Vector3d home = Vector3d(0.25, 0, over_height);
Matrix3d point_down;
Vector2d target_location_board = Vector2d(3, 4);
std::string target_piece = "BPawn4";
Vector3d target_piece_pos;

int controller_mode = MOVING_TO_PIECE;

int main(int argc, char* argv[]) {

	double angle = -PI/4;
	point_down <<cos(angle), sin(angle), 0,
     			sin(angle), -cos(angle), 0,
     			0, 0, -1;

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
	const Vector3d pos_in_ee_link = Vector3d(0, 0, 0.21);
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

	VectorXd finger_mask = VectorXd(dof);
	finger_mask <<  0, 0, 0, 0, 0, 0, 0, 1, 1;	

	int failCount = 0;
	int failMax = 20;

	Vector3d piece_lock;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		target_piece_pos = redis_client.getEigenMatrixJSON(PIECE_POS_KEY);
		target_piece_pos[2] += grab_height;

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		Vector3d x, x_d, x_dot, x_d_dot, w;
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
		q_desired <<0,-0.785398,-0,-2.18166,-0,1.39626,0.1,0,0;


		// ---------------------------  controller ---------------------------------------
		Vector3d delta_phi;
		Matrix3d R_d;
		VectorXd F(6);

		double kp = 150;
		double kv = 20;
		double kpRot = 500;
		double kvRot = 50;

		double kpj = 500;
		double kvj = 50;

		double open = 0.020;
		double closed = 0;

		double fingerPos;
		Vector3d target_location;
		target_location << 0.71 - 0.10 - target_location_board[1] * 0.06, -0.21 + target_location_board[0] * 0.06, 0;

		Vector3d target_piece_pos_z0 = Vector3d(target_piece_pos[0], target_piece_pos[1], 0);

		R_d = point_down;

		if (controller_mode == MOVING_TO_PIECE){
			x_d = target_piece_pos_z0;
			x_d[2] += over_height;
			fingerPos = open;

		}else if (controller_mode == PICKING_DOWN){
			x_d = target_piece_pos_z0;
			x_d[2] += grab_height;
			fingerPos = open;
			piece_lock = target_piece_pos_z0;

		}else if (controller_mode == PICKING){
			x_d = piece_lock;
			x_d[2] += grab_height;
			fingerPos = closed;

		}else if (controller_mode == PICKING_UP){
			x_d = piece_lock;
			x_d[2] += over_height;
			fingerPos = closed;

		}else if(controller_mode == MOVING_PIECE){
			x_d = target_location;
			x_d[2] += over_height;
			fingerPos = closed;

		}else if(controller_mode == PLACING_DOWN){
			x_d = target_location;
			x_d[2] += grab_height;
			fingerPos = closed;

		}else if(controller_mode == PLACING){
			x_d = target_location;
			x_d[2] += grab_height;
			fingerPos = open;

		}else if(controller_mode == PLACING_UP){
			x_d = target_location;
			x_d[2] += over_height;
			fingerPos = open;

		}else if(controller_mode == RETURNING){
			x_d = home;
			fingerPos = open;
		}else{
			cout <<"NO MODE"<< endl;
		}

		q_desired[7] = fingerPos;
		q_desired[8] = -fingerPos;


		// calculate delta_phi
		delta_phi.setZero();
		for (int i = 0; i < 3; i++){
			delta_phi += -0.5 * R.col(i).cross(R_d.col(i)); //World fixed x,y,z
		}
		// calculate pos_d, position desired
		VectorXd pos_d(6);
		pos_d << kp*(x_d-x) - kv*x_dot, kpRot*(-delta_phi) - kvRot*w;

		// calculate F
		F = Lambda0 * pos_d;
			
		// calculate command_torques
		command_torques = J0.transpose() * F + N0.transpose() * (-kpj * (robot->_q-q_desired) -kvj * (robot->_dq)) + g;

		//Moves to next mode
		VectorXd move_error_vec(6);
		move_error_vec << x_d-x, -delta_phi;
		double move_error = move_error_vec.norm();

		double finger_error = (finger_mask.cwiseProduct(robot->_q-q_desired)).norm();
		if (finger_mask.cwiseProduct(robot->_dq).norm() < 0.001){
			failCount++;
			if (failCount > failMax){
				finger_error = 0;
			}
		}else{
			failCount = 0;
		}

		double error = move_error + finger_error * 10;
		if (error < 0.01 && controller_mode != RETURNING){
			controller_mode += 1;
			failCount = 0;
		}

		if(controller_counter % 200 == 0){
			cout << "state: "<< controller_mode << endl;
			cout << "failCount: "<< failCount << endl;
			cout << "x: \n"<< x << endl << endl;
			cout << "x_d: \n"<< x_d << endl << endl;
			cout << "error: "<< error << endl;
			cout << "move error: "<< move_error << endl;
			cout << "finger error: "<< finger_error << endl;
			cout << "finger move error: "<< finger_mask.cwiseProduct(robot->_dq).norm() << endl << endl;
		}

// --------------------------- redis  ---------------------------------------

		// get next move
		
		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.set(PIECE_NAME_KEY, target_piece);

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
