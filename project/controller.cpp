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

const string robot_file = "./resources/panda_arm.urdf";
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

	// read terminal input, i.e. ./hw3 51
	int controller_number = QUESTION_4B;  

	string filename;
	if(controller_number == QUESTION_1A)
		filename = "../../hw3/data_files/question_1a.txt";
	else if(controller_number == QUESTION_1C)
		filename = "../../hw3/data_files/question_1c.txt";
	else if(controller_number == QUESTION_2D)
		filename = "../../hw3/data_files/question_2d.txt";
	else if(controller_number == QUESTION_2E)
		filename = "../../hw3/data_files/question_2e.txt";
	else if(controller_number == QUESTION_2F)
		filename = "../../hw3/data_files/question_2f.txt";
	else if(controller_number == QUESTION_2G)
		filename = "../../hw3/data_files/question_2g.txt";
	else if(controller_number == QUESTION_3)
		filename = "../../hw3/data_files/question_3.txt";
	else if(controller_number == QUESTION_4A)
		filename = "../../hw3/data_files/question_4a.txt";
	else if(controller_number == QUESTION_4B)
		filename = "../../hw3/data_files/question_4b.txt";
	else if(controller_number == QUESTION_5A)
		filename = "../../hw3/data_files/question_5a.txt";
	else if(controller_number == QUESTION_5B)
		filename = "../../hw3/data_files/question_5b.txt";
	else if(controller_number == QUESTION_5C)
		filename = "../../hw3/data_files/question_5c.txt";

	ofstream data_file;
	data_file.open(filename);

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

		// read force sensor from redis
		if(controller_number == QUESTION_5A || controller_number == QUESTION_5B || controller_number == QUESTION_5C) {
			force = redis_client.getEigenMatrixJSON(EE_FORCE_KEY);
			moment = redis_client.getEigenMatrixJSON(EE_MOMENT_KEY);
		}

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
		q_desired <<0,0,0,0,0,0,0;

		VectorXd q_original(dof);
		q_original  <<0,-0.785398,-0,-2.18166,-0,1.39626,0.1;


		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1A)
		{

			double kp = 100;
			double kv = 20;
			double kpj = 50;
			double kvj = 14;

			//Updating Done Above

			x_d << (0.3 + 0.1*sin(PI * time)) , (0.1 + 0.1*cos(PI * time)) , 0.5;// set x_d

			// calculate joint_task_torque

			// calculate F
			F = Lambda * (-kp*(x - x_d) - kv * x_dot);

			// calculate command_torques
			command_torques = Jv.transpose() * F + N.transpose() * (-kp * (robot->_q-q_desired) -kv * (robot->_dq)) + g;

			if(controller_counter % 10 == 0)
			{
				data_file << time << '\t' << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\n';
			}
		}
		else if(controller_number == QUESTION_1C)
		{
			double kp = 100;
			double kv = 20;
			double kpj = 50;
			double kvj = 14;

			//Updating Done Above

			x_d << (0.3 + 0.1*sin(PI * time)) , (0.1 + 0.1*cos(PI * time)) , 0.5;// set x_d

			x_d_dot << (0.1*PI*cos(PI * time)) , (-0.1*PI*sin(PI * time)) , 0;

			// calculate joint_task_torque

			// calculate F
			F = Lambda * (-kp*(x - x_d) - kv * (x_dot - x_d_dot ));

			// calculate command_torques
			command_torques = Jv.transpose() * F + N.transpose() * (-kp * (robot->_q-q_desired) -kv * (robot->_dq)) + g;

			if(controller_counter % 10 == 0)
			{
				data_file << time << '\t' << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\n';
			}
		}

		// ---------------------------  question 2 ---------------------------------------
		else if(controller_number == QUESTION_2D)
		{
			// part 1
			VectorXd Gamma_damp(dof);

			double kp = 100;
			double kv = 20;
			double kdamp = 14; // set kdamp value

			VectorXd q_low(dof);
			q_low <<-165, -100, -165, -170, -165, 0, -165;  // set q_low
			q_low = q_low * M_PI / 180.0;
			
			VectorXd q_high(dof);
			q_high <<165, 100, 165, -30, 165, 210, 165;  // set q_high
			q_high= q_high * M_PI / 180.0;
			// set x_d
			x_d << -0.1,0.15,0.2;

			// calculate F
			F = Lambda * (-kp*(x - x_d) - kv * (x_dot));
			// calculate Gamma_damp
			Gamma_damp = - kdamp * robot->_dq;

			// calculate command_torques
			command_torques = Jv.transpose() * F + N.transpose()*Gamma_damp + g;

			if(controller_counter % 10 == 0)
			{
				data_file << time << '\t';
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << robot->_q(3) << '\t' << q_low(3) << '\t' << q_high(3) << '\t';
				data_file << robot->_q(5) << '\t' << q_low(5) << '\t' << q_high(5) << '\n';
			}
		}
		else if(controller_number == QUESTION_2E)
		{
			// part 2
			VectorXd Gamma_damp(dof), Gamma_mid(dof);

			double kp = 100;
			double kv = 20;
			double kdamp = 14; // set kdamp value
			double kmid= 25; // set kmid value

			VectorXd q_low(dof);
			q_low <<-165, -100, -165, -170, -165, 0, -165;  // set q_low
			q_low = q_low * M_PI / 180.0;
			
			VectorXd q_high(dof);
			q_high <<165, 100, 165, -30, 165, 210, 165;  // set q_high
			q_high= q_high * M_PI / 180.0;
			// set x_d
			x_d << -0.1,0.15,0.2;

			// calculate F
			F = Lambda * (-kp*(x - x_d) - kv * (x_dot));
			// calculate Gamma_damp
			Gamma_damp = - kdamp * robot->_dq;

			// calculate Gamma_mid
			Gamma_mid = - 2 * kmid * (robot->_q - (q_high + q_low)/2);

			// calculate command_torques
			command_torques = Jv.transpose() * F + N.transpose()*(Gamma_mid + Gamma_damp) + g;

			if(controller_counter % 10 == 0)
			{
				data_file << time << '\t';
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << robot->_q(3) << '\t' << q_low(3) << '\t' << q_high(3) << '\t';
				data_file << robot->_q(5) << '\t' << q_low(5) << '\t' << q_high(5) << '\n';
			}
		}
		else if(controller_number == QUESTION_2F)
		{
			VectorXd Gamma_damp(dof), Gamma_mid(dof);

			double kp = 100;
			double kv = 20;
			double kdamp = 14; // set kdamp value
			double kmid= 25; // set kmid value

			VectorXd q_low(dof);
			q_low <<-165, -100, -165, -170, -165, 0, -165;  // set q_low
			q_low = q_low * M_PI / 180.0;
			
			VectorXd q_high(dof);
			q_high <<165, 100, 165, -30, 165, 210, 165;  // set q_high
			q_high= q_high * M_PI / 180.0;
			// set x_d
			x_d << -0.65,-0.45,0.7;

			// calculate F
			F = Lambda * (-kp*(x - x_d) - kv * (x_dot));
			// calculate Gamma_damp
			Gamma_damp = - kdamp * robot->_dq;

			// calculate Gamma_mid
			Gamma_mid = - 2 * kmid * (robot->_q - (q_high + q_low)/2);

			// calculate command_torques
			command_torques = Jv.transpose() * F + N.transpose()*(Gamma_mid + Gamma_damp) + g;

			if(controller_counter % 10 == 0)
			{
				data_file << time << '\t';
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << robot->_q(3) << '\t' << q_low(3) << '\t' << q_high(3) << '\t';
				data_file << robot->_q(5) << '\t' << q_low(5) << '\t' << q_high(5) << '\n';
			}
		}
		else if (controller_number == QUESTION_2G)
		{
			VectorXd Gamma_damp(dof), Gamma_mid(dof);

			double kp = 100;
			double kv = 20;
			double kdamp = 14; // set kdamp value
			double kmid= 25; // set kmid value

			VectorXd q_low(dof);
			q_low <<-165, -100, -165, -170, -165, 0, -165;  // set q_low
			q_low = q_low * M_PI / 180.0;
			
			VectorXd q_high(dof);
			q_high <<165, 100, 165, -30, 165, 210, 165;  // set q_high
			q_high= q_high * M_PI / 180.0;
			// set x_d
			x_d << -0.65,-0.45,0.7;

			// calculate F
			F = Lambda * (-kp*(x - x_d) - kv * (x_dot));
			// calculate Gamma_damp
			Gamma_damp = - kdamp * robot->_dq;

			// calculate Gamma_mid
			Gamma_mid = - 2 * kmid * (robot->_q - (q_high + q_low)/2);

			// calculate command_torques
			command_torques = Jv.transpose() * F + Gamma_mid + N.transpose()*(Gamma_damp) + g;

			if(controller_counter % 10 == 0)
			{
				data_file << time << '\t';
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << robot->_q(3) << '\t' << q_low(3) << '\t' << q_high(3) << '\t';
				data_file << robot->_q(5) << '\t' << q_low(5) << '\t' << q_high(5) << '\n';
			}
		}

		// ---------------------------  question 3 ---------------------------------------
		else if(controller_number == QUESTION_3)
		{
			Vector3d delta_phi;
			Matrix3d R_d;
			VectorXd F(6);

			double kp = 100;
			double kv = 20;
			double kvj = 14;


			R_d <<	cos(PI/3), 0, sin(PI/3),
     				0, 1, 0,
     				-sin(PI/3), 0, cos(PI/3);
     				// set R_d

			x_d << 0.6,0.3,0.5;// set x_d

			// calculate delta_phi
			delta_phi.setZero();
			for (int i = 0; i < 3; i++){
				delta_phi += -0.5 * R.col(i).cross(R_d.col(i));
			}
			// calculate pos_d, position desired
			VectorXd pos_d(6);
			pos_d << kp*(x_d-x) - kv*x_dot, kp*(-delta_phi) - kv*w;
			// calculate F
			F = Lambda0 * pos_d;
			
			// calculate command_torques
			command_torques = J0.transpose() * F - N0.transpose()*kvj*robot->_dq + g;

			if(controller_counter % 10 == 0)
			{
				data_file << time << '\t';
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << delta_phi(0) << '\t' << delta_phi(1) << '\t' << delta_phi(2) << '\n';
			}

		}

		// ---------------------------  question 4 ---------------------------------------
		else if(controller_number == QUESTION_4A)		
		{
			double Vmax = 0.1;
			double kp = 200;      // chose your p gain
			double kv = 28;      // chose your d gain
			double kvj = 4;
			Vector3d p(0,0,0);
			p = J_bar.transpose() * g;
			Eigen::VectorXd F(dof);

			x_d << 0.6,0.3,0.4;// set x_d

			F = Lambda * (-kp*(x - x_d) - kv * x_dot);
			command_torques = Jv.transpose() * F + N.transpose() * robot->_M * (-kp * (robot->_q-q_original) -kv * (robot->_dq)) + g;

			if(controller_counter % 10 == 0)
			{
				data_file << time << '\t';
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << x_dot(0) << '\t' << x_dot(1) << '\t' << x_dot(2) << '\t' << x_dot.norm() << '\t' << Vmax <<'\n';
			}
		}
		else if (controller_number == QUESTION_4B)
		{
			double Vmax = 0.1;
			double kp = 200;      // chose your p gain
			double kv = 28;      // chose your d gain
			double kvj = 4;
			Vector3d p(0,0,0);
			p = J_bar.transpose() * g;
			Eigen::VectorXd F(dof);

			x_d << 0.6,0.3,0.4;// set x_d
			x_d_dot = kp / kv * (x_d-x);

			double v = sat(Vmax / x_d_dot.norm());

			F = Lambda * (- kv * (x_dot-v*x_d_dot));
			command_torques = Jv.transpose() * F + N.transpose() * robot->_M * (-kp * (robot->_q-q_original) -kv * (robot->_dq)) + g;

			if(controller_counter % 10 == 0)
			{
				data_file << time << '\t';
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << x_dot(0) << '\t' << x_dot(1) << '\t' << x_dot(2) << '\t' << x_dot.norm() << '\t' << Vmax <<'\n';
			}
		}
		// ---------------------------  question 5 ---------------------------------------
		else if(controller_number == QUESTION_5A)
		{
			MatrixXd J(6, dof), Lambda0(6, 6);
			Vector3d x, x_d, dx_d, dx, ddx_d, w, delta_phi, pos_d_x, pos_d_w;
			VectorXd F(6), g(dof), b(dof), joint_task_torque(dof);
			Matrix3d R, R_d;

			double Vmax = 0.1;
			double nu;

			double kp = 200;
			double kv = 24;
			double kpj = 50;
			double kvj = 14;

			// update J
			// update Lambda0
			// update N
			// update x
			// update dx
			// update w
			// update g
			// update R

			// nested function for state machine
			auto moving_down  = [&] () {
				// set x_d, vertically moving down
				// calculate dx_d
				// calculate nu
				// calculate pos_d_x

				// if force reading is NOT 0, change state to MOVING_CIRCLE
			};

			// nested function for state machine
			auto moving_circle = [&] () {
				double s_time = time/3; // slower time rate
				// set x_d, circular motion
				// calculate dx_d, circular motion
				// calculate ddx_d, circular motion
				// calculate pos_d_x
			};

			// state switch for state machine
			switch(state) {
				case MOVING_DOWN :
					moving_down();
					break;
				case MOVING_CIRCLE :
					moving_circle();
					break;
				default:
					if (controller_counter%500 == 0) std::cout << "No state set" << std::endl;
			}

			// set R_d
			// calculate delta_phi
			// calculate pos_d_w
			
			// initialize and set pos_d

			// calculate F
			// calculate joint_task_torque

			// calculate command_torques
			command_torques.setZero();

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << force(0) << "\t" << force(1) << "\t" << force(2) << "\n";
			}
		}
		else if(controller_number == QUESTION_5B)
		{
			MatrixXd J(6, dof), Lambda0(6, 6);
			Vector3d x, x_d, dx_d, dx, ddx_d, w, delta_phi, pos_d_x, pos_d_w;
			VectorXd F(6), g(dof), b(dof), joint_task_torque(dof);
			Matrix3d R, R_d;

			double Vmax = 0.1;
			double nu;

			double kp = 200;
			double kv = 24;
			double kpj = 50;
			double kvj = 14;

			// update J
			// update Lambda0
			// update N
			// update x
			// update dx
			// update w
			// update g
			// update R

			// nested function for state machine
			auto moving_down  = [&] () {
				// set x_d, vertically moving down
				// calculate dx_d
				// calculate nu
				// calculate pos_d_x

				// if force reading is NOT 0, change state to MOVING_CIRCLE
			};

			// nested function for state machine
			auto moving_circle = [&] () {
				double s_time = time/3; // slower time rate
				// set x_d, circular motion
				// calculate dx_d, circular motion
				// calculate ddx_d, circular motion
				// calculate pos_d_x
			};

			// state switch for state machine
			switch(state) {
				case MOVING_DOWN :
					moving_down();
					break;
				case MOVING_CIRCLE :
					moving_circle();
					break;
				default:
					if (controller_counter%500 == 0) std::cout << "No state set" << std::endl;
			}

			// set R_d
			// calculate delta_phi
			// calculate pos_d_w
			
			// initialize and set pos_d

			// calculate F
			// calculate joint_task_torque

			// calculate command_torques
			command_torques.setZero();

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << force(0) << "\t" << force(1) << "\t" << force(2) << "\n";
			}

		}

		else if(controller_number == QUESTION_5C)
		{
			MatrixXd J(6, dof), Lambda0(6, 6);
			Vector3d x, x_d, dx_d, dx, ddx_d, w, delta_phi, pos_d_x;
			VectorXd F(6), F_desired(6), g(dof), b(dof), joint_task_torque(dof);
			Matrix3d R, R_d;

			double Vmax = 0.1;
			double nu;

			double kp = 200;
			double kv = 24;
			double kpj = 50;
			double kvj = 14;

			// update J
			// update Jv
			// update Lambda0
			// update Lambda
			// update N
			// update x
			// update dx
			// update w
			// update g
			// update b
			// update R

			// nested function for state machine
			auto moving_down  = [&] () {
				// set x_d, vertically moving down
				// calculate dx_d
				// calculate nu
				// calculate pos_d_x

				// if force reading is NOT 0, change state to APPLY_FORCE
			};

			// nested function for state machine
			auto apply_force = [&] () {
				// set x_d, vertically moving down
				// calculate dx_d
				// calculate nu
				// calculate pos_d_x

				// if force applied is >= 10, change state to MOVING_CIRCLE
			};

			// nested function for state machine
			auto moving_circle = [&] () {
				double s_time = time/3; // slower time rate
				// set x_d, circular motion
				// calculate dx_d, circular motion
				// calculate ddx_d, circular motion
				// calculate pos_d_x
			};

			switch(state) {
				case MOVING_DOWN :
					moving_down();
					break;
				case APPLY_FORCE :
					apply_force();
					break;
				case MOVING_CIRCLE :
					moving_circle();
					break;
				default:
					if (controller_counter%500 == 0) std::cout << "No state set" << std::endl;
			}

			// set R_d
			// calculate delta_phi
			// calculate pos_d_w
			
			// initialize and set pos_d

			// initialize and set section matrix, selc_omh
			// initialize and set section matrix, selc_omh_bar

			// set F_desired

			// initialize and set dx_vw (dx and w)

			// calculate joint_task_torque
			
			if (state == MOVING_CIRCLE) {
				// calculate F
			} else {
				// calculate F
			}

			// calculate command_torques
			command_torques.setZero();

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << x_d(0) << '\t' << x_d(1) << '\t' << x_d(2) << '\t';
				data_file << force(0) << "\t" << force(1) << "\t" << force(2) << "\n";
			}
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	data_file.close();

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
