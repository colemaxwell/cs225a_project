#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

#include <iostream>
#include <fstream>
#include <string>

// state machine
#define MOVING_TO_PIECE	0
#define PICKING_DOWN		1
#define PICKING		2
#define PICKING_UP		3
#define MOVING_PIECE		4
#define PLACING_DOWN		5
#define PLACING		6
#define PLACING_UP		7
#define RETURNING		8


#define MOVING_TO_PIECE	0
#define PICKING_DOWN		1
#define PICKING		2
#define PICKING_UP		3
#define MOVING_PIECE		4
#define DROPPING		5
#define LIFTING		6
#define RETURNING_2		7

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
const std::string JOINT_ANGLES_KEY_1 = "sai2::cs225a::panda_robot::sensors::q1";
const std::string JOINT_VELOCITIES_KEY_1 = "sai2::cs225a::panda_robot::sensors::dq1";
const std::string EE_FORCE_KEY_1 = "sai2::cs225a::panda_robot::sensors::force1";
const std::string EE_MOMENT_KEY_1 = "sai2::cs225a::panda_robot::sensors::moment1";
const std::string JOINT_ANGLES_KEY_2 = "sai2::cs225a::panda_robot::sensors::q2";
const std::string JOINT_VELOCITIES_KEY_2 = "sai2::cs225a::panda_robot::sensors::dq2";
const std::string EE_FORCE_KEY_2 = "sai2::cs225a::panda_robot::sensors::force2";
const std::string EE_MOMENT_KEY_2 = "sai2::cs225a::panda_robot::sensors::moment2";

// - write
const std::string JOINT_TORQUES_COMMANDED_KEY_1 = "sai2::cs225a::panda_robot::actuators::fgc1";
const std::string JOINT_TORQUES_COMMANDED_KEY_2 = "sai2::cs225a::panda_robot::actuators::fgc2";
const string CONTROLLER_RUNNING_KEY = "sai2::cs225a::controller_running";


// - chess_interface
const std::string FINAL_PIECE_LOCATION_KEY_X = "sai2::cs225a::pieces::final_pos_x";
const std::string FINAL_PIECE_LOCATION_KEY_Y = "sai2::cs225a::pieces::final_pos_y";
const std::string INITIAL_PIECE_LOCATION_KEY_X = "sai2::cs225a::pieces::initial_pos_x";
const std::string INITIAL_PIECE_LOCATION_KEY_Y = "sai2::cs225a::pieces::initial_pos_y";
const std::string ROBOT_RUNNING_KEY = "sai2::cs225a::state::dynamics";

unsigned long long controller_counter = 0;

double grab_height = 0.04;
double over_height = 0.2;
const Vector3d home_white = Vector3d(0.0, 0.4, over_height);
const Vector3d home_black = Vector3d(0.85, -0.4, over_height);
Matrix3d point_down;
Vector2d target_location_board = Vector2d(3, 4);
std::string target_piece = "BPawn4";
Vector3d target_piece_pos;

int controller_mode = MOVING_TO_PIECE;
int controller_mode2 = MOVING_TO_PIECE;

const vector<string> robot_names = {
	"PANDA1",
	"PANDA2",
};

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
	const string world_file = "./resources/world.urdf";
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	vector<Sai2Model::Sai2Model*> robots_sim;
	int n_robots = 2;
	for(int i=0; i<n_robots; i++) {
		robots_sim.push_back(new Sai2Model::Sai2Model(robot_file, false, sim->getRobotBaseTransform(robot_names[i])));
	}
	robots_sim[0]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY_1);
	VectorXd initial_q_1 = robots_sim[0]->_q;
	robots_sim[0]->updateModel();
	
	robots_sim[1]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY_2);
	VectorXd initial_q_2 = robots_sim[1]->_q;
	robots_sim[1]->updateModel();

	// prepare controller
	int dof = robots_sim[0]->dof();
	const string ee_link_name = "link7";
	const Vector3d pos_in_ee_link = Vector3d(0, 0, 0.21);
	VectorXd command_torques_1 = VectorXd::Zero(dof);
	VectorXd command_torques_2 = VectorXd::Zero(dof);
	
	// model quantities for operational space control
	MatrixXd Jv_1 = MatrixXd::Zero(3,dof);
	MatrixXd J0_1 = MatrixXd::Zero(6,dof);
	MatrixXd Lambda_1 = MatrixXd::Zero(3,3);
	MatrixXd Lambda0_1 = MatrixXd::Zero(6,6);
	// MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N_1 = MatrixXd::Zero(dof,dof);
	MatrixXd N0_1 = MatrixXd::Zero(dof,dof);

	robots_sim[0]->Jv(Jv_1, ee_link_name, pos_in_ee_link);
	robots_sim[0]->taskInertiaMatrix(Lambda_1, Jv_1);
	// robot->dynConsistentInverseJacobian(J_bar, Jv);
	robots_sim[0]->nullspaceMatrix(N_1, Jv_1);
	robots_sim[0]->nullspaceMatrix(N0_1, J0_1);
	
	MatrixXd Jv_2 = MatrixXd::Zero(3,dof);
	MatrixXd J0_2 = MatrixXd::Zero(6,dof);
	MatrixXd Lambda_2 = MatrixXd::Zero(3,3);
	MatrixXd Lambda0_2 = MatrixXd::Zero(6,6);
	// MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N_2 = MatrixXd::Zero(dof,dof);
	MatrixXd N0_2 = MatrixXd::Zero(dof,dof);

	robots_sim[1]->Jv(Jv_2, ee_link_name, pos_in_ee_link);
	robots_sim[1]->taskInertiaMatrix(Lambda_2, Jv_2);
	// robot->dynConsistentInverseJacobian(J_bar, Jv);
	robots_sim[1]->nullspaceMatrix(N_2, Jv_2);
	robots_sim[1]->nullspaceMatrix(N0_2, J0_2);
	

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

	redis_client.set(CONTROLLER_RUNNING_KEY, "1");
	redis_client.set(ROBOT_RUNNING_KEY, "0");

	VectorXd finger_mask = VectorXd(dof);
	finger_mask <<  0, 0, 0, 0, 0, 0, 0, 1, 1;	

	int failCount = 0;
	int failMax = 20;

	Vector3d piece_lock;
	double time;
	double lastWPTime;
	double nextWPTime;
	int CHANGE_STATE = 1;
	int WHITE_TURN = 1;

	vector<Affine3d> robot_pose_in_world;
	Affine3d pose = Affine3d::Identity();
	pose.translation() = Vector3d(0,0,-0.05);
	pose.linear() = AngleAxisd(-1.57079,Vector3d::UnitZ()).toRotationMatrix();
	robot_pose_in_world.push_back(pose);
	pose.translation() = Vector3d(0.85,0,-0.05);
	pose.linear() = AngleAxisd(1.57079,Vector3d::UnitZ()).toRotationMatrix();
	robot_pose_in_world.push_back(pose);

	while (runloop) {
	
			const string ROBOT_RUNNING = redis_client.get(ROBOT_RUNNING_KEY);
		
		// wait for next scheduled loop
			timer.waitForNextLoop();
			time = timer.elapsedTime() - start_time;

			// read robot state from redis
			robots_sim[0]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY_1);
			robots_sim[0]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY_1);
			robots_sim[0]->updateModel();
			
			robots_sim[1]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY_2);
			robots_sim[1]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY_2);
			robots_sim[1]->updateModel();
			
			Vector3d final_piece_pos;
			
			final_piece_pos[0] = stod(redis_client.get(FINAL_PIECE_LOCATION_KEY_X));
			final_piece_pos[1] = stod(redis_client.get(FINAL_PIECE_LOCATION_KEY_Y));
			final_piece_pos[2] = 0;
			
			Vector3d initial_piece_pos;
			
			initial_piece_pos[0] = stod(redis_client.get(INITIAL_PIECE_LOCATION_KEY_X));
			initial_piece_pos[1] = stod(redis_client.get(INITIAL_PIECE_LOCATION_KEY_Y));
			initial_piece_pos[2] = 0;
			
			// **********************
			// WRITE YOUR CODE AFTER
			// **********************
			Vector3d x_1, x_d_1, waypoint, lastWaypoint, x_dot_1, v_d, w_1;
			Vector3d x_2, x_dot_2, w_2;
			MatrixXd J_bar_1 = MatrixXd::Zero(dof,3);
			MatrixXd J_bar_2 = MatrixXd::Zero(dof,3);
			MatrixXd J_1(6, dof);
			MatrixXd J_2(6, dof);
			Matrix3d R_1;
			Matrix3d R_2;

			//ROBOT 1
			Eigen::VectorXd g_1(dof); // Empty Gravity Vector
			robots_sim[0]->gravityVector(g_1);
			Eigen::VectorXd cF_1(dof); // Empty Coriolis Vector
			robots_sim[0]->coriolisForce(cF_1);

			robots_sim[0]->Jv(Jv_1, ee_link_name, pos_in_ee_link);
			robots_sim[0]->J_0(J0_1, ee_link_name, pos_in_ee_link);

			robots_sim[0]->taskInertiaMatrix(Lambda_1, Jv_1);
			robots_sim[0]->taskInertiaMatrix(Lambda0_1, J0_1);

			robots_sim[0]->dynConsistentInverseJacobian(J_bar_1, Jv_1);
			robots_sim[0]->nullspaceMatrix(N_1, Jv_1);
			robots_sim[0]->nullspaceMatrix(N0_1, J0_1);

			robots_sim[0]->position(x_1,ee_link_name, pos_in_ee_link);
			robots_sim[0]->linearVelocity(x_dot_1,ee_link_name, pos_in_ee_link);

			robots_sim[0]->rotation(R_1,ee_link_name);
			robots_sim[0]->angularVelocity(w_1,ee_link_name, pos_in_ee_link);

			VectorXd q_desired(dof);
			VectorXd q_desired_0(dof);
			q_desired <<0,-0.785398,-0,-2.18166,-0,1.39626,0.1,0,0;
			
			
			//ROBOT 2
			Eigen::VectorXd g_2(dof); // Empty Gravity Vector
			robots_sim[1]->gravityVector(g_2);
			Eigen::VectorXd cF_2(dof); // Empty Coriolis Vector
			robots_sim[1]->coriolisForce(cF_2);

			robots_sim[1]->Jv(Jv_2, ee_link_name, pos_in_ee_link);
			robots_sim[1]->J_0(J0_2, ee_link_name, pos_in_ee_link);

			robots_sim[1]->taskInertiaMatrix(Lambda_2, Jv_2);
			robots_sim[1]->taskInertiaMatrix(Lambda0_2, J0_2);

			robots_sim[1]->dynConsistentInverseJacobian(J_bar_2, Jv_2);
			robots_sim[1]->nullspaceMatrix(N_2, Jv_2);
			robots_sim[1]->nullspaceMatrix(N0_2, J0_2);

			robots_sim[1]->position(x_2,ee_link_name, pos_in_ee_link);
			robots_sim[1]->linearVelocity(x_dot_2,ee_link_name, pos_in_ee_link);

			robots_sim[1]->rotation(R_2,ee_link_name);
			robots_sim[1]->angularVelocity(w_2,ee_link_name, pos_in_ee_link);
			
						
			if (CHANGE_STATE == 1) {
				if (WHITE_TURN) {
				q_desired_0 = robots_sim[0]->_q;
				}
				else {
				q_desired_0 = robots_sim[1]->_q;
				}
			}

			if (ROBOT_RUNNING == "1" || ROBOT_RUNNING == "2") {

			// ---------------------------  controller ---------------------------------------
			Vector3d delta_phi;
			Matrix3d R_d;
			VectorXd F(6);

			double vMax = 1.5;

			double kp = 500;
			double kv = 50;
			double kpRot = 500;
			double kvRot = 50;

			double kpj = 800;
			double kvj = 50;

			double open = 0.020;
			double closed = 0;

			double fingerPos;

			Vector3d target_piece_pos_z0 = Vector3d(initial_piece_pos[0], initial_piece_pos[1], 0);

			R_d = point_down;
			
			if (ROBOT_RUNNING == "1") {

				if (controller_mode == MOVING_TO_PIECE){
					waypoint = target_piece_pos_z0;
					waypoint[2] += over_height;
					fingerPos = open;

				}else if (controller_mode == PICKING_DOWN){
					waypoint = target_piece_pos_z0;
					waypoint[2] += grab_height;
					fingerPos = open;
					piece_lock = target_piece_pos_z0;

				}else if (controller_mode == PICKING){
					waypoint = piece_lock;
					waypoint[2] += grab_height;
					fingerPos = closed;

				}else if (controller_mode == PICKING_UP){
					waypoint = piece_lock;
					waypoint[2] += over_height;
					fingerPos = closed;

				}else if(controller_mode == MOVING_PIECE){
					waypoint = final_piece_pos;
					waypoint[2] += over_height;
					fingerPos = closed;

				}else if(controller_mode == PLACING_DOWN){
					waypoint = final_piece_pos;
					waypoint[2] += grab_height;
					fingerPos = closed;

				}else if(controller_mode == PLACING){
					waypoint = final_piece_pos;
					waypoint[2] += grab_height;
					fingerPos = open;

				}else if(controller_mode == PLACING_UP){
					waypoint = final_piece_pos;
					waypoint[2] += over_height;
					fingerPos = open;

				}else if(controller_mode == RETURNING){
					if (WHITE_TURN) {
					waypoint = home_white;
					}
					else {
					waypoint = home_black;
					}
					fingerPos = open;
				}else{
					cout <<"NO MODE"<< endl;
				}
			}
			
			
			if (ROBOT_RUNNING == "2") {

				if (controller_mode2 == MOVING_TO_PIECE){
					waypoint = target_piece_pos_z0;
					waypoint[2] += over_height;
					fingerPos = open;

				}else if (controller_mode2 == PICKING_DOWN){
					waypoint = target_piece_pos_z0;
					waypoint[2] += grab_height;
					fingerPos = open;
					piece_lock = target_piece_pos_z0;

				}else if (controller_mode2 == PICKING){
					waypoint = piece_lock;
					waypoint[2] += grab_height;
					fingerPos = closed;

				}else if (controller_mode2 == PICKING_UP){
					waypoint = piece_lock;
					waypoint[2] += over_height;
					fingerPos = closed;

				}else if(controller_mode2 == MOVING_PIECE){
					waypoint = final_piece_pos;
					waypoint[2] += over_height;
					fingerPos = closed;

				}else if(controller_mode2 == DROPPING){
					waypoint = final_piece_pos;
					waypoint[2] = over_height;
					fingerPos = open;
				}
				else if(controller_mode2 == LIFTING){
					waypoint = final_piece_pos;
					waypoint[2] = over_height;
					fingerPos = open;
				}
				else if(controller_mode2 == RETURNING_2){
					if (WHITE_TURN) {
					waypoint = home_white;
					}
					else {
					waypoint = home_black;
					}
					fingerPos = open;
				}else{
					cout <<"NO MODE"<< endl;
				}
			}
			
			if(WHITE_TURN) {
			waypoint = robot_pose_in_world[0].linear().transpose()*(waypoint-robot_pose_in_world[0].translation());
			}
			else {
			waypoint = robot_pose_in_world[1].linear().transpose()*(waypoint-robot_pose_in_world[1].translation());
			}
			
			q_desired[7] = fingerPos;
			q_desired[8] = -fingerPos;

			//move x_d
			/*Vector3d dirUnit = (waypoint - lastWaypoint).normalized();
			nextWPTime = (waypoint - lastWaypoint).norm() / vMax + lastWPTime;
			double tFactor = (time - lastWPTime) / (nextWPTime - lastWPTime);
			if (tFactor > 1){
				tFactor = 1;
				x_d = waypoint;
				v_d.setZero();
			}else{
				x_d = tFactor * waypoint + (1-tFactor) * lastWaypoint;
				v_d = dirUnit * vMax;
			}*/
			double waypointDis = (waypoint - lastWaypoint).norm();
			double targetDis = 0;
			if (WHITE_TURN) {
				targetDis = (x_1 - waypoint).norm();
			}
			else {
				targetDis = (x_2 - waypoint).norm();
			}
			Vector3d dirUnit;
			double vScale;

			if (waypointDis != 0){
				if (WHITE_TURN) {
				dirUnit = -(x_1 - waypoint).normalized();
				}
				else {
				dirUnit = -(x_2 - waypoint).normalized(); 
				}
				 if (targetDis < 0.02){
				 	vScale = 0;
				 }else{
				 	vScale = 1;
				 }
				 	
				
			}else{
				dirUnit.setZero();
				vScale = 1;
			}
			v_d = dirUnit * vMax/2 * vScale;


			// calculate delta_phi
			delta_phi.setZero();
			for (int i = 0; i < 3; i++){
				if (WHITE_TURN) {
				delta_phi += -0.5 * R_1.col(i).cross(R_d.col(i));
				} //World fixed x,y,z
				else {
				delta_phi += -0.5 * R_2.col(i).cross(R_d.col(i)); //World fixed x,y,z
				}
			}
			// calculate pos_d, position desired
			VectorXd pos_d(6);
			if (WHITE_TURN) {
			pos_d << -kp*(x_1 - waypoint) - kv*(x_dot_1 - v_d), kpRot*(-delta_phi) - kvRot*w_1;
			}
			else {
			pos_d << -kp*(x_2 - waypoint) - kv*(x_dot_2 - v_d), kpRot*(-delta_phi) - kvRot*w_2;
			}

			// calculate F
			if (WHITE_TURN) {
			F = Lambda0_1 * pos_d;
			}
			else {
			F = Lambda0_2 * pos_d;
			}
				
			// calculate command_torques
			if (WHITE_TURN) {
			command_torques_1 = J0_1.transpose() * F + N0_1.transpose() * (-kpj * (robots_sim[0]->_q-q_desired) -kvj * (robots_sim[0]->_dq)) + g_1;
			
			command_torques_2 = g_2 + robots_sim[1]->_M*(-kvj * (robots_sim[1]->_dq));
			}
			else {
			command_torques_2 = J0_2.transpose() * F + N0_2.transpose() * (-kpj * (robots_sim[1]->_q-q_desired) -kvj * (robots_sim[1]->_dq)) + g_2;
			command_torques_1 = g_1 + robots_sim[0]->_M*(-kvj * (robots_sim[0]->_dq));
			}


			//Moves to next mode
			VectorXd move_error_vec(6);
			if (WHITE_TURN) {
				move_error_vec << waypoint-x_1, -delta_phi;
			}
			else {
				move_error_vec << waypoint-x_2, -delta_phi;
			}
			double move_error = move_error_vec.norm();

			double finger_error = 0;
			
			if (WHITE_TURN) {
			finger_error = (finger_mask.cwiseProduct(robots_sim[0]->_q-q_desired)).norm();
			}
			else {
			finger_error = (finger_mask.cwiseProduct(robots_sim[1]->_q-q_desired)).norm();
			}

			if (WHITE_TURN) {
			if (finger_mask.cwiseProduct(robots_sim[0]->_dq).norm() < 0.001){
				failCount++;
				if (failCount > failMax){
					finger_error = 0;
				}
			}else{
				failCount = 0;
			}
			}
			else {
			if (finger_mask.cwiseProduct(robots_sim[1]->_dq).norm() < 0.001){
				failCount++;
				if (failCount > failMax){
					finger_error = 0;
				}
			}else{
				failCount = 0;
			}
			}

			double error = move_error + finger_error * 10;
			if (error < 0.01 && ((controller_mode != RETURNING && ROBOT_RUNNING == "1") || (controller_mode2 != RETURNING_2 && ROBOT_RUNNING == "2"))) {
			
				if (ROBOT_RUNNING == "1") {
				controller_mode += 1;
				}
				else {
				 controller_mode2 += 1;
				}
				//x_d = waypoint;
				lastWaypoint = waypoint;
				//lastWPTime = time;
				failCount = 0;
			}
			else if (error < 0.01 && (controller_mode == RETURNING || controller_mode2 == RETURNING_2)){
				controller_mode = 0;
				controller_mode2 = 0;
				CHANGE_STATE = 1;
				redis_client.set(ROBOT_RUNNING_KEY, "0");
				if (WHITE_TURN && ROBOT_RUNNING == "1") {
				WHITE_TURN = 0;
				}
				else if (ROBOT_RUNNING == "1") {
				WHITE_TURN = 1;
				}
			}
			

			if(controller_counter % 200 == 0){
				cout << ROBOT_RUNNING_KEY << endl;
				cout << "state: "<< controller_mode << endl;
				cout << "state: "<< controller_mode2 << endl;
				cout << "failCount: "<< failCount << endl;
				cout << "x: \n"<< x_1 << endl << endl;
				cout << "waypoint: \n"<< waypoint << endl << endl;
				cout << "v_d: \n"<< v_d << endl << endl;
				cout << "|v|: "<< x_dot_1.norm() << endl << endl;
				cout << "vScale: \n"<< vScale << endl << endl;
				cout << "error: "<< error << endl;
				cout << "move error: "<< move_error << endl;
				cout << "finger error: "<< finger_error << endl;
				cout << "finger move error: "<< finger_mask.cwiseProduct(robots_sim[0]->_dq).norm() << endl << endl;
				cout << "robot running "<< ROBOT_RUNNING << endl << endl;
			}
		}
		else {
		CHANGE_STATE = 0;
		robots_sim[0]->gravityVector(g_1);
		robots_sim[1]->gravityVector(g_2);
		double kpj = 50;
		double kvj = 10;
		
		command_torques_1 = g_1 + robots_sim[0]->_M*(-kvj * (robots_sim[0]->_dq));
		command_torques_2 = g_2 + robots_sim[1]->_M*(-kvj * (robots_sim[1]->_dq));
		
		if(controller_counter % 200 == 0){
				cout << q_desired_0;
				cout << robots_sim[0]->_q;
				cout << "state: "<< controller_mode << endl;
				cout << "state: "<< controller_mode2 << endl;
				cout << "robot running "<< ROBOT_RUNNING << endl << endl;
			}
		}

// --------------------------- redis  ---------------------------------------

		// get next move
		
		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY_1, command_torques_1);
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY_2, command_torques_2);
		//redis_client.set(PIECE_NAME_KEY, target_piece);

		controller_counter++;

	}

	command_torques_1.setZero();
	command_torques_2.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY_1, command_torques_1);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY_2, command_torques_2);
	redis_client.set(CONTROLLER_RUNNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
