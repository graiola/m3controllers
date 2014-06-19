#include "m3controllers/vf_controller/vf_controller.h"

namespace m3controllers{
	
using namespace m3rt;
using namespace std;
using namespace m3;
using namespace tools;
using namespace kdl_kinematics;
using namespace Eigen;

/*bool VfController::LinkDependentComponents()
{
	
	if(!M3Controller::LinkDependentComponents())
		return false;
	
	kin_component_ = (CartController*) factory->GetComponent(kin_component_name_);
	if (kin_component_== NULL){
		M3_INFO("CartController component %s not found for component %s\n",kin_component_name_.c_str(),GetName().c_str());
		return false;
	}
	
	return true;
}*/

void VfController::Startup()
{	
	
	M3Controller::Startup();
	
	/*if (kin_component_ == NULL)
		SetStateError();
	else
		SetStateSafeOp();*/
		
	// Resize
	cart_pos_status_.resize(3);
	cart_pos_cmd_.resize(3);
	cart_vel_status_.resize(3);
	// Clear
	cart_pos_status_.fill(0.0);
	cart_pos_cmd_.fill(0.0);
	cart_vel_status_.fill(0.0);

	//kin_component_->EnableInternalCommunication(); // Enable the communication with it.
	
	// Controller sample time
	//dt_ = 1/static_cast<double>(RT_TASK_FREQUENCY);
	
	// Set the kinematic mask
	kin_->setMask(cart_mask_str_);
	
	// User velocity, vf and joint vel commands
	v_.resize(3);
	vd_.resize(3);
	T_.resize(3,1);
	Pi_.resize(3,1);
	Pf_.resize(3,1);
	joints_vel_cmd_.resize(Ndof_);
	
	// Define the virtual fixture
	c_ = 1;
	Pi_ << 0.0, 0.0, 0.0;
	Pf_ << 0.0, 0.0, 0.5;
	T_ = (Pf_-Pi_)/(Pf_-Pi_).norm();
	D_ = T_*(T_.transpose()*T_).inverse() * T_.transpose();
	I_ = MatrixXd::Identity(3,3);

#ifdef USE_ROS_RT_PUBLISHER
	if(ros::master::check()){
		rt_publishers_.AddPublisher(*ros_nh_ptr_,"user_velocity",3,&v_);
		rt_publishers_.AddPublisher(*ros_nh_ptr_,"desired_velocity",3,&vd_);
	}
#endif

/*#ifdef USE_ROS_RT_PUBLISHER
	RosInit();
	//rt_publishers_.AddPublisher(*ros_nh_ptr_,"dmp_state_status",2,&dmp_trajectory_,y_init);
	//rt_publishers_.AddPublisher(*ros_nh_ptr_,"dmp_state_status",2,&dmp_trajectory_);
#endif*/
}

void VfController::Shutdown()
{
	
	M3Controller::Shutdown();
	
/*#ifdef USE_ROS_RT_PUBLISHER
	RosShutdown();
#endif*/
}
						  
bool VfController::ReadConfig(const char* cfg_filename)
{
	//YAML::Node doc;
	//GetYamlDoc(cfg_filename, doc);
	
	if (!M3Controller::ReadConfig(cfg_filename))
		return false;
	
	const YAML::Node& ik = doc["ik"];
	double damp_max, epsilon;
	ik["cart_mask"] >> cart_mask_str_;
	ik["damp_max"] >> damp_max;
	ik["epsilon"] >> epsilon;
	
	Eigen::MatrixXd gains = MatrixXd::Zero(6,6);
	ik["gain_x"] >> gains(0,0);
	ik["gain_y"] >> gains(1,1);
	ik["gain_z"] >> gains(2,2);
	ik["gain_roll"] >> gains(3,3);
	ik["gain_pitch"] >> gains(4,4);
	ik["gain_yaw"] >> gains(5,5);
	
	// Sample time
	double dt = 1/static_cast<double>(RT_TASK_FREQUENCY);
	
	std::string root_name = "T0"; //FIXME
	std::string end_effector_name;
	if(chain_name_ == "RIGHT_ARM")
		end_effector_name = "palm_right";
	else if(chain_name_ == "LEFT_ARM")
		 end_effector_name = "palm_left";
	else
	{
		M3_ERR("Only RIGHT_ARM and LEFT_ARM are supported");
		return false;
	}
	
	try
	{
		kin_ = new KDLClik (root_name,end_effector_name,damp_max,epsilon,gains,dt);
	}
	catch(const std::runtime_error& e)
	{	
		M3_ERR("Failed to create kdl kinematics: ",e.what());
		return false;
	}

	return true;
}

void VfController::StepStatus()
{
	
	M3Controller::StepMotorsStatus(); // Read the joints status from motors

	// Generate an user movement
	//v_[0] = 0.5 * sin(2.0 * M_PI * static_cast<double>(loop_cnt_) / (1 * RT_TASK_FREQUENCY));
	//v_[1] = 0.5 * sin(2.0 * M_PI * static_cast<double>(loop_cnt_) / (1 * RT_TASK_FREQUENCY));
	//v_[2] = 0.5 * sin(2.0 * M_PI * static_cast<double>(loop_cnt_) / (1 * RT_TASK_FREQUENCY));

	
	kin_->clikStatusStep(joints_pos_status_,cart_pos_status_);
	
	kin_->ComputeFkDot(joints_pos_status_,joints_vel_status_,v_);

	
	//kin_component_->GetStatus(dmp_state_status_.segment(0,6));
	
/*#ifdef USE_ROS_RT_PUBLISHER
	//rt_publishers_.PublishAll();
#endif*/
	
	M3Controller::StepStatus(); // Update the status sds

}

void VfController::StepCommand()
{	
	M3Controller::StepCommand(); // Update the command sds
	
	vd_ = (c_*D_*v_ + (1-c_)*(I_-D_)*v_);
	
	cart_pos_cmd_ = vd_*dt_ + cart_pos_status_;
	kin_->clikCommandStep(joints_pos_status_,cart_pos_cmd_,joints_pos_cmd_);
	
	//kin_->ComputeIk(joints_pos_status_,vd_,joints_vel_cmd_);
	
	//qdot_*dt_ + joints_pos_status_;
	
	M3Controller::StepMotorsCommand(joints_pos_cmd_);
	
	// Gooooo
	//kin_component_->EnableController();
	//kin_component_->SetCommand(dmp_state_command_.segment(0,6));
}

}
