#include "m3controllers/cart_controller/cart_controller.h"

namespace m3controllers{
	
using namespace m3rt;
using namespace std;
using namespace m3;
using namespace tools;
using namespace kdl_kinematics;

void CartController::Startup()
{	
	M3Controller::Startup();
	
	// Set the kdl_kinematics mask
	kin_->setMask(cart_mask_str_);
	cart_size_ = kin_->getCartSize();
	
	// Resize
	cart_pos_status_.resize(6); // NOTE always six
	cart_pos_cmd_.resize(6); // NOTE always six
	cart_vel_status_.resize(6); // NOTE always six
	// Clear
	cart_pos_status_.fill(0.0);
	cart_pos_cmd_.fill(0.0);
	cart_vel_status_.fill(0.0);
	
	if(reference_source_ == TRJ_REF){
		ReadTxtFile(cmd_input_file_name_.c_str(),cart_cmd_mat_);
		// Checks over the file
		assert(cart_cmd_mat_.size() > 0);
		assert(cart_cmd_mat_[0].size() == 6);
		samples_to_execute_ = cart_cmd_mat_.size();
	}
	else if(reference_source_ == CONST_REF)
	{
		// Constant reference
		assert(ef_pose_.size() == 6);
		cart_pos_cmd_ = Map<VectorXd>(&ef_pose_[0],6);
	}
	else if(reference_source_ == SDS_REF)
	{
		// ????
	}
	
	// Dump
	if(file_length_samples_ > 0){
		file_dumpers_.AddDumper(file_length_samples_,&cart_pos_status_,"cart_pos_status",file_path_);
		file_dumpers_.AddDumper(file_length_samples_,&cart_pos_cmd_,"cart_pos_cmd",file_path_);
	}
}

void CartController::Shutdown()
{		
	M3Controller::Shutdown();
}
						  
bool CartController::ReadConfig(const char* cfg_filename)
{
	YAML::Node doc;

	if(!M3Controller::ReadConfig(cfg_filename))
		return false;
	
	GetYamlDoc(cfg_filename, doc);
	
	if(YAML::Node parameter = doc["ef_trj"])
	{
		doc["ef_trj"] >> cmd_input_file_name_;
		M3_INFO("Using an input file for the component %s\n",GetName().c_str());
		reference_source_ = TRJ_REF;
	}
	else if(YAML::Node parameter = doc["ef_pose"])
	{
		doc["ef_pose"] >> ef_pose_;
		M3_INFO("Using a constant pose for the component %s\n",GetName().c_str());
		reference_source_ = CONST_REF;
	}
	else if(YAML::Node parameter = doc["ef_use_sds"])
	{
		doc["ef_use_sds"] >> enable_sds_; // FIXME useless???
		M3_INFO("Using sds communication for the component %s\n",GetName().c_str());
		reference_source_ = SDS_REF;
	}
	else
	{
		M3_INFO("No input selection for component %s\n",GetName().c_str());
		return false;
	}
	
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

void CartController::StepStatus()
{
	M3Controller::StepMotorsStatus(); // Read the joints status from motors

	kin_->clikStatusStep(joints_pos_status_,cart_pos_status_);
	//kin_->clikStatusStep(joints_pos_status_);
	
	SetStatus(cart_pos_status_);
	
	M3Controller::StepStatus(); // Update the status sds
}

void CartController::StepCommand()
{	
	M3Controller::StepCommand(); // Update the command sds
	
	if(reference_source_ == TRJ_REF && samples_executed_ < samples_to_execute_){
		cart_pos_cmd_ = Map<VectorXd>(&cart_cmd_mat_[samples_executed_][0],6);	
		samples_executed_++; // Increase the counter
	}
	if(reference_source_ == SDS_REF)
	{
		GetCommand(cart_pos_cmd_);
	}
	
	// Compute IK
	kin_->clikCommandStep(joints_pos_status_,cart_pos_cmd_,joints_pos_cmd_);
	
	M3Controller::StepMotorsCommand(joints_pos_cmd_); // Send the commands to the motors
}

}
