#include "m3controllers/joints_controller/joints_controller.h"

namespace m3controllers{
	
using namespace m3rt;
using namespace std;
using namespace m3;
using namespace tools;

void JointsController::Startup()
{
	M3Controller::Startup();

	if(reference_source_ == TRJ_REF){
		ReadTxtFile(cmd_input_file_name_.c_str(),joints_cmd_mat_);
		// Checks over the file
		assert(joints_cmd_mat_.size() > 0);
		assert(joints_cmd_mat_[0].size() == static_cast<size_t>(Ndof_));
		samples_to_execute_ = joints_cmd_mat_.size();
		joints_position_.resize(Ndof_);
	}
	if(reference_source_ == CONST_REF)
		assert(joints_position_.size() == static_cast<size_t>(Ndof_));
}

void JointsController::Shutdown()
{	
	M3Controller::Shutdown();
}
						  
bool JointsController::ReadConfig(const char* cfg_filename)
{
	YAML::Node doc;

	if(!M3Controller::ReadConfig(cfg_filename))
		return false;
	
	GetYamlDoc(cfg_filename, doc);
	
	if(YAML::Node parameter = doc["joints_trj"])
	{
		doc["joints_trj"] >> cmd_input_file_name_;
		M3_INFO("Using an input file for the component %s\n",GetName().c_str());
		reference_source_ = TRJ_REF;
	}
	else if(YAML::Node parameter = doc["joints_position"])
	{
		doc["joints_position"] >> joints_position_;
		M3_INFO("Using a constant pose for the component %s\n",GetName().c_str());
		reference_source_ = CONST_REF;
	}
	else
	{
		M3_INFO("No input selection for component %s\n",GetName().c_str());
		return false;
	}
	
	return true;
}

void JointsController::StepStatus()
{
	M3Controller::StepMotorsStatus(); // Read the joints status from motors
	
	M3Controller::StepStatus(); // Update the status sds
}

void JointsController::StepCommand()
{
	M3Controller::StepCommand(); // Update the command sds

	if(reference_source_ == TRJ_REF && samples_executed_ < samples_to_execute_){
		joints_position_ = joints_cmd_mat_[samples_executed_];	
		samples_executed_++; // Increase the counter
	}
	
	M3Controller::StepMotorsCommand(joints_position_); // Send the commands to the motors
}


}
