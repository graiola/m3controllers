#include "m3controllers/sin_controller/sin_controller.h"

namespace m3controllers{
	
using namespace m3rt;
using namespace std;
using namespace m3;
using namespace tools;

void SinController::Startup()
{
	M3Controller::Startup();
	
	// Set the mask
	//mask_.resize(Ndof_);
	
	// Resizes
	sin_waves_.resize(Ndof_);
	sin_waves_.fill(0.0);
	
	// Checks
	assert(centers_.size() == static_cast<size_t>(Ndof_));
	assert(periods_.size() == static_cast<size_t>(Ndof_));
	assert(magnitudes_.size() == static_cast<size_t>(Ndof_));
}

void SinController::Shutdown()
{	
	M3Controller::Shutdown();
}
						  
bool SinController::ReadConfig(const char* cfg_filename)
{
	//YAML::Node doc;

	if(!M3Controller::ReadConfig(cfg_filename))
		return false;
	
	//GetYamlDoc(cfg_filename, doc);
	
	doc["centers"] >> centers_;
	doc["periods"] >> periods_;
	doc["magnitudes"] >> magnitudes_;
	
	return true;
}

void SinController::StepStatus()
{	
	M3Controller::StepMotorsStatus(); // Read the joints status from motors
	
	M3Controller::StepStatus();
}

void SinController::StepCommand()
{
	M3Controller::StepCommand(); // Update the command sds
	
	for(int i=0;i<sin_waves_.size();i++)
		sin_waves_[i] = centers_[i] + magnitudes_[i] * sin(2.0 * M_PI * static_cast<double>(loop_cnt_) / (periods_[i] * RT_TASK_FREQUENCY));
	
	M3Controller::StepMotorsCommand(sin_waves_);
}


}
