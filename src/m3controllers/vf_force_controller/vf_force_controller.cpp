#include "m3controllers/vf_force_controller/vf_force_controller.h"

namespace m3controllers{
	
using namespace m3rt;
using namespace std;
using namespace m3;
using namespace tools;
using namespace kdl_kinematics;
using namespace Eigen;
using namespace virtual_mechanism_gmr;
using namespace DmpBbo;

// Checks
// 	assert(centers_.size() == static_cast<size_t>(Ndof_));
// 	assert(periods_.size() == static_cast<size_t>(Ndof_));
// 	assert(magnitudes_.size() == static_cast<size_t>(Ndof_));


bool VfForceController::LinkDependentComponents()
{
  
  if(!M3Controller::LinkDependentComponents())
	  return false;
  
  dyn_component_ = (M3Dynamatics*) factory->GetComponent(dyn_component_name_);
  if (dyn_component_== NULL){
	  M3_INFO("VfForceController component %s not found for component %s\n",dyn_component_name_.c_str(),GetName().c_str());
	  return false;
  }
  return true;
}

void VfForceController::Startup()
{	
  M3Controller::Startup();

  vm_thread_ = new VmThread("MAMMT");
  vm_thread_->Startup(file_names_,chain_name_,Ndof_controlled_,ros_nh_ptr_);
  
  //////// RESIZES AND INITIALIZATIONS
  torques_cmd_.resize(4);
  torques_status_.resize(Ndof_controlled_);
  position_status_.resize(Ndof_controlled_);
  velocity_status_.resize(Ndof_controlled_);
  // Clear
  torques_cmd_.fill(0.0);
  torques_status_.fill(0.0);
  position_status_.fill(0.0);
  velocity_status_.fill(0.0);
  
  vm_thread_->startThread();
  
  
  INIT_CNT(tmp_dt_status_);
  INIT_CNT(tmp_dt_cmd_);
}

void VfForceController::Shutdown()
{
	M3Controller::Shutdown();
	vm_thread_->kill_thread_ = false;
	vm_thread_->stopThread();
}
						  
bool VfForceController::ReadConfig(const char* cfg_filename)
{

	if (!M3Controller::ReadConfig(cfg_filename))
		return false;

	doc["dynamic"] >> dyn_component_name_;
	doc["file_names"] >> file_names_;
	
	return true;
}

void VfForceController::StepStatus()
{
	
        SAVE_TIME(start_dt_status_);
  
	M3Controller::StepMotorsStatus(); // Read the joints status from motors

        joints_mask_cnt_ = 0;
        for(int i=0;i<Ndof_;i++)
            if(joints_mask_[i])
            {
                torques_status_[joints_mask_cnt_] = -1 *  (dyn_component_->GetG(i)/1000 + joints_torques_status_[i]);
                position_status_[joints_mask_cnt_] = joints_pos_status_[i];
                velocity_status_[joints_mask_cnt_] = joints_vel_status_[i];
                joints_mask_cnt_++;
            }
            
        vm_thread_->setSharedStatus(torques_status_, position_status_, velocity_status_);    
            
        vm_thread_->RunThread();    
            
	
	SAVE_TIME(end_dt_status_);
        PRINT_TIME(start_dt_status_,end_dt_status_,tmp_dt_status_,"status");
	
}

void VfForceController::StepCommand()
{	
  SAVE_TIME(start_dt_cmd_);

  vm_thread_->getSharedCmd(torques_cmd_);

  // Motors on
  if (m3_controller_interface_command_.enable())
  {
    bot_->SetMotorPowerOn();
    for(int i=0;i<4;i++)
    {
      bot_->SetStiffness(chain_,i,1.0);
      bot_->SetSlewRateProportional(chain_,i,1.0);
      bot_->SetModeTorqueGc(chain_,i);
      bot_->SetTorque_mNm(chain_,i,m2mm(torques_cmd_[i]));
    }
    for(int i=4;i<Ndof_;i++)
    {
	bot_->SetStiffness(chain_,i,1.0);
	bot_->SetSlewRateProportional(chain_,i,1.0);
	bot_->SetModeThetaGc(chain_,i);
	bot_->SetThetaDeg(chain_,i,0.0);
    }
  }

  SAVE_TIME(end_dt_cmd_);
  PRINT_TIME(start_dt_cmd_,end_dt_cmd_,tmp_dt_cmd_,"cmd");
	
	
	
}

}