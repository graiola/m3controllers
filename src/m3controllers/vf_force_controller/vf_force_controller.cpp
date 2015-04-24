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
	
	// Set the kinematic mask
	kin_->setMask("1,1,1,0,0,0"); // xyz

	cart_size_ = kin_->getCartSize();
	Nodf_kin_ = kin_->getNdof();
	
	// Resize
	torques_id_.resize(Ndof_controlled_);
	user_torques_.resize(Ndof_controlled_);
        torques_cmd_.resize(Nodf_kin_);
        torques_status_.resize(Ndof_controlled_);
        position_status_.resize(Ndof_controlled_);
        velocity_status_.resize(Ndof_controlled_);
	f_user_.resize(3);
	f_vm_.resize(3);
        
	// Clear
	torques_id_.fill(0.0);
	user_torques_.fill(0.0);
        torques_cmd_.fill(0.0);
        torques_status_.fill(0.0);
        position_status_.fill(0.0);
        velocity_status_.fill(0.0);
	f_user_.fill(0.0);
	f_vm_.fill(0.0);
	
	// Cart resize
	cart_pos_status_.resize(cart_size_);
	cart_pos_cmd_.resize(cart_size_);
	cart_vel_status_.resize(cart_size_);
	
	// User velocity, vf and joint vel commands
	jacobian_.resize(cart_size_,Nodf_kin_);
        //jacobian_position_.resize(3,Nodf_kin_);
	jacobian_t_.resize(Nodf_kin_,cart_size_); // only pos
	jacobian_t_pinv_.resize(cart_size_,Nodf_kin_); // only pos

        open_hand_ = false;
        
        svd_vect_.resize(cart_size_);
        svd_.reset(new svd_t(Nodf_kin_,cart_size_)); // It should have the same dimensionality of the problem
	
	// Reset force filter
	for(int i=0; i<cart_size_; i++)
	  force_filters_[i].Reset();
	  
	// Inverse kinematics pre-allocations
	svd_->compute(jacobian_t_, ComputeThinU | ComputeThinV); // This is not rt safe! We trigger it here to pre-allocate the internal variables
	matrixU_t_.resize(cart_size_,Nodf_kin_); // Eigen does some tricks with the dimensionalities, check the .h for info
	matrixV_.resize(cart_size_,cart_size_);
	jacobian_t_pinv_tmp_.resize(cart_size_,cart_size_);
	

	INIT_CNT(tmp_dt_status_);
	INIT_CNT(tmp_dt_cmd_);
}

void VfForceController::Shutdown()
{
	M3Controller::Shutdown();
}
						  
bool VfForceController::ReadConfig(const char* cfg_filename)
{

	if (!M3Controller::ReadConfig(cfg_filename))
		return false;

	doc["dynamic"] >> dyn_component_name_;

	// IK
	const YAML::Node& ik_node = doc["ik"];
	double damp_max, epsilon;
	//ik_node["cart_mask"] >> cart_mask_str_;
	ik_node["damp_max"] >> damp_max;
	ik_node["epsilon"] >> epsilon;

	// Controller sample time
	dt_ = 1/static_cast<double>(RT_TASK_FREQUENCY);
	
	root_name_ = "T0"; //FIXME
	
	if(chain_name_ == "RIGHT_ARM")
        {
		end_effector_name_ = "fixed_right_wrist"; //wrist_RIGHT
                hand_chain_ = RIGHT_HAND;
        }
	else if(chain_name_ == "LEFT_ARM")
        {
		 end_effector_name_ = "fixed_left_wrist";
                 hand_chain_ = LEFT_HAND;
        }
	else
	{
		M3_ERR("Only RIGHT_ARM and LEFT_ARM are supported");
		return false;
	}

	try
	{
		kin_ = new KDLKinematics (root_name_,end_effector_name_,damp_max,epsilon);
	}
	catch(const std::runtime_error& e)
	{
		M3_ERR("Failed to create kdl kinematics: ",e.what());
		return false;
	}

	// FORCE FILTER
	const YAML::Node& force_filter_node = doc["force_filter"];
	int order;
	double cutoff_freq;
	//string filter_type;
	//force_filter_node["type"] >> filter_type;
	force_filter_node["order"] >> order;
	force_filter_node["cutoff_freq"] >> cutoff_freq;
	
	force_filters_.resize(3);
	
	for(int i=0; i<3; i++)
	{
	  force_filters_[i].ReadConfig(force_filter_node);
	  force_filters_[i].GetXdf()->SetOrder(order);
	  force_filters_[i].GetXdf()->SetCutoff_freq(cutoff_freq);
	}
	
	return true;
}

void VfForceController::StepStatus()
{
	//Eigen::internal::set_is_malloc_allowed(false);
        SAVE_TIME(start_dt_status_);
  
	M3Controller::StepMotorsStatus(); // Read the joints status from motors

	for(int i=0;i<Ndof_controlled_;i++)
	  torques_id_[i] = - dyn_component_->GetG(i) /1000;
        
        joints_mask_cnt_ = 0;
        for(int i=0;i<Ndof_;i++)
            if(joints_mask_[i])
            {
                torques_status_[joints_mask_cnt_] = joints_torques_status_[i];
                position_status_[joints_mask_cnt_] = joints_pos_status_[i];
                velocity_status_[joints_mask_cnt_] = joints_vel_status_[i];
                joints_mask_cnt_++;
            }
            
	kin_->ComputeJac(position_status_,jacobian_);
        
        //jacobian_position_ = jacobian_.block<3,4>(0,0);
        
	jacobian_t_= jacobian_.transpose();
	
	// IK
	svd_->compute(jacobian_t_, ComputeThinU | ComputeThinV);
        svd_vect_ = svd_->singularValues();
	damp_max = 0.001;
	epsilon = 0.01;
	for (int i = 0; i < svd_vect_.size(); i++)
	{
		svd_curr = svd_vect_[i];
		damp = std::exp(-4/epsilon*svd_vect_[i])*damp_max;
		svd_vect_[i] = svd_curr/(svd_curr*svd_curr+damp*damp);
	}
	
	matrixU_t_ = svd_->matrixU().transpose();
	matrixV_ = svd_->matrixV();
	
	jacobian_t_pinv_tmp_ = svd_->matrixV() * svd_vect_.asDiagonal();
	jacobian_t_pinv_.noalias() = jacobian_t_pinv_tmp_ * matrixU_t_; // NOTE .noalias() does the trick
	// END IK

	// Robot cart stuff
	kin_->ComputeFk(position_status_,cart_pos_status_);
	kin_->ComputeFkDot(position_status_,velocity_status_,cart_vel_status_);
	
	/*if(phase_(i)>= 0.90)
	    open_hand_ = true;
	if(phase_(i) <= 0.01)
	    open_hand_ = false;
        */  
	
	user_torques_ = torques_status_ - torques_id_;
	
        f_user_.noalias() = jacobian_t_pinv_ * (-1) * user_torques_.segment<4>(0);
	
        // Filter the user force
        for(int i=0; i<3; i++)
        {
          force_filters_[i].Step(f_user_(i),0.0);
          f_user_(i) =  force_filters_[i].GetX();
        }
        
        //if (f_user_.norm() < 2.0 && f_user_.norm() > -2.0)
        if (f_user_.norm() < 4.0 && f_user_.norm() > -4.0)
	{
          f_user_.fill(0.0);
	  mechanism_manager_.Update(cart_pos_status_,cart_vel_status_,dt_,f_vm_,false); // no force applied
	}
	else
	{
	  mechanism_manager_.Update(cart_pos_status_,cart_vel_status_,dt_,f_vm_,true); 
	}
	  
	// Update the virtual mechanisms
	//mechanism_manager_.Update(cart_pos_status_,cart_vel_status_,dt_,f_vm_);
	
	
	
	M3Controller::StepStatus(); // Update the status sds
	
	SAVE_TIME(end_dt_status_);
        PRINT_TIME(start_dt_status_,end_dt_status_,tmp_dt_status_,"status");
	//Eigen::internal::set_is_malloc_allowed(true);
}

void VfForceController::StepCommand()
{	
	//Eigen::internal::set_is_malloc_allowed(false);
        SAVE_TIME(start_dt_cmd_);
  
	M3Controller::StepCommand(); // Update the command sds
        
        torques_cmd_.noalias() = jacobian_t_ * f_vm_;
        
          // Motors on
          if (m3_controller_interface_command_.enable())
          {
            bot_->SetMotorPowerOn();
            for(int i=0;i<Nodf_kin_;i++)
            {
                bot_->SetStiffness(chain_,i,1.0);
                bot_->SetSlewRateProportional(chain_,i,1.0);
                bot_->SetModeTorqueGc(chain_,i);
                bot_->SetTorque_mNm(chain_,i,m2mm(torques_cmd_[i]));
            }

            for(int i=Nodf_kin_;i<Ndof_;i++)
            {
                bot_->SetStiffness(chain_,i,1.0);
                bot_->SetSlewRateProportional(chain_,i,1.0);
                bot_->SetModeThetaGc(chain_,i);
                bot_->SetThetaDeg(chain_,i,0.0);
            }

            if(open_hand_)
            {
                /*for(int i=0;i<5;i++)
                {
                    bot_->SetStiffness(hand_chain_,i,1.0);
                    bot_->SetSlewRateProportional(hand_chain_,i,1.0);
                    bot_->SetModeThetaGc(hand_chain_,i);
                    bot_->SetThetaDeg(hand_chain_,i,0.0);
                }*/
            }
            else
            {
                /*for(int i=0;i<5;i++)
                {
                    bot_->SetStiffness(hand_chain_,i,1.0);
                    bot_->SetSlewRateProportional(hand_chain_,i,1.0);
                    bot_->SetModeTorqueGc(hand_chain_,i);
                    bot_->SetTorque_mNm(hand_chain_,i,m2mm(1));
                }*/
            }

          }
          else
          {
            bot_->SetMotorPowerOn();
            for(int i=0;i<Nodf_kin_;i++)
            {
                bot_->SetStiffness(chain_,i,0.0);
                bot_->SetSlewRateProportional(chain_,i,1.0);
                bot_->SetModeTorqueGc(chain_,i);
                //bot_->SetTorque_mNm(chain_,i,m2mm(user_torques_[i]));
            }
            
            for(int i=Nodf_kin_;i<Ndof_;i++)
            {
                bot_->SetStiffness(chain_,i,1.0);
                bot_->SetSlewRateProportional(chain_,i,1.0);
                bot_->SetModeThetaGc(chain_,i);
                bot_->SetThetaDeg(chain_,i,0.0);
            }


            if(open_hand_)
            {
              /*  for(int i=0;i<5;i++)
                {
                    bot_->SetStiffness(hand_chain_,i,1.0);
                    bot_->SetSlewRateProportional(hand_chain_,i,1.0);
                    bot_->SetModeThetaGc(hand_chain_,i);
                    bot_->SetThetaDeg(hand_chain_,i,0.0);
                }*/
            }
            else
            {
                /*for(int i=0;i<5;i++)
                {
                    bot_->SetStiffness(hand_chain_,i,1.0);
                    bot_->SetSlewRateProportional(hand_chain_,i,1.0);
                    bot_->SetModeTorqueGc(hand_chain_,i);
                    bot_->SetTorque_mNm(hand_chain_,i,m2mm(1));
                }*/
            }

          }

	SAVE_TIME(end_dt_cmd_);
        PRINT_TIME(start_dt_cmd_,end_dt_cmd_,tmp_dt_cmd_,"cmd");
	//Eigen::internal::set_is_malloc_allowed(true);

}

}