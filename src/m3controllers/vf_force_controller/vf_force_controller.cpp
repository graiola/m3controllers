#include "m3controllers/vf_force_controller/vf_force_controller.h"

namespace m3controllers{
	
using namespace m3rt;
using namespace std;
using namespace m3;
using namespace tools;
using namespace kdl_kinematics;
using namespace Eigen;
using namespace virtual_mechanism;

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
	
	// Resize
	torques_id_.resize(Ndof_controlled_);
	user_torques_.resize(Ndof_controlled_);
        torques_cmd_.resize(3);
        torques_status_.resize(Ndof_controlled_);
        position_status_.resize(Ndof_controlled_);
        velocity_status_.resize(Ndof_controlled_);
	// Clear
	torques_id_.fill(0.0);
	user_torques_.fill(0.0);
        torques_cmd_.fill(0.0);
        torques_status_.fill(0.0);
        position_status_.fill(0.0);
        velocity_status_.fill(0.0);
	
	// Set the kinematic mask
	kin_->setMask("1,1,1,0,0,0"); // xyz

	cart_size_ = kin_->getCartSize();
	
	// Cart resize
	cart_pos_status_.resize(cart_size_);
	cart_pos_cmd_.resize(cart_size_);
	cart_vel_status_.resize(cart_size_);
	vm_state_.resize(cart_size_);
	vm_state_dot_.resize(cart_size_);
	
	vm_ = new VirtualMechanism(cart_size_);
	
	// User velocity, vf and joint vel commands
	jacobian_.resize(3,Ndof_controlled_);
	jacobian_t_.resize(Ndof_controlled_,3);
	jacobian_t_pinv_.resize(Ndof_controlled_,3);
        jacobian_t_reduced_.resize(4,3);
	f_user_.resize(3);
	f_vm_.resize(3);
	f_cmd_.resize(3);
	//T_.resize(3,1);
	//Pi_.resize(3,1);
	//Pf_.resize(3,1);
        
        svd_vect_.resize(3);
        svd_.reset(new svd_t(3,Ndof_controlled_));
	
	// Define the virtual fixture
	//Pi_ << 0.0, 0.0, 0.0;
	//Pf_ << 0.0, 0.0, 0.5;
	//T_ = (Pf_-Pi_)/(Pf_-Pi_).norm();
	//D_ = T_*(T_.transpose()*T_).inverse() * T_.transpose();
 	//I_ = MatrixXd::Identity(3,3);

#ifdef USE_ROS_RT_PUBLISHER
	if(ros::master::check()){ 
//              rt_publishers_.AddPublisher(*ros_nh_ptr_,"user_torques",Ndof_controlled_,&user_torques_);      
//              rt_publishers_.AddPublisher(*ros_nh_ptr_,"id_torques",Ndof_controlled_,&torques_id_);
// 		rt_publishers_.AddPublisher(*ros_nh_ptr_,"user_force",3,&f_user_);
// 		rt_publishers_.AddPublisher(*ros_nh_ptr_,"vm_force",3,&f_vm_);
// 		rt_publishers_.AddPublisher(*ros_nh_ptr_,"cmd_force",3,&f_cmd_);
	  
	  
		boost::shared_ptr<RealTimePublisherWrench> tmp_ptr = NULL;
		tmp_ptr = boost::make_shared<RealTimePublisherWrench>(*ros_nh_ptr_,"vm_force",end_effector_name_);
		rt_publishers_wrench_.AddPublisher(tmp_ptr,&f_vm_);
		tmp_ptr = boost::make_shared<RealTimePublisherWrench>(*ros_nh_ptr_,"user_force",end_effector_name_);
		rt_publishers_wrench_.AddPublisher(tmp_ptr,&f_user_);
		tmp_ptr = boost::make_shared<RealTimePublisherWrench>(*ros_nh_ptr_,"cmd_force",end_effector_name_);
		rt_publishers_wrench_.AddPublisher(tmp_ptr,&f_cmd_);

 	        
		rt_publishers_path_.AddPublisher(*ros_nh_ptr_,"robot_pos",cart_pos_status_.size(),&cart_pos_status_);
		rt_publishers_path_.AddPublisher(*ros_nh_ptr_,"vm_pos",vm_state_.size(),&vm_state_);
	}
#endif

}

void VfForceController::Shutdown()
{
	M3Controller::Shutdown();
}
						  
bool VfForceController::ReadConfig(const char* cfg_filename)
{
	//YAML::Node doc;
	//GetYamlDoc(cfg_filename, doc);
	
	if (!M3Controller::ReadConfig(cfg_filename))
		return false;

	doc["dynamic"] >> dyn_component_name_;

	const YAML::Node& ik = doc["ik"];
	double damp_max, epsilon;
	//ik["cart_mask"] >> cart_mask_str_;
	ik["damp_max"] >> damp_max;
	ik["epsilon"] >> epsilon;

	Eigen::MatrixXd gains = MatrixXd::Zero(6,6);
	ik["gain_x"] >> gains(0,0);
	ik["gain_y"] >> gains(1,1);
	ik["gain_z"] >> gains(2,2);
	ik["gain_roll"] >> gains(3,3);
	ik["gain_pitch"] >> gains(4,4);
	ik["gain_yaw"] >> gains(5,5);
	
	// Controller sample time
	dt_ = 1/static_cast<double>(RT_TASK_FREQUENCY);
	
	std::string root_name = "T0"; //FIXME
	std::string end_effector_name;
	if(chain_name_ == "RIGHT_ARM")
		end_effector_name_ = "palm_right"; //wrist_RIGHT
	else if(chain_name_ == "LEFT_ARM")
		 end_effector_name_ = "palm_left";
	else
	{
		M3_ERR("Only RIGHT_ARM and LEFT_ARM are supported");
		return false;
	}

	try
	{
		kin_ = new KDLClik (root_name,end_effector_name_,damp_max,epsilon,gains,dt_);
	}
	catch(const std::runtime_error& e)
	{
		M3_ERR("Failed to create kdl kinematics: ",e.what());
		return false;
	}

	
	return true;
}

void VfForceController::StepStatus()
{
	
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
            
 
        
            
	user_torques_ = torques_status_ - torques_id_;

        //for(int i=0;i<Ndof_controlled_;i++)
        //    user_torques_[i] = user_torques_[i];

        
	kin_->ComputeJac(position_status_,jacobian_);
        
	//std::cout<<jacobian_<<std::endl;
	

	jacobian_t_= jacobian_.transpose();
	

	//std::cout<<jacobian_t_<<std::endl;
	//getchar();
	
	//jacobian_.transposeInPlace(); //NOTE No rt safe!!! http://eigen.tuxfamily.org/dox-devel/classEigen_1_1DenseBase.html#ac501bd942994af7a95d95bee7a16ad2a
	
	// IK
	//Eigen::JacobiSVD<Eigen::MatrixXd> svd;
	svd_->compute(jacobian_t_, ComputeThinU | ComputeThinV);
	//Eigen::VectorXd svd_vect = svd.singularValues();
        svd_vect_ = svd_->singularValues();
	damp_max = 0.0;
	epsilon = 0.01;
	for (int i = 0; i < svd_vect_.size(); i++)
	{
		svd_curr = svd_vect_[i];
		damp = std::exp(-4/epsilon*svd_vect_[i])*damp_max;
		svd_vect_[i] = svd_curr/(svd_curr*svd_curr+damp*damp);
	}
	jacobian_t_pinv_ = svd_->matrixV() * svd_vect_.asDiagonal() * svd_->matrixU().transpose();
	// END IK

	f_user_ = jacobian_t_pinv_ * user_torques_;
	
                
//         std::cout<<"***"<<std::endl;
//         std::cout<<f_user_<<std::endl;
        
        
	// Robot cart stuff
	kin_->ComputeFk(position_status_,cart_pos_status_);
	kin_->ComputeFkDot(position_status_,velocity_status_,cart_vel_status_);
	
	// MV Stuff
	vm_->Update(cart_pos_status_,cart_vel_status_,dt_);
	vm_->getState(vm_state_);
	vm_->getStateDot(vm_state_dot_);
	
	K_ = vm_->getK();
	B_ = vm_->getB();
	
	f_vm_ = K_ * (vm_state_ - cart_pos_status_) + B_ * (vm_state_dot_ - cart_vel_status_);

	//std::cout<<"HELLO"<<std::endl;
	//getchar();
	
	rt_publishers_path_.PublishAll();
	rt_publishers_wrench_.PublishAll();
	
	M3Controller::StepStatus(); // Update the status sds
}

void VfForceController::StepCommand()
{	
	M3Controller::StepCommand(); // Update the command sds
	
	f_cmd_ = f_vm_ - f_user_;

         //for(int i=0;i<Ndof_controlled_;i++)
        jacobian_t_reduced_.row(0) = jacobian_t_.row(0);
        jacobian_t_reduced_.row(1) = jacobian_t_.row(1);
        jacobian_t_reduced_.row(2) = jacobian_t_.row(2);
        jacobian_t_reduced_.row(3) = jacobian_t_.row(3);

        
        torques_cmd_ = jacobian_t_reduced_ * f_cmd_;
        
        
        
        //torques_cmd_ = jacobian_t_ * f_cmd_;
	
	//fd_ = (c_*D_*f_ + (1-c_)*(I_-D_)*f_);
	//joints_torques_cmd_ = jacobian_.transpose() * fd_;
	
	// Compute IK
	//kin_->clikCommandStep(joints_pos_status_,cart_pos_cmd_,joints_pos_cmd_);
	
	
	//torques_cmd_[0] = 0.8;
        //torques_cmd_[3] = 0.8;
	//joints_torques_cmd_.head(3) = torques_cmd_;

// 	joints_torques_cmd_[0] = 0.8;
//         joints_torques_cmd_[3] = 0.8;
// 	M3Controller::StepMotorsCommand(joints_torques_cmd_);
	
	

	
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


	
	
	
}

}