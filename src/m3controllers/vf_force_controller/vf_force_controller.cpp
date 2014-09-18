#include "m3controllers/vf_force_controller/vf_force_controller.h"

namespace m3controllers{
	
using namespace m3rt;
using namespace std;
using namespace m3;
using namespace tools;
using namespace kdl_kinematics;
using namespace Eigen;

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
	torques_id_.resize(Ndof_);
	user_torques_.resize(Ndof_);
	// Clear
	torques_id_.fill(0.0);
	user_torques_.fill(0.0);
	
	// Set the kinematic mask
	kin_->setMask("1,1,1,0,0,0"); // xyz

	// Controller sample time
	dt_ = 1/static_cast<double>(RT_TASK_FREQUENCY);

	// User velocity, vf and joint vel commands
	jacobian_.resize(3,Ndof_);
	jacobian_t_pinv_.resize(Ndof_,3);
	f_.resize(3);
	fd_.resize(3);
	T_.resize(3,1);
	Pi_.resize(3,1);
	Pf_.resize(3,1);
        
        svd_vect_.resize(3);
        svd_.reset(new svd_t(3,Ndof_));
	
	// Define the virtual fixture
	c_ = 1;
	Pi_ << 0.0, 0.0, 0.0;
	Pf_ << 0.0, 0.0, 0.5;
	T_ = (Pf_-Pi_)/(Pf_-Pi_).norm();
	D_ = T_*(T_.transpose()*T_).inverse() * T_.transpose();
	I_ = MatrixXd::Identity(3,3);

#ifdef USE_ROS_RT_PUBLISHER
	if(ros::master::check()){ 
                rt_publishers_.AddPublisher(*ros_nh_ptr_,"user_torques",7,&user_torques_);      
                rt_publishers_.AddPublisher(*ros_nh_ptr_,"id_torques",7,&torques_id_);
		rt_publishers_.AddPublisher(*ros_nh_ptr_,"user_force",3,&f_);
		rt_publishers_.AddPublisher(*ros_nh_ptr_,"desired_force",3,&fd_);
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
		kin_ = new KDLKinematics (root_name,end_effector_name,damp_max,epsilon);
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

	for(int i=0;i<Ndof_;i++)
	  torques_id_[i] = - dyn_component_->GetG(i) /1000;
        
	user_torques_ = joints_torques_status_ - torques_id_;

        for(int i=0;i<Ndof_;i++)
            user_torques_[i] = user_torques_[i];
        
	kin_->ComputeJac(joints_pos_status_,jacobian_);

	// IK
	//Eigen::JacobiSVD<Eigen::MatrixXd> svd;
	svd_->compute(jacobian_.transpose(), ComputeThinU | ComputeThinV);
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

	f_ = jacobian_t_pinv_ * user_torques_;
	
        //for(int i=0;i<3;i++)
         //   f_[i] = f_[i];
        
	M3Controller::StepStatus(); // Update the status sds
}

void VfForceController::StepCommand()
{	
	M3Controller::StepCommand(); // Update the command sds
	
	//fd_ = (c_*D_*f_ + (1-c_)*(I_-D_)*f_);
	//joints_torques_cmd_ = jacobian_.transpose() * fd_;
	
	
	joints_torques_cmd_[0] = 0.8;
	joints_torques_cmd_[3] = 0.8;


	M3Controller::StepMotorsCommand(joints_torques_cmd_);
}

}
