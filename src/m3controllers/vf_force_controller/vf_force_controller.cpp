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
	
	// Resize
	torques_id_.resize(Ndof_controlled_);
	user_torques_.resize(Ndof_controlled_);
        torques_cmd_.resize(4);
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
	kin_->setMask("1,1,1,1,1,1"); // xyz rpy

	cart_size_ = kin_->getCartSize();
	
	// Cart resize
	cart_pos_status_.resize(cart_size_);
	cart_pos_cmd_.resize(cart_size_);
	cart_vel_status_.resize(cart_size_);
	
	// Number of virtual mechanisms
	vm_nb_ = fa_vector_.size();
	
	cart_t vect(3);
	vect.fill(0.0);
	for(int i=0; i<vm_nb_;i++)
	{
	  vm_vector_ .push_back(new VirtualMechanismGmr(3,fa_vector_[i])); // Should be always 3 xyz
	  vm_state_.push_back(vect);
	  vm_state_dot_.push_back(vect);
          errors_.push_back(vect);
	}
	
	vect.resize(3+3); // NOTE dim = dim(mean) + dim(variance)
	vect.fill(0.0);
	for(int i=0; i<vm_nb_;i++)
	{
	  vm_kernel_.push_back(vect);
	}
	
	for(int i=0; i<vm_nb_;i++)
        {
            vm_vector_[i]->Init();
	    //vm_vector_[i]->setAdaptGains(adapt_gains_[i]);
	    vm_vector_[i]->setWeightedDist(use_weighted_dist_[i]);
        }
	
	// User velocity, vf and joint vel commands
	jacobian_.resize(6,Ndof_);
        
        jacobian_position_.resize(3,4);
        jacobian_orientation_.resize(3,3);
        
	jacobian_t_.resize(4,3); // only pos
	jacobian_t_pinv_.resize(3,4); // only pos
        
        //jacobian_t_reduced_.resize(4,3);

	scales_.resize(vm_nb_);
	phase_.resize(vm_nb_);
	phase_dot_.resize(vm_nb_);
        phase_ddot_.resize(vm_nb_);
        fades_.resize(vm_nb_);
	det_mv_.resize(vm_nb_);
	torque_mv_.resize(vm_nb_);
        power_mv_.resize(vm_nb_);
        Ks_.resize(vm_nb_);
	f_user_.resize(3);
	f_vm_.resize(3);
	f_cmd_.resize(3);
       
	orientation_ref_.resize(3);
	orientation_.resize(3);
        joints_orientation_cmd_.resize(3);
        joints_orientation_dot_.resize(3);
        joint_orientation_.resize(3);
	
	// Clear
	f_user_.fill(0.0);
	f_vm_.fill(0.0);
	f_cmd_.fill(0.0);
	scales_ .fill(0.0);
	phase_.fill(0.0);
	phase_dot_.fill(0.0);
        phase_ddot_.fill(0.0);
	det_mv_.fill(0.0);
	torque_mv_.fill(0.0);
        power_mv_.fill(0.0);
	fades_.fill(0.0);
        Ks_.fill(0.0);
        
        orientation_.fill(0.0);
        orientation_ref_.fill(0.0);

        joints_orientation_cmd_.fill(0.0);
        joints_orientation_dot_.fill(0.0);
        joint_orientation_.fill(0.0);
        
        if (vm_nb_ > 1)
            treshold_ = 1.0/vm_nb_ + 0.1;
        else
            treshold_ = 0.0;
        
	sum_ = 1.0;
        
        open_hand_ = false;
        
        svd_vect_.resize(3);
        svd_.reset(new svd_t(4,3)); // It should have the same dimensionality of the problem
	
	// Reset force filter
	for(int i=0; i<3; i++)
	  force_filters_[i].Reset();
	  
	// Create the scale adapter
	min_jerk_scale_.Create(0,0,0,1,treshold_,0.9);
	  
	// Inverse kinematics pre-allocations
	svd_->compute(jacobian_t_, ComputeThinU | ComputeThinV); // This is not rt safe! We trigger it here to pre-allocate the internal variables
	matrixU_t_.resize(3,4); // Eigen does some tricks with the dimensionalities, check the .h for info
	matrixV_.resize(3,3);
	jacobian_t_pinv_tmp_.resize(3,3);
	

#ifdef USE_ROS_RT_PUBLISHER
	if(ros::master::check()){ 
		boost::shared_ptr<RealTimePublisherWrench> tmp_ptr = NULL;
		tmp_ptr = boost::make_shared<RealTimePublisherWrench>(*ros_nh_ptr_,"vm_force",root_name_);
		rt_publishers_wrench_.AddPublisher(tmp_ptr,&f_vm_);
		tmp_ptr = boost::make_shared<RealTimePublisherWrench>(*ros_nh_ptr_,"user_force",root_name_);
		rt_publishers_wrench_.AddPublisher(tmp_ptr,&f_user_);
		tmp_ptr = boost::make_shared<RealTimePublisherWrench>(*ros_nh_ptr_,"cmd_force",root_name_);
		rt_publishers_wrench_.AddPublisher(tmp_ptr,&f_cmd_);
		
		rt_publishers_values_.AddPublisher(*ros_nh_ptr_,"scales",scales_.size(),&scales_);
                rt_publishers_values_.AddPublisher(*ros_nh_ptr_,"K",Ks_.size(),&Ks_);
		rt_publishers_values_.AddPublisher(*ros_nh_ptr_,"phase",phase_.size(),&phase_);
		
 		rt_publishers_path_.AddPublisher(*ros_nh_ptr_,"robot_pos",cart_pos_status_.size(),&cart_pos_status_);
		for(int i=0; i<vm_nb_;i++)
		{
		  std::string topic_name = "vm_pos_" + std::to_string(i+1);
		  rt_publishers_path_.AddPublisher(*ros_nh_ptr_,topic_name,vm_state_[i].size(),&vm_state_[i]);
                  
                  topic_name = "error_" + std::to_string(i+1);
                  rt_publishers_values_.AddPublisher(*ros_nh_ptr_,topic_name,errors_[i].size(),&errors_[i]);
		  
		  topic_name = "vm_ker_" + std::to_string(i+1);
		  boost::shared_ptr<RealTimePublisherMarkers> tmp_ptr = boost::make_shared<RealTimePublisherMarkers>(*ros_nh_ptr_,topic_name,root_name_);
		  rt_publishers_markers_.AddPublisher(tmp_ptr,&vm_kernel_[i]);
		}
	}
#endif

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

	// RETRAIN THE DATA FROM TXT FILE
	std::vector<std::string> file_names;
	std::string prob_mode_string;
	bool serialized;
	doc["file_names"] >> file_names;
	doc["serialized"] >> serialized;
	doc["adapt_gains"] >> adapt_gains_;
	doc["adapt_gains"] >> use_weighted_dist_;
	doc["prob_mode"] >> prob_mode_string;
	
	if (prob_mode_string == "scaled")
	  prob_mode_ = SCALED;
	else if (prob_mode_string == "conditional")
	  prob_mode_ = CONDITIONAL;
	else if (prob_mode_string == "priors")
	  prob_mode_ = PRIORS;
	else if (prob_mode_string == "mix")
	  prob_mode_ = MIX;
	else
	   prob_mode_ = SCALED; // Default
	
 	for(int i=0;i<file_names.size();i++)
	{
	  // Read from file the inputs / targets
	  std::vector<std::vector<double> > data;
	  ReadTxtFile(file_names[i].c_str(),data);
	  FunctionApproximatorGMR* fa_ptr = NULL;
	  
	  if(!serialized)
	  {
	    // CONVERT TO EIGEN MATRIX
	    //MatrixXd inputs(data.size(), 1);
	    MatrixXd inputs = VectorXd::LinSpaced(data.size(),0.0,1.0);
	    MatrixXd targets(data.size(), data[0].size()-1); // NOTE Skip time
	    for (int i = 0; i < data.size(); i++)
	    {
	      targets.row(i) = VectorXd::Map(&data[i][1],data[0].size()-1);
	      //inputs.row(i) = VectorXd::Map(&data[i][0],1);
	    }
	    
	    // MAKE THE FUNCTION APPROXIMATORS
	    int input_dim = 1;
	    int n_basis_functions = 6;
	    
	    // GMR
	    MetaParametersGMR* meta_parameters_gmr = new MetaParametersGMR(input_dim,n_basis_functions);
	    fa_ptr = new FunctionApproximatorGMR(meta_parameters_gmr);
	    
	    // TRAIN
	    fa_ptr->train(inputs,targets);

	  }
	  else
	  {
	    ModelParametersGMR* model_parameters_gmr = ModelParametersGMR::loadGMMFromMatrix(file_names[i]);
	    fa_ptr = new FunctionApproximatorGMR(model_parameters_gmr);
	  }
	
	  // MAKE SHARED POINTER
	  fa_shr_ptr_.reset(fa_ptr);
	  fa_vector_.push_back(fa_shr_ptr_);
	}
	
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
        
        jacobian_position_ = jacobian_.block<3,4>(0,0);
        
	jacobian_t_= jacobian_position_.transpose();
	
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
	
	// Update the vms, take their distances
	for(int i=0; i<vm_nb_;i++)
	{
	  vm_vector_[i]->Update(cart_pos_status_.segment<3>(0),cart_vel_status_.segment<3>(0),dt_);
	  
	  switch(prob_mode_) 
	  {
	    case SCALED:
	      scales_(i) = 1/(vm_vector_[i]->getDistance(cart_pos_status_.segment<3>(0)) + 0.001); // NOTE 0.001 it's kind of eps to avoid division by 0
	      break;
	    case CONDITIONAL:
	      scales_(i) = vm_vector_[i]->getProbability(cart_pos_status_.segment<3>(0));
	      break;
	    case PRIORS:
	      scales_(i) = std::exp(-10*vm_vector_[i]->getDistance(cart_pos_status_.segment<3>(0)));
	      break;
	    case MIX:
	      scales_(i) = vm_vector_[i]->getProbability(cart_pos_status_.segment<3>(0));
	      break;
	    default:
	      break;
	  }
	    
          //fades_(i) = vm_vector_[i]->getFade();
	  phase_(i) = vm_vector_[i]->getPhase();
	  //phase_dot_(i) = vm_vector_[i]->getPhaseDot();
          //phase_ddot_(i) = vm_vector_[i]->getPhaseDDot();
	  //det_mv_(i) = vm_vector_[i]->getDet();
	  //torque_mv_(i) = vm_vector_[i]->getTorque();
          //power_mv_(i) = torque_mv_(i)*phase_dot_(i);
          
          if(phase_(i)>= 0.90)
              open_hand_ = true;
          if(phase_(i) <= 0.01)
              open_hand_ = false;
          
	}
	
	user_torques_ = torques_status_ - torques_id_;
	
        f_user_.noalias() = jacobian_t_pinv_ * (-1) * user_torques_.segment<4>(0);
	
        // Filter the user force
        for(int i=0; i<3; i++)
        {
          force_filters_[i].Step(f_user_(i),0.0);
          f_user_(i) =  force_filters_[i].GetX();
        }
        
        //if (f_user_.norm() < 2.0 && f_user_.norm() > -2.0)
        /*if (f_user_.norm() < 4.0 && f_user_.norm() > -4.0)
        {
            f_user_.fill(0.0);
            for(int i=0; i<vm_nb_;i++)
            {
                //std::cout<<"Active "<<f_user_.norm()<<std::endl;
                vm_vector_[i]->setActive(true);
            }
        }
        else
            for(int i=0; i<vm_nb_;i++)
            {
                //std::cout<<"Deactive "<<f_user_.norm()<<std::endl;
                vm_vector_[i]->setActive(false);
            }*/
	

	sum_ = scales_.sum();

	// Compute and adapt the scales
	for(int i=0; i<vm_nb_;i++)
	{
	  switch(prob_mode_) 
	  {
	    case SCALED:
	      // Minjerk scaling
	      if(scales_(i)/sum_ > treshold_)
	      {
		min_jerk_scale_.Compute(scales_(i)/sum_);
		scales_(i) = min_jerk_scale_.GetX();
	      }
	      else
		scales_(i) = 0.0;
	      // Linear scaling
	      /*if(scales_(i)/sum_ > treshold_)
		scales_(i) = (treshold_ - scales_(i)/sum_)/(treshold_ - 1);
	      else
		scales_(i) = 0.0;*/
	      break;
	    case CONDITIONAL:
	      scales_(i) =  scales_(i)/sum_;
	      break;
	    case PRIORS:
	      scales_(i) = scales_(i);
	      break;
	     case MIX:
	      scales_(i) = std::exp(-10*vm_vector_[i]->getDistance(cart_pos_status_.segment<3>(0))) * scales_(i)/sum_;
	      break;
	    default:
	      break;
	  }

	}
	// Compute the force from the vms
	f_vm_.fill(0.0);
	for(int i=0; i<vm_nb_;i++)
	{  
	  vm_vector_[i]->getState(vm_state_[i]);
	  vm_vector_[i]->getStateDot(vm_state_dot_[i]);
	  
	  vm_vector_[i]->getLocalKernel(vm_kernel_[i]);
	
	  K_ = vm_vector_[i]->getK();
	  B_ = vm_vector_[i]->getB();
          
          Ks_(i) = K_;
	  
          //errors_[i] = (vm_state_[i] - cart_pos_status_.segment<3>(0));
          
	  f_vm_ += scales_(i) * (K_ * (vm_state_[i] - cart_pos_status_.segment<3>(0)) + B_ * (vm_state_dot_[i] - cart_vel_status_.segment<3>(0))); // Sum over all the vms
	}
	
	rt_publishers_path_.PublishAll();
	rt_publishers_wrench_.PublishAll();
	rt_publishers_values_.PublishAll();
	rt_publishers_markers_.PublishAll();
	
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

        f_cmd_ = f_vm_;
        
         //for(int i=0;i<Ndof_controlled_;i++)
        /*jacobian_t_reduced_.row(0) = jacobian_t_.row(0);
        jacobian_t_reduced_.row(1) = jacobian_t_.row(1);
        jacobian_t_reduced_.row(2) = jacobian_t_.row(2);
        jacobian_t_reduced_.row(3) = jacobian_t_.row(3);*/
	
        torques_cmd_.noalias() = jacobian_t_ * f_cmd_;
        
        /*orientation_ = cart_pos_status_.segment<3>(3);
        
        Eigen::AngleAxisd rollAngle(orientation_(2), Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle(orientation_(1), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle(orientation_(0), Eigen::Vector3d::UnitX());
        Eigen::Quaternion<double> q = rollAngle  * pitchAngle *  yawAngle;
        Eigen::Quaternion<double> qref(1.0,0.0,0.0,0.0);
        rotArm = q.matrix();
        rotRef = qref.matrix();
        rotWrist = rotArm.transpose() * rotRef;
        
        // Original
        /*joints_orientation_cmd_(2) = -std::atan2(rotWrist(1,0),rotWrist(0,0));
        joints_orientation_cmd_(1) = std::atan2(rotWrist(2,0),std::sqrt(std::pow(rotWrist(2,1),2) + std::pow(rotWrist(2,2),2)));
        joints_orientation_cmd_(0) = std::atan2(rotWrist(2,1),rotWrist(2,2));
        
        
        joints_orientation_cmd_(2) = -std::atan2(rotWrist(1,0),rotWrist(0,0));
        joints_orientation_cmd_(1) = std::atan2(rotWrist(2,0),std::sqrt(std::pow(rotWrist(2,1),2) + std::pow(rotWrist(2,2),2)));
        joints_orientation_cmd_(0) = std::atan2(rotWrist(2,1),rotWrist(2,2));
        
        
        joint_orientation_ = position_status_.segment<3>(4);
        
        joints_orientation_dot_ = 1000 * (joints_orientation_cmd_ - joint_orientation_);
        joints_orientation_cmd_ = joints_orientation_dot_ * dt_ + joint_orientation_;
        */
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

            if(open_hand_)
            {
                for(int i=0;i<5;i++)
                {
                    bot_->SetStiffness(hand_chain_,i,1.0);
                    bot_->SetSlewRateProportional(hand_chain_,i,1.0);
                    bot_->SetModeThetaGc(hand_chain_,i);
                    bot_->SetThetaDeg(hand_chain_,i,0.0);
                }
            }
            else
            {
                for(int i=0;i<5;i++)
                {
                    bot_->SetStiffness(hand_chain_,i,1.0);
                    bot_->SetSlewRateProportional(hand_chain_,i,1.0);
                    bot_->SetModeTorqueGc(hand_chain_,i);
                    bot_->SetTorque_mNm(hand_chain_,i,m2mm(1));
                }
            }
            
            /*bot_->SetStiffness(chain_,4,1.0);
            bot_->SetSlewRateProportional(chain_,4,1.0);
            bot_->SetModeThetaGc(chain_,4);
            bot_->SetThetaDeg(chain_,4,RAD2DEG(joints_orientation_cmd_(0)));

            bot_->SetStiffness(chain_,5,1.0);
            bot_->SetSlewRateProportional(chain_,5,1.0);
            bot_->SetModeThetaGc(chain_,5);
            bot_->SetThetaDeg(chain_,5,RAD2DEG(joints_orientation_cmd_(1)));

            bot_->SetStiffness(chain_,6,1.0);
            bot_->SetSlewRateProportional(chain_,6,1.0);
            bot_->SetModeThetaGc(chain_,6);
            bot_->SetThetaDeg(chain_,6,RAD2DEG(joints_orientation_cmd_(2)));*/

          }
          else
          {
            bot_->SetMotorPowerOn();
            for(int i=0;i<4;i++)
            {
                bot_->SetStiffness(chain_,i,0.0);
                bot_->SetSlewRateProportional(chain_,i,1.0);
                bot_->SetModeTorqueGc(chain_,i);
                //bot_->SetTorque_mNm(chain_,i,m2mm(user_torques_[i]));
            }
            
            for(int i=4;i<Ndof_;i++)
            {
                bot_->SetStiffness(chain_,i,1.0);
                bot_->SetSlewRateProportional(chain_,i,1.0);
                bot_->SetModeThetaGc(chain_,i);
                bot_->SetThetaDeg(chain_,i,0.0);
            }

            /*bot_->SetStiffness(chain_,4,1.0);
            bot_->SetSlewRateProportional(chain_,4,1.0);
            bot_->SetModeThetaGc(chain_,4);
            bot_->SetThetaDeg(chain_,4,RAD2DEG(joints_orientation_cmd_(0)));

            bot_->SetStiffness(chain_,5,1.0);
            bot_->SetSlewRateProportional(chain_,5,1.0);
            bot_->SetModeThetaGc(chain_,5);
            bot_->SetThetaDeg(chain_,5,RAD2DEG(joints_orientation_cmd_(1)));

            bot_->SetStiffness(chain_,6,1.0);
            bot_->SetSlewRateProportional(chain_,6,1.0);
            bot_->SetModeThetaGc(chain_,6);
            bot_->SetThetaDeg(chain_,6,RAD2DEG(joints_orientation_cmd_(2)));*/
            
            
            if(open_hand_)
            {
                for(int i=0;i<5;i++)
                {
                    bot_->SetStiffness(hand_chain_,i,1.0);
                    bot_->SetSlewRateProportional(hand_chain_,i,1.0);
                    bot_->SetModeThetaGc(hand_chain_,i);
                    bot_->SetThetaDeg(hand_chain_,i,0.0);
                }
            }
            else
            {
                for(int i=0;i<5;i++)
                {
                    bot_->SetStiffness(hand_chain_,i,1.0);
                    bot_->SetSlewRateProportional(hand_chain_,i,1.0);
                    bot_->SetModeTorqueGc(hand_chain_,i);
                    bot_->SetTorque_mNm(hand_chain_,i,m2mm(1));
                }
            }

          }

	SAVE_TIME(end_dt_cmd_);
        PRINT_TIME(start_dt_cmd_,end_dt_cmd_,tmp_dt_cmd_,"cmd");
	//Eigen::internal::set_is_malloc_allowed(true);

}

}