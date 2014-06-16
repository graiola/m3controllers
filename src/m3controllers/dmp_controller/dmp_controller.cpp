#include "m3controllers/dmp_controller/dmp_controller.h"

namespace m3controllers{
	
using namespace m3rt;
using namespace std;
using namespace m3;
using namespace tools;
using namespace kdl_kinematics;
using namespace DmpBbo;
using namespace Eigen;

/*Trajectory generateViapointTrajectory(const VectorXd& ts, const VectorXd& y_first, const VectorXd& y_last, const double& Tf, const double& Ti)
{
    //VectorXd y_first = VectorXd::Zero(n_dims);
    //VectorXd y_last  = VectorXd::Ones(n_dims) * 0.3;
    
    assert(y_first.size() == y_last.size());
    
    int n_dims = y_first.size();
    double viapoint_time = (Tf -Ti)/2;

    VectorXd y_yd_ydd_viapoint = VectorXd::Zero(3*n_dims);
    
    //for(int i = 0; i<n_dims; i++)
    //	   y_yd_ydd_viapoint[i] = (y_last[i] - y_first[i])/2;
    
    y_yd_ydd_viapoint[0] = 0.24;
    y_yd_ydd_viapoint[1] = -0.1;
    y_yd_ydd_viapoint[2] = -0.53;
    
    return  Trajectory::generatePolynomialTrajectoryThroughViapoint(ts,y_first,y_yd_ydd_viapoint,viapoint_time,y_last);
}

dmp_t* generateDemoDmp(const VectorXd y_init, const VectorXd y_attr, const double dt, const int Ndof, const double Ti, const double Tf, int& n_time_steps_trajectory){

	assert(y_init.size() == Ndof);
	assert(y_attr.size() == Ndof);
	
	// GENERATE A TRAJECTORY
	n_time_steps_trajectory = (int)((Tf-Ti)/dt) + 1;
	
	// Some default values for dynamical system
	//VectorXd y_init = VectorXd::Zero(Ndof);
	//VectorXd y_attr  = VectorXd::Ones(Ndof) * 0.4;
	
	VectorXd ts = VectorXd::LinSpaced(n_time_steps_trajectory,Ti,Tf); // From Ti to Tf in n_time_steps_trajectory steps
	
	Trajectory trajectory = generateViapointTrajectory(ts, y_init, y_attr, Tf, Ti);
  
        // MAKE THE FUNCTION APPROXIMATORS
        // Initialize some meta parameters for training LWR function approximator
        int n_basis_functions = 25;
        int input_dim = 1;
        double overlap = 0.01;
        MetaParametersLWR* meta_parameters = new MetaParametersLWR(input_dim,n_basis_functions,overlap);
        FunctionApproximatorLWR* fa_lwr = new FunctionApproximatorLWR(meta_parameters);

	//Dmp::DmpType dmp_type = Dmp::KULVICIUS_2012_JOINING;
	dmp_t::DmpType dmp_type = dmp_t::IJSPEERT_2002_MOVEMENT;
	
	std::vector<FunctionApproximator*> function_approximators(Ndof);    	
	for (int dd=0; dd<Ndof; dd++)
		function_approximators[dd] = fa_lwr->clone();
	
	  //Dmp(double tau, Eigen::VectorXd y_init, Eigen::VectorXd y_attr, std::vector<FunctionApproximator*> //function_approximators, DmpType dmp_type=KULVICIUS_2012_JOINING);
	
	dmp_t* dmp = new dmp_t(Tf,y_init,y_attr,function_approximators,dmp_type);
	
	dmp->train(trajectory);
	
	return dmp; 
}*/

bool DmpController::LinkDependentComponents()
{
	kin_component_ = (CartController*) factory->GetComponent(kin_component_name_);
	if (kin_component_== NULL){
		M3_INFO("CartController component %s not found for component %s\n",kin_component_name_.c_str(),GetName().c_str());
		return false;
	}
	
	return true;
}

void DmpController::Startup()
{	
	if (kin_component_ == NULL)
		SetStateError();
	else
		SetStateSafeOp();
		
	// Resize
	cart_pos_status_.resize(6); // NOTE always six
	cart_pos_cmd_.resize(6); // NOTE always six
	cart_vel_status_.resize(6); // NOTE always six
	// Clear
	cart_pos_status_.fill(0.0);
	cart_pos_cmd_.fill(0.0);
	cart_vel_status_.fill(0.0);

	kin_component_->EnableInternalCommunication(); // Enable the communication with it.
	
	// Controller sample time
	dt_ = 1/static_cast<double>(RT_TASK_FREQUENCY);
		
	// Dmp Creation
	/*double Tf = 6; //sec
	double Ti = 0.0;
	double dt = 0.025; // For the trajectory
	int n_time_steps_trajectory;
	int Ndof;
	//bool cart_dmp = true;
	//bool closed_loop = true;
	VectorXd y_attr;
	//if(cart_dmp){
		Ndof = 6;
		y_attr.resize(Ndof);
		y_attr << 0.4, -0.1, -0.1, 0.0, 0.0, 0.0; // Cart
	//}
	//else{
	//	Ndof = 7;
	//	y_attr  = VectorXd::Ones(Ndof) * 0.4; // Joints
	//}
	
	//VectorXd y_init = VectorXd::Zero(Ndof);
	VectorXd y_init;
	y_init.resize(Ndof);
	y_init << 0.0, -0.21, -0.6, 0.0, 0.0, 0.0;
	
	// Generate and train a DMP
	dmp_ptr_ = generateDemoDmp(y_init,y_attr,dt,Ndof,Ti,Tf,n_time_steps_trajectory);*/
	
	// Resize the dmp's state vectors
	dmp_state_size_ = dmp_ptr_->dim();
	dmp_state_status_.resize(dmp_state_size_);
	dmp_state_status_dot_.resize(dmp_state_size_);
	dmp_state_command_.resize(dmp_state_size_);
	dmp_state_command_dot_.resize(dmp_state_size_);
	dmp_trajectory_.resize(3);
	// Clear
	dmp_state_status_.fill(0.0);
	dmp_state_status_dot_.fill(0.0);
	dmp_state_command_.fill(0.0);
	dmp_state_command_dot_.fill(0.0);
	dmp_trajectory_.fill(0.0);
	
	// Prepare the state vectors to be integrated by dmp
	dmp_ptr_->integrateStart(dmp_state_status_,dmp_state_status_dot_);
	
#ifdef USE_ROS_RT_PUBLISHER
	RosInit();
	//rt_publishers_.AddPublisher(*ros_nh_ptr_,"dmp_state_status",2,&dmp_trajectory_,y_init);
	rt_publishers_.AddPublisher(*ros_nh_ptr_,"dmp_state_status",2,&dmp_trajectory_);
#endif
}

void DmpController::Shutdown()
{
#ifdef USE_ROS_RT_PUBLISHER
	RosShutdown();
#endif
}
						  
bool DmpController::ReadConfig(const char* cfg_filename)
{
	//YAML::Node doc;
	//GetYamlDoc(cfg_filename, doc);
	
	if (!M3Component::ReadConfig(cfg_filename))
		return false;
	
	doc["kin_name"] >> kin_component_name_;
	doc["dmp_file"] >> dmp_file_name_; // FIXME add the path

	std::ifstream ifs(dmp_file_name_);
	if (ifs.is_open())
	{
		boost::archive::xml_iarchive ia(ifs);
		ia >> BOOST_SERIALIZATION_NVP(dmp_ptr_);
	}
	else
	{
		M3_ERR("Unable to open file : [%s]",dmp_file_name_.c_str());
		return false;
	}
	ifs.close();
	
	return true;
}

void DmpController::StepStatus()
{
	kin_component_->GetStatus(dmp_state_status_.segment(0,6));
	dmp_ptr_->integrateStep(dt_,dmp_state_status_,dmp_state_command_,dmp_state_command_dot_);
	dmp_trajectory_ = dmp_state_command_.segment(0,3);
#ifdef USE_ROS_RT_PUBLISHER
	rt_publishers_.PublishAll();
#endif	
}

void DmpController::StepCommand()
{	
	// Gooooo
	kin_component_->EnableController(); // FIXME it's a bit stupid to call it every loop...
	kin_component_->SetCommand(dmp_state_command_.segment(0,6));
	// Update the gating system state
	dmp_state_status_ =  dmp_state_command_;
}

}
