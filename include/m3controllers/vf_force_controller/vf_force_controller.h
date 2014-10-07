#ifndef VF_FORCE_CONTROLLER_H
#define VF_FORCE_CONTROLLER_H

////////// M3_CONTROLLER_INTERFACE
#include "m3controllers/m3_controller_interface/m3_controller_interface.h"

////////// CART_CONTROLLER
//#include "m3controllers/cart_controller/cart_controller.h"

////////// Eigen
#include <eigen3/Eigen/Core>

////////// Google protobuff
//#include <google/protobuf/message.h>
//#include "m3controllers/vf_controller/vf_controller.pb.h"

////////// KDL_KINEMATICS
#include <kdl_kinematics/kdl_kinematics.h>

////////// VIRTUAL_MECHANISM
#include <virtual_mechanism/virtual_mechanism_gmr.h>

////////// Function Approximator
#include <functionapproximators/FunctionApproximatorGMR.hpp>
#include <functionapproximators/MetaParametersGMR.hpp>
#include <functionapproximators/ModelParametersGMR.hpp>

////////// Boost
# include <boost/thread.hpp>

////////// Atomic
# include <atomic>

////////// Activate some timing infos
static int tmp_dt_status_;
static int tmp_dt_cmd_;
static int tmp_dt_computation_;
static long long start_dt_status_,  end_dt_status_, elapsed_dt_status_;
static long long start_dt_cmd_,     end_dt_cmd_,    elapsed_dt_cmd_;
static long long start_dt_computation_,  end_dt_computation_, elapsed_dt_computation_;
#ifdef TIMING
#define TIME_ACTIVE 1
#else
#define TIME_ACTIVE 0
#endif

#define NANO2SEC(a)	a/1e9
#define SEC2NANO(a)	a*1e9

#define INIT_CNT(cnt) do { if (TIME_ACTIVE) (cnt) = 0; } while (0) 
#define SAVE_TIME(out) do { if (TIME_ACTIVE) getCpuCount((out)); } while (0)
#define PRINT_TIME(T_start,T_end,cnt,string) do { if (TIME_ACTIVE) if ((cnt)%100==0) ROS_INFO("%s: %fs",string,count2Sec(((T_end) - (T_start)))); cnt = cnt++ & INT_MAX;} while (0)

inline void getCpuCount(long long& out)
{
    out = nano2count(rt_get_cpu_time_ns());
}

inline double count2Sec(const long long in)
{
    return (NANO2SEC((double)count2nano(in)));
}

bool rtTaskInit(RT_TASK* rt_task ,std::string task_name, double dt)
{
    rt_allow_nonroot_hrt();							
    //Args: Name, Priority, Stack Size, max_msg_size, Policy, cpus_allowed
    if (!(rt_task = rt_task_init_schmod(nam2num(task_name.c_str()),0,0,0,SCHED_FIFO,0xF)))
    {
	std::string err("Cannot create real time task: "+task_name);
	throw std::runtime_error(err);
	return false;
    }
    // Set the task period
    RTIME tick_period = nano2count(SEC2NANO(dt)); // This is ~=dt;
    rt_task_make_periodic(rt_task, rt_get_time() + tick_period, tick_period);
    mlockall(MCL_CURRENT | MCL_FUTURE); // Prevent memory swaps
    
#ifdef HARD_RT
    rt_make_hard_real_time();
#endif
    return true;
}



namespace m3controllers
{
  
  typedef DmpBbo::FunctionApproximatorGMR fa_t;
  typedef Eigen::JacobiSVD<Eigen::MatrixXd> svd_t;
  
class VmThread
{
  typedef boost::mutex mutex_t;
  
  public:
    
    VmThread(std::string name):kill_thread_(false)//,run_status_(false)
    {
      name_ = name;
    }
    
    ~VmThread()
    {
     if(kin_!=NULL)
       delete kin_;
     
     for(int i=0; i<vm_nb_; i++)
      delete vm_vector_[i];
     
    }
    
    bool Startup(std::vector<std::string> file_names, std::string chain_name, int Ndof_controlled, ros::NodeHandle* ros_nh_ptr)
    {
      
      assert(ros_nh_ptr != NULL);
      ros_nh_ptr_ = ros_nh_ptr;
      
      assert(Ndof_controlled >= 1);
      Ndof_controlled_ = Ndof_controlled;
      
      //////// CREATE THE FUNCTION APPROXIMATORS
      boost::shared_ptr<fa_t> tmp_fa_ptr;
      for(int i=0;i<file_names.size();i++)
	{
	  // Read from file the inputs / targets
	  std::vector<std::vector<double> > data;
	  tools::ReadTxtFile(file_names[i].c_str(),data);
	  // CONVERT TO EIGEN MATRIX
	  MatrixXd inputs(data.size(), 1);
	  MatrixXd targets(data.size(), data[0].size()-1); // NOTE Skip time
	  for (int i = 0; i < data.size(); i++)
	  {
	    targets.row(i) = VectorXd::Map(&data[i][1],data[0].size()-1);
	    inputs.row(i) = VectorXd::Map(&data[i][0],1);
	  }
	  
	  // MAKE THE FUNCTION APPROXIMATORS
	  int input_dim = 1;
	  int n_basis_functions = 25;
	  
	  // GMR
	  DmpBbo::MetaParametersGMR* meta_parameters_gmr = new DmpBbo::MetaParametersGMR(input_dim,n_basis_functions);
	  DmpBbo::FunctionApproximatorGMR* fa_ptr = new DmpBbo::FunctionApproximatorGMR(meta_parameters_gmr);
	  
	  // TRAIN
	  fa_ptr->train(inputs,targets);
	  
	  // CREATE THE LIST OF FAs
	  tmp_fa_ptr.reset(fa_ptr); // MAKE SHARED POINTER
	  fa_vector_.push_back(tmp_fa_ptr);
	}
	
	// Controller sample time
	dt_ = 1/static_cast<double>(RT_TASK_FREQUENCY);
	
	//////// CREATE THE INVERSE KINEMATICS
	root_name_ = "T0"; //FIXME
	end_effector_name_ = "palm_right";
	double damp_max = 0.0;
	double epsilon = 0.001;
	if(chain_name == "RIGHT_ARM")
		end_effector_name_ = "palm_right"; //wrist_RIGHT
	else if(chain_name == "LEFT_ARM")
		 end_effector_name_ = "palm_left";
	else
	{
		M3_ERR("Only RIGHT_ARM and LEFT_ARM are supported");
		return false;
	}
	try
	{
		kin_ = new kdl_kinematics::KDLKinematics(root_name_,end_effector_name_,damp_max,epsilon);
	}
	catch(const std::runtime_error& e)
	{
		M3_ERR("Failed to create kdl kinematics: ",e.what());
		return false;
	}

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
		
	// Set the kinematic mask
	kin_->setMask("1,1,1,0,0,0"); // xyz

	cart_size_ = kin_->getCartSize();
	
	// Cart resize
	cart_pos_status_.resize(cart_size_);
	cart_pos_cmd_.resize(cart_size_);
	cart_vel_status_.resize(cart_size_);
	
	// Number of virtual mechanisms
	vm_nb_ = fa_vector_.size();
	
	cart_t vect(cart_size_);
	vect.fill(0.0);
	for(int i=0; i<vm_nb_;i++)
	{
	  vm_vector_ .push_back(new virtual_mechanism_gmr::VirtualMechanismGmr(cart_size_,fa_vector_[i]));
	  vm_state_.push_back(vect);
	  vm_state_dot_.push_back(vect);
	}
	
	// User velocity, vf and joint vel commands
	jacobian_.resize(3,Ndof_controlled_);
	jacobian_t_.resize(Ndof_controlled_,3);
	jacobian_t_pinv_.resize(Ndof_controlled_,3);
        jacobian_t_reduced_.resize(4,3);
	scales_.resize(vm_nb_);
	f_user_.resize(3);
	f_vm_.resize(3);
	f_cmd_.resize(3);
	
	// Clear
	f_user_.fill(0.0);
	f_vm_.fill(0.0);
	f_cmd_.fill(0.0);
	scales_ .fill(0.0);
	
	treshold_ = 0.5;

        svd_vect_.resize(3);
        svd_.reset(new svd_t(3,Ndof_controlled_));

	
#ifdef USE_ROS_RT_PUBLISHER
	if(ros::master::check()){ 

		boost::shared_ptr<tools::RealTimePublisherWrench> tmp_ptr = NULL;
		tmp_ptr = boost::make_shared<tools::RealTimePublisherWrench>(*ros_nh_ptr_,"vm_force",root_name_);
		rt_publishers_wrench_.AddPublisher(tmp_ptr,&f_vm_);
		tmp_ptr = boost::make_shared<tools::RealTimePublisherWrench>(*ros_nh_ptr_,"user_force",root_name_);
		rt_publishers_wrench_.AddPublisher(tmp_ptr,&f_user_);
		tmp_ptr = boost::make_shared<tools::RealTimePublisherWrench>(*ros_nh_ptr_,"cmd_force",root_name_);
		rt_publishers_wrench_.AddPublisher(tmp_ptr,&f_cmd_);

 		rt_publishers_path_.AddPublisher(*ros_nh_ptr_,"robot_pos",cart_pos_status_.size(),&cart_pos_status_);
		for(int i=0; i<vm_nb_;i++)
		{
		  std::string topic_name = "vm_pos_" + std::to_string(i+1);
		  rt_publishers_path_.AddPublisher(*ros_nh_ptr_,topic_name,vm_state_[i].size(),&vm_state_[i]);
		}
	}
#endif	

    }
    
    void RunThread() 
    {
      // check if already triggered
      boost::unique_lock<mutex_t> guard(mtx_run_, boost::defer_lock);
      if(guard.try_lock())
      {
        //run_status_ = true;
        //guard.unlock(); get unlocked after the scope anyways
     
	INIT_CNT(tmp_dt_computation_);
      }
    }
    
    void Update()
    {
      while(!kill_thread_)
      {
	SAVE_TIME(start_dt_computation_);
        //r_.reset();
        computeNewCmd();
        //run_status_ = false;
        //r_.sleep();
	SAVE_TIME(end_dt_computation_);
	PRINT_TIME(start_dt_computation_,end_dt_computation_,tmp_dt_computation_,"computation");
      }
    }
    
    void computeNewCmd()
    {
      boost::unique_lock<mutex_t> guard(mtx_run_,boost::defer_lock);
      
      // START COMPUTATIONS
      guard.lock();
      
      kin_->ComputeJac(position_status_,jacobian_);
      jacobian_t_= jacobian_.transpose();
      
      // IK
      //Eigen::JacobiSVD<Eigen::MatrixXd> svd;
      svd_->compute(jacobian_t_, ComputeThinU | ComputeThinV);
      //Eigen::VectorXd svd_vect = svd.singularValues();
      svd_vect_ = svd_->singularValues();
      double damp_max = 0.0;
      double epsilon = 0.01;
      for (int i = 0; i < svd_vect_.size(); i++)
      {
	      svd_curr = svd_vect_[i];
	      double damp = std::exp(-4/epsilon*svd_vect_[i])*damp_max;
	      svd_vect_[i] = svd_curr/(svd_curr*svd_curr+damp*damp);
      }
      jacobian_t_pinv_ = svd_->matrixV() * svd_vect_.asDiagonal() * svd_->matrixU().transpose();
      // END IK
      
      f_user_ = jacobian_t_pinv_ * torques_status_;
      
      // Robot cart stuff
      kin_->ComputeFk(position_status_,cart_pos_status_);
      kin_->ComputeFkDot(position_status_,velocity_status_,cart_vel_status_);
      
      // Update the vms, take their distances
      for(int i=0; i<vm_nb_;i++)
      {
	vm_vector_[i]->Update(cart_pos_status_,cart_vel_status_,dt_);
	scales_(i) = 1/(vm_vector_[i]->getDistance(cart_pos_status_)+0.001); // NOTE 0.001 it's kind of eps to avoid division by 0
      }

      // Compute and adapt the scales
      for(int i=0; i<vm_nb_;i++)
      {
	scales_(i) = (treshold_ - scales_(i)/scales_.sum())/(treshold_ - 1);
	
      }
      // Compute the force from the vms
      f_vm_.fill(0.0);
      for(int i=0; i<vm_nb_;i++)
      {  
	vm_vector_[i]->getState(vm_state_[i]);
	vm_vector_[i]->getStateDot(vm_state_dot_[i]);
      
	K_ = vm_vector_[i]->getK();
	B_ = vm_vector_[i]->getB();
	
	f_vm_ += scales_(i) * (K_ * (vm_state_[i] - cart_pos_status_) + B_ * (vm_state_dot_[i] - cart_vel_status_)); // Sum over all the vms
      }
      
      f_cmd_ = f_vm_ + f_user_;
    
      //for(int i=0;i<Ndof_controlled_;i++)
      jacobian_t_reduced_.row(0) = jacobian_t_.row(0);
      jacobian_t_reduced_.row(1) = jacobian_t_.row(1);
      jacobian_t_reduced_.row(2) = jacobian_t_.row(2);
      jacobian_t_reduced_.row(3) = jacobian_t_.row(3);

      torques_cmd_ = jacobian_t_reduced_ * f_cmd_;
      
      // STOP COMPUTATIONS
      guard.unlock();
      
      rt_publishers_path_.PublishAll();
      rt_publishers_wrench_.PublishAll();
      
    }
    
    void startThread()
    {
     kill_thread_ = false;
     thread_ = boost::thread(&VmThread::Update, this);
    }

    void stopThread()
    {
      thread_.interrupt();
    }
    
    inline void setSharedStatus(const Ref<const VectorXd>& torques_status, const Ref<const VectorXd>& position_status, const Ref<const VectorXd>& velocity_status) // Called externally (Works only if the thread is not running)
    {
      boost::unique_lock<mutex_t> guard(mtx_run_,boost::defer_lock);
      if(guard.try_lock())
      {
	assert(torques_status.size() == torques_status_.size());
	assert(position_status.size() == position_status_.size());
	assert(velocity_status.size() == velocity_status_.size());
	torques_status_ = torques_status;
	position_status_ = position_status;
	velocity_status_ = velocity_status;
	guard.unlock();
      }
    }
    inline void getSharedCmd(Ref<VectorXd> torques_cmd) // Called externally (Works only if the thread is not running)
    {
      boost::unique_lock<mutex_t> guard(mtx_run_,boost::defer_lock);
      if(guard.try_lock())
      {
	assert(torques_cmd.size() == torques_cmd_.size());
	torques_cmd = torques_cmd_;
	guard.unlock();
      }
    }
    
  private:
    /// SHARED VARIABLES
    /// Input state
    joints_t torques_cmd_;
    joints_t torques_status_;
    joints_t position_status_;
    joints_t velocity_status_;
    /// Output state
    cart_t f_cmd_;
    
    /// INTERNAL STUFF
    int Ndof_controlled_;
    int cart_size_;
    cart_t f_user_;
    cart_t f_vm_;
    cart_t cart_pos_status_;
    cart_t cart_pos_cmd_;
    cart_t cart_vel_status_;
    int vm_nb_;
    
    std::string root_name_, end_effector_name_;
    
    tools::RealTimePublishers<tools::RealTimePublisherPath> rt_publishers_path_;
    tools::RealTimePublishers<tools::RealTimePublisherWrench> rt_publishers_wrench_;
    
    ros::NodeHandle* ros_nh_ptr_;
    std::vector<boost::shared_ptr<fa_t> > fa_vector_;
    
    std::vector<cart_t> vm_state_;
    std::vector<cart_t> vm_state_dot_;
    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd jacobian_t_;
    Eigen::MatrixXd jacobian_t_reduced_;
    Eigen::MatrixXd jacobian_t_pinv_;
    
    double treshold_;
    Eigen::VectorXd scales_;
    Eigen::VectorXd svd_vect_;
    boost::shared_ptr<svd_t> svd_;
    double svd_curr, damp_, damp_max_, epsilon_;
    double dt_;
    double B_, K_;
    std::vector<virtual_mechanism_gmr::VirtualMechanismGmr*> vm_vector_;
    kdl_kinematics::KDLKinematics* kin_;
    
    
    /// Thread Stuff
    std::string name_;
    boost::thread thread_;		
    //std::atomic<bool> updated_;
    /// \brief Execution status of the Device, true is running, false is stopped.
    std::atomic<bool> run_status_;
    mutex_t mtx_run_;
    //mutex_t mtx_status_;
    //mutex_t mtx_cmd_;
    
  public: //FIXME Put in a get set function
    std::atomic<bool> kill_thread_;
    
    
};
  
class VfForceController : public M3Controller
{
	public:
		VfForceController():M3Controller(){}
		~VfForceController(){if(vm_thread_!=NULL) delete vm_thread_;}
		
	protected:
		bool ReadConfig(const char* filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
		
	private:
		enum {DEFAULT};
		
		
		std::vector<std::string> file_names_;

		
		std::string dyn_component_name_;
		m3::M3Dynamatics* dyn_component_;

		//std::vector<virtual_mechanism_gmr::VirtualMechanismGmr*> vm_vector_;

		joints_t torques_id_;
		joints_t user_torques_;
                joints_t torques_cmd_;
                joints_t torques_status_;
                joints_t position_status_;
                joints_t velocity_status_;
                
		VmThread* vm_thread_;
		

		int cart_size_;

		cart_t f_cmd_;
		
};


}

#endif


