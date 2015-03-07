#ifndef VF_FORCE_CONTROLLER_H
#define VF_FORCE_CONTROLLER_H

////////// CMATH
#include <cmath>

////////// M3_CONTROLLER_INTERFACE
#include "m3controllers/m3_controller_interface/m3_controller_interface.h"

////////// CART_CONTROLLER
//#include "m3controllers/cart_controller/cart_controller.h"

////////// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

////////// Google protobuff
//#include <google/protobuf/message.h>
//#include "m3controllers/vf_controller/vf_controller.pb.h"

////////// KDL_KINEMATICS
#include <kdl_kinematics/kdl_kinematics.h>

////////// Mechanism Manager
#include <mechanism_manager/mechanism_manager.h>

////////// Activate some timing infos
static int tmp_dt_status_;
static int tmp_dt_cmd_;
static long long start_dt_status_,  end_dt_status_, elapsed_dt_status_;
static long long start_dt_cmd_,     end_dt_cmd_,    elapsed_dt_cmd_;
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

inline void getCpuCount(long long& out){
    out = nano2count(rt_get_cpu_time_ns());
}

inline double count2Sec(const long long in){
    return (NANO2SEC((double)count2nano(in)));
}


namespace m3controllers
{
  typedef Eigen::JacobiSVD<Eigen::MatrixXd> svd_t;
	
class VfForceController : public M3Controller
{
	public:
		VfForceController():M3Controller(),kin_(NULL){}
		~VfForceController(){if(kin_!=NULL) delete kin_;}
		
	protected:
		bool ReadConfig(const char* filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
		
	private:
		enum {DEFAULT};
		
		std::string dyn_component_name_, cart_mask_str_, end_effector_name_, root_name_;
		m3::M3Dynamatics* dyn_component_;
		
		mechanism_manager::MechanismManager mechanism_manager_;
		
		kdl_kinematics::KDLKinematics* kin_;
		
		joints_t torques_id_;
		joints_t user_torques_;
                joints_t torques_cmd_;
                joints_t torques_status_;
                joints_t position_status_;
                joints_t velocity_status_;
                
		cart_t cart_pos_status_;
		cart_t cart_pos_cmd_;
		cart_t cart_vel_status_;
       
                Eigen::MatrixXd jacobian_position_;
                Eigen::MatrixXd jacobian_orientation_;
                
		Eigen::MatrixXd jacobian_;
		Eigen::MatrixXd jacobian_t_;
                
                //Eigen::MatrixXd jacobian_t_reduced_;
                //Eigen::MatrixXd jacobian_reduced_;
		Eigen::MatrixXd jacobian_t_pinv_;
		
		int cart_size_;
                Eigen::VectorXd svd_vect_;
                boost::shared_ptr<svd_t> svd_;
                
		cart_t f_user_;
		cart_t f_vm_;
		
                double svd_curr, damp, damp_max, epsilon;
		double dt_;
		
		std::vector<m3::M3SensorFilter> force_filters_;
		
		Eigen::MatrixXd matrixU_t_;
		Eigen::MatrixXd matrixV_;
		Eigen::MatrixXd jacobian_t_pinv_tmp_;
		Eigen::VectorXd f_user_tmp_;
		
		Eigen::VectorXd orientation_ref_;
		Eigen::VectorXd orientation_;
                Eigen::VectorXd joints_orientation_cmd_;
                Eigen::VectorXd joints_orientation_dot_;
                Eigen::VectorXd joint_orientation_;

                Eigen::Matrix3d rotArm;
                Eigen::Matrix3d rotRef;
                Eigen::Matrix3d rotWrist;
                
                M3Chain hand_chain_;

                bool open_hand_;
};


}

#endif


