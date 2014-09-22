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
#include <virtual_mechanism/virtual_mechanism.h>

typedef Eigen::JacobiSVD<Eigen::MatrixXd> svd_t;

namespace m3controllers
{
	
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
		
		/*VfControllerStatus vf_controller_status_;
		VfControllerCommand vf_controller_cmd_;
		VfControllerParam vf_controller_param_;
		
		M3BaseStatus* GetBaseStatus(){return vf_controller_status_.mutable_base();} //NOTE make abstract M3Component happy*/
/*		
#ifdef USE_ROS_RT_PUBLISHER
		ros::NodeHandle* ros_nh_ptr_;
		tools::RealTimePublishers<tools::RealTimePublisherPath> rt_publishers_;
		
		void RosInit()
		{
			std::string ros_node_name = GetName();
			int argc = 1;
			char* arg0 = strdup(ros_node_name.c_str());
			char* argv[] = {arg0, 0};
			ros::init(argc, argv, ros_node_name,ros::init_options::NoSigintHandler);
			free (arg0);
			
			m3rt::M3_INFO("Checking for running roscore... %s\n",GetName().c_str());
			if(ros::master::check()){
				ros_nh_ptr_ = new ros::NodeHandle(ros_node_name);
			}
			else
			{
				ros_nh_ptr_ = NULL;
				m3rt::M3_INFO("Roscore is not running, can not initializate the realtime publishers for component %s, proceeding without them...\n",GetName().c_str());
			}
		}
		
		void RosShutdown()
		{	if(ros_nh_ptr_!=NULL)
				ros_nh_ptr_->shutdown();
		}
#endif
*/		
	private:
		enum {DEFAULT};

		std::string dyn_component_name_, cart_mask_str_;
		m3::M3Dynamatics* dyn_component_;

		virtual_mechanism::VirtualMechanism* vm_;
		
		kdl_kinematics::KDLClik* kin_;
		
		joints_t torques_id_;
		joints_t user_torques_;

		cart_t cart_pos_status_;
		cart_t cart_pos_cmd_;
		cart_t cart_vel_status_;
		
		cart_t vm_state_;
		cart_t vm_state_dot_;
		
		double c_;
		//long long loop_cnt_;
// 		Eigen::MatrixXd T_;
// 		Eigen::MatrixXd Pi_;
// 		Eigen::MatrixXd Pf_;
// 		Eigen::MatrixXd D_;
// 		Eigen::MatrixXd I_;
		Eigen::MatrixXd jacobian_;
		Eigen::MatrixXd jacobian_t_;
		Eigen::MatrixXd jacobian_t_pinv_;
		
		int cart_size_;
		
                Eigen::VectorXd svd_vect_;
                boost::shared_ptr<svd_t> svd_;
                
		cart_t f_user_;
		cart_t f_vm_;
		cart_t f_cmd_;
		
                double svd_curr, damp, damp_max, epsilon;
		double dt_;
		double B_, K_;
};


}

#endif


