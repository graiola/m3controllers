#ifndef VF_CONTROLLER_H
#define VF_CONTROLLER_H

////////// M3_CONTROLLER_INTERFACE
#include "m3controllers/m3_controller_interface/m3_controller_interface.h"

////////// CART_CONTROLLER
#include "m3controllers/cart_controller/cart_controller.h"

////////// Eigen
#include <eigen3/Eigen/Core>

////////// Google protobuff
#include <google/protobuf/message.h>
#include "m3controllers/vf_controller/vf_controller.pb.h"

////////// KDL_KINEMATICS
#include <kdl_kinematics/kdl_kinematics.h>

namespace m3controllers
{
	
class VfController : public M3Controller
{
	public:
		VfController():M3Controller(),kin_component_(NULL){}
		~VfController(){if(kin_solver_ptr_!=NULL) delete kin_solver_ptr_;};
		
		/*google::protobuf::Message*  GetCommand(){return &vf_controller_status_;} //NOTE make abstract M3Component happy
		google::protobuf::Message*  GetStatus(){return &vf_controller_cmd_;}
		google::protobuf::Message*  GetParam(){return &vf_controller_param_;}*/
		
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
		std::string kin_component_name_;
		CartController* kin_component_;
		
		//int samples_to_execute_, samples_executed_;
		
		cart_t cart_pos_status_;
		cart_t cart_pos_cmd_;
		cart_t cart_vel_status_;
		
		double c_;
		//long long loop_cnt_;
		Eigen::MatrixXd T_;
		Eigen::MatrixXd Pi_;
		Eigen::MatrixXd Pf_;
		Eigen::MatrixXd D_;
		Eigen::MatrixXd I_;
		
		cart_t v_;
		cart_t vd_;
		
		joints_t joints_vel_cmd_;
		
		//Eigen::VectorXd qdot_;
		//Eigen::VectorXd qdot_gen_;
		//Eigen::VectorXd q_;
		//Eigen::MatrixXd J_;
		
		double dt_;
		
		kdl_kinematics::KDLKinematics* kin_solver_ptr_;
		
		
		
};


}

#endif


