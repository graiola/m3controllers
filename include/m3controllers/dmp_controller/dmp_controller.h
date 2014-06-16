#ifndef DMP_CONTROLLER_H
#define DMP_CONTROLLER_H

////////// CART_CONTROLLER
#include "m3controllers/cart_controller/cart_controller.h"

////////// Eigen
#include <eigen3/Eigen/Core>

////////// Google protobuff
#include <google/protobuf/message.h>
#include "m3controllers/dmp_controller/dmp_controller.pb.h"

////////// BOOST
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/vector.hpp>

////////// DMP
#include <dmp/Dmp.hpp>
#include <dmp/DmpContextual.hpp>
#include <dmp/DmpContextualOneStep.hpp>
#include <dmp/DmpContextualTwoStep.hpp>
#include <dmp/Trajectory.hpp>
#include <dynamicalsystems/DynamicalSystem.hpp>
#include <dynamicalsystems/ExponentialSystem.hpp>
#include <dynamicalsystems/SigmoidSystem.hpp>
#include <dynamicalsystems/TimeSystem.hpp>
#include <dynamicalsystems/SpringDamperSystem.hpp>
#include <functionapproximators/FunctionApproximatorLWR.hpp>
#include <functionapproximators/MetaParametersLWR.hpp>
#include <functionapproximators/ModelParametersLWR.hpp>

namespace m3controllers
{
	typedef DmpBbo::Dmp dmp_t;
	
class DmpController : public m3rt::M3Component
{
	public:
		DmpController():m3rt::M3Component(MAX_PRIORITY),kin_component_(NULL),samples_to_execute_(0),samples_executed_(0),dmp_ptr_(NULL){RegisterVersion("default",DEFAULT);}
		~DmpController(){if(dmp_ptr_!=NULL) delete dmp_ptr_; if(ros_nh_ptr_!=NULL) delete ros_nh_ptr_;};
		
		google::protobuf::Message*  GetCommand(){return &dmp_controller_status_;} //NOTE make abstract M3Component happy
		google::protobuf::Message*  GetStatus(){return &dmp_controller_cmd_;}
		google::protobuf::Message*  GetParam(){return &dmp_controller_param_;}
		
	protected:
		bool ReadConfig(const char* filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
		
		DmpControllerStatus dmp_controller_status_;
		DmpControllerCommand dmp_controller_cmd_;
		DmpControllerParam dmp_controller_param_;
		
		M3BaseStatus* GetBaseStatus(){return dmp_controller_status_.mutable_base();} //NOTE make abstract M3Component happy
		
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
		
	private:
		enum {DEFAULT};
		std::string kin_component_name_, dmp_file_name_;
		CartController* kin_component_;
		
		int samples_to_execute_, samples_executed_;
		
		cart_t cart_pos_status_;
		cart_t cart_pos_cmd_;
		cart_t cart_vel_status_;
		
		int dmp_state_size_;
		Eigen::VectorXd dmp_state_status_;
		Eigen::VectorXd dmp_state_status_dot_;
		Eigen::VectorXd dmp_state_command_;
		Eigen::VectorXd dmp_state_command_dot_;
		Eigen::VectorXd dmp_trajectory_;
		
		/** Pointer to a trained dmp. */
		dmp_t* dmp_ptr_;
		
		double dt_;
};


}

#endif


