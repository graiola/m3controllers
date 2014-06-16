#ifndef TORQUE_CONTROLLER_H
#define TORQUE_CONTROLLER_H

////////// M3_CONTROLLER_INTERFACE
#include "m3_controller_interface/m3_controller_interface.h"

////////// Google protobuff
#include "torque_controller.pb.h"

////////// Eigen
#include <eigen3/Eigen/Core>

namespace m3_controllers
{
	
class TorqueController : public M3Controller
{
	public:
		TorqueController():M3Controller(),dyn_(NULL),l_upper_(0.27862),l_lower_(0.27747),q_upper_(0.0),q_lower_(0.0),loop_cnt_(0){}
		
		~TorqueController(){}
		
		google::protobuf::Message * GetTorqueControllerCommand(){return &torque_controller_command_;}
		google::protobuf::Message * GetTorqueControllerStatus(){return &torque_controller_status_;}
		google::protobuf::Message * GetTorqueControllerParam(){return &torque_controller_param_;}
		
	public:
	protected:
		bool ReadConfig(const char* filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
		
		/** Google protobuf messages */
		TorqueControllerStatus torque_controller_status_;
		TorqueControllerCommand torque_controller_command_;
		TorqueControllerParam torque_controller_param_;
		
	private:
		m3::M3Dynamatics* dyn_;
		double l_upper_, l_lower_, q_upper_, q_lower_, pz_, pzdot_, px_, pxdot_;
		double 	qdot_upper_, qdot_lower_;
		int loop_cnt_;
		double c_;
		
		double s_, sdot_, period_;
		Eigen::MatrixXd P_;
		Eigen::MatrixXd Pi_;
		Eigen::MatrixXd Pf_;
		Eigen::MatrixXd T_;

		Eigen::MatrixXd J_;
		Eigen::VectorXd q_;
		Eigen::VectorXd qdot_;
		Eigen::VectorXd qddot_;
		Eigen::VectorXd torque_;
		Eigen::VectorXd force_;
		Eigen::VectorXd torque_des_;
		Eigen::VectorXd torque_grav_;
		Eigen::VectorXd force_des_;
		
		double file_length_sec_;
		int file_length_samples_;
		std::string force_out_file_name_, torque_out_file_name_;
		std::string pos_out_file_name_, vel_out_file_name_, acc_out_file_name_;
		std::vector<joints_t > force_mat_;
		std::vector<joints_t > torque_mat_;
		std::vector<joints_t > pos_mat_;
		std::vector<joints_t > vel_mat_;
		std::vector<joints_t > acc_mat_;
		
		std::string dyn_name_;
		

};


}

#endif


