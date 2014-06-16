#ifndef TORQUE_CONTROLLER_H
#define TORQUE_CONTROLLER_H

////////// M3_CONTROLLER_INTERFACE
#include "m3_controller_interface/m3_controller_interface.h"

////////// Eigen
#include <eigen3/Eigen/Core>

namespace m3_controllers
{
	
class TorqueController : public M3Controller
{
	public:
		TorqueController():M3Controller(),l_upper_(0.27862),l_lower_(0.27747),q_upper_(0.0),q_lower_(0.0),loop_cnt_(0){}
	public:
	protected:
		bool ReadConfig(const char* filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();

	private:
		
		double l_upper_, l_lower_, pz_, pzdot_, px_, pxdot_, q_upper_, q_lower_;
		double 	qdot_upper_, qdot_lower_;
		int loop_cnt_;
		double c_;
		
		double s_, sdot_, period_;
		Eigen::MatrixXd P_;
		Eigen::MatrixXd T_;
		Eigen::MatrixXd Pc_;
		Eigen::MatrixXd Pd_;
		Eigen::MatrixXd Pi_;
		Eigen::MatrixXd Pf_;
		Eigen::VectorXd v_;
		Eigen::VectorXd vd_;
		Eigen::VectorXd qdot_;
		Eigen::VectorXd qdot_gen_;
		Eigen::VectorXd q_;
		Eigen::MatrixXd J_;
		
};


}

#endif


