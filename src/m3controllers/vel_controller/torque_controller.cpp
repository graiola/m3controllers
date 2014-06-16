#include "torque_controller.h"

void ForwardKinematics(const double q_1, const double q_2,const double l_1,const double l_2, double& px, double& pz){
	pz = -(l_1 * cos(q_1) + l_2 * cos(q_1 + q_2));
	px = l_1 * sin(q_1) + l_2 * sin(q_1 + q_2);
}

void InverseKinematicsPos(const double px, const double pz, const double l_1, const double l_2, double& q_1, double& q_2){

	double c_lower = (pow(pz,2)+pow(px,2)-pow(l_1,2)-pow(l_2,2))/(2*l_1*l_2);
	double s_lower = (sqrt(1-pow(c_lower,2)));
	
	q_2 = atan2(s_lower,c_lower);
	q_1 = atan2(px,-pz) - atan2(l_2*s_lower,l_1+l_2*c_lower);
}

void ComputeJacobian(const double q_1, const double q_2, const double l_1, const double l_2, Eigen::MatrixXd& J){
	J(0,0) = (l_1 * cos(q_1) + l_2 * cos(q_1 + q_2));
	J(0,1) = (l_2 * cos(q_1 + q_2));
	J(1,0) = (l_1 * sin(q_1) + l_2 * sin(q_1 + q_2)); 
	J(1,1) = (l_2 * sin(q_1 + q_2)); 
}	

namespace m3_controllers{
	
using namespace m3rt;
using namespace std;
using namespace m3;
using namespace tools;
using namespace Eigen;

void TorqueController::Startup()
{
	M3Controller::Startup();
	
	P_.resize(2,1);
	T_.resize(2,1);
	Pc_.resize(2,1);
	Pi_.resize(2,1);
	Pf_.resize(2,1);
	v_.resize(2);
	vd_.resize(2);
	qdot_.resize(2);
	qdot_gen_.resize(2);
	J_.resize(2,2);

	Pi_ << 0.4, -0.3;
	Pf_ << 0.4, 0.0;
	period_ = 30; //sec
	c_ = 1;

}

void TorqueController::Shutdown()
{	
	M3Controller::Shutdown();
}
						  
bool TorqueController::ReadConfig(const char* cfg_filename)
{
	YAML::Node doc;
	/*if(!GenerateDocFromCfg(cfg_filename, doc))
		return false;*/

	if(!M3Controller::ReadConfig(cfg_filename))
		return false;
	
	GetYamlDoc(cfg_filename, doc);
	
	return true;
}

void TorqueController::StepStatus()
{	
	q_upper_ = DEG2RAD(bot_->GetThetaDeg(chain_,0));
	q_lower_ = DEG2RAD(bot_->GetThetaDeg(chain_,3));
	qdot_upper_ = DEG2RAD(bot_->GetThetaDotDeg(chain_,0));
	qdot_lower_ = DEG2RAD(bot_->GetThetaDotDeg(chain_,3));
	
	ComputeJacobian(q_upper_,q_lower_,l_upper_,l_lower_,J_);
	
	//qdot_ << qdot_upper_, qdot_lower_;
	//qdot_gen_ << 0.5*sin(2.0 * M_PI * static_cast<double>(loop_cnt_) / (period_ * RT_TASK_FREQUENCY)), 0.0;
	//qdot_gen_ << 0.25*sin(2.0 * M_PI * static_cast<double>(loop_cnt_) / (period_ * RT_TASK_FREQUENCY)), 0.25*sin(2.0 * M_PI * static_cast<double>(loop_cnt_) / (period_ * RT_TASK_FREQUENCY));
	//v_ = J_ * qdot_gen_; // It is a command
	
	//qdot_gen_ << qdot_upper_, qdot_lower_;
	//v_ = J_ * qdot_gen_; // It is a command
	
	//v_ << 0+0*sin(2.0 * M_PI * static_cast<double>(loop_cnt_) / (period_ * RT_TASK_FREQUENCY)), 0.01*sin(2.0 * M_PI * static_cast<double>(loop_cnt_) / (period_ * RT_TASK_FREQUENCY));
	
	// FK
	ForwardKinematics(q_upper_,q_lower_,l_upper_,l_lower_,Pc_(0),Pc_(1));

	// HACK INIT
	/*if(loop_cnt_ == 0){
		Pi_ = Pc_;
		Pf_ = Pc_;
		//Pf_(0) = Pf_(0) + 0.2; 
		Pf_(1) = Pf_(1) + 0.3; 
	}*/
	
	if (loop_cnt_%100==0){
		M3_INFO("** STATUS **\n");
		M3_INFO("px: %f\n",Pc_(0));
		M3_INFO("pz: %f\n",Pc_(1));
		M3_INFO("vx: %f\n",v_[0]);
		M3_INFO("vz: %f\n",v_[1]);
	}
}

void TorqueController::StepCommand()
{	
	//s_ = (Pc_ - Pi_).norm();
	//P_ = Pi_ + s_/(Pf_-Pi_).norm() * (Pf_-Pi_);
	
	T_ = (Pf_-Pi_)/(Pf_-Pi_).norm();
	
	/*std::cout<<"s_"<<std::endl;
	std::cout<<s_<<std::endl;
	std::cout<<"P_"<<std::endl;
	std::cout<<P_<<std::endl;
	std::cout<<"Pc_"<<std::endl;
	std::cout<<Pc_<<std::endl;*/
	
	// IK
	//InverseKinematicsPos(Pc_(0),Pc_(1),l_upper_,l_lower_,q_upper_,q_lower_);
	
	//P_[0] = Pi_[0] + s_/(Pf_-Pi_).norm() * (Pf_[0]-Pi_[0]);
	//P_[1] = Pi_[1] + s_/(Pf_-Pi_).norm() * (Pf_[1]-Pi_[1]);
	
	// Trajectory
	//s_ =  abs(sin(2.0 * M_PI * static_cast<double>(loop_cnt_) / (period_*4 * RT_TASK_FREQUENCY)));
	
	//P_[0] = Pi_[0] + s_ * (Pf_[0]-Pi_[0]);
	//P_[1] = Pi_[1] + s_ * (Pf_[1]-Pi_[1]);
	
	Eigen::MatrixXd D = T_*(T_.transpose()*T_).inverse() * T_.transpose();
	Eigen::MatrixXd I = MatrixXd::Identity(2,2);
	
	vd_ = (c_*D*v_ + (1-c_)*(I-D)*v_);
	
	//qdot_ = J_.inverse() * vd_;
	
	Eigen::JacobiSVD<Eigen::MatrixXd> svd;
	svd.compute(J_, ComputeThinU | ComputeThinV);
	Eigen::VectorXd svd_vect = svd.singularValues();
	double svd_min = svd_vect.minCoeff();
	double svd_curr, damp, damp_max, epsilon;
	damp_max = 0.5;
	epsilon = 0.01;
	for (int i = 0; i < svd_vect.size(); i++)
	{
		svd_curr = svd_vect[i];
		
		damp = std::exp(-4/epsilon*svd_vect[i])*damp_max;
		svd_vect[i] = svd_curr/(svd_curr*svd_curr+damp*damp);
	}
	
	Eigen::MatrixXd J_pinv_ = svd.matrixV() * svd_vect.asDiagonal() * svd.matrixU().transpose();	
	
	qdot_ = J_pinv_ * vd_;
	
	q_upper_ = qdot_[0] * RT_TASK_FREQUENCY + q_upper_;
	q_lower_ = qdot_[1] * RT_TASK_FREQUENCY + q_lower_;
	
	//Pd_ = v_ * RT_TASK_FREQUENCY + Pd_;
	//InverseKinematicsPos(Pc_(0),Pc_(1),l_upper_,l_lower_,q_upper_, q_lower_);
	
	//Pc_ = v_ * RT_TASK_FREQUENCY + Pc_;
	//InverseKinematicsPos(Pc_(0),Pc_(1),l_upper_,l_lower_,q_upper_, q_lower_);
	
	bot_->SetMotorPowerOn();
	for(int i=0; i<Ndof_; i++)
	{
		bot_->SetModeThetaGc(chain_,i);
		bot_->SetStiffness(chain_,i,1.0);
		bot_->SetSlewRateProportional(chain_,i,1.0);
		
		if(i==0) // Upper arm
			bot_->SetThetaDeg(chain_,i,RAD2DEG(q_upper_));
		else if(i==3) // Lower arm
			bot_->SetThetaDeg(chain_,i,RAD2DEG(q_lower_));
		else
			bot_->SetThetaDeg(chain_,i,0.0);
	}
	
	//Eigen::MatrixXd J = bot_->GetJacobian(chain_);
	/*J_(0,0) = J(0,0);
	J_(0,1) = J(0,3);
	J_(1,0) = J(2,0);
	J_(1,1) = J(2,3);*/
	
	if (loop_cnt_%100==0){
		M3_INFO("** COMMAND **\n");
		//M3_INFO("q_upper_: %f\n",q_upper_);
		//M3_INFO("q_lower_: %f\n",q_lower_);
		M3_INFO("vx: %f\n",vd_[0]);
		M3_INFO("vz: %f\n",vd_[1]);
	}
	
	loop_cnt_++;
	
}


}
