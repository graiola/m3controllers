#include "torque_controller.h"

void ForwardKinematics(const double q_1, const double q_2,const double l_1,const double l_2, double& px, double& pz){
	px = l_1 * sin(q_1) + l_2 * sin(q_1 + q_2);
	pz = -(l_1 * cos(q_1) + l_2 * cos(q_1 + q_2));
}

void InverseKinematicsPos(const double px, const double pz, const double l_1, const double l_2, double& q_1, double& q_2){

	double c_lower = (pow(pz,2)+pow(px,2)-pow(l_1,2)-pow(l_2,2))/(2*l_1*l_2);
	double s_lower = (sqrt(1-pow(c_lower,2)));
	
	q_2 = atan2(s_lower,c_lower);
	q_1 = atan2(px,-pz) - atan2(l_2*s_lower,l_1+l_2*c_lower);
}

void ComputeJacobian(const double q_1, const double q_2, const double l_1, const double l_2, Eigen::MatrixXd& J){
	J(0,0) = (l_1 * sin(q_1) + l_2 * sin(q_1 + q_2)); //l_1 * cos(q_1) + l_2 * cos(q_1 + q_2)
	J(0,1) = (l_2 * sin(q_1 + q_2)); //l_2 * cos(q_1 + q_2)
	J(1,0) = (l_1 * cos(q_1) + l_2 * cos(q_1 + q_2)); //l_1 * sin(q_1) + l_2 * sin(q_1 + q_2)
	J(1,1) = (l_2 * cos(q_1 + q_2)); //l_2 * sin(q_1 + q_2)
}	

namespace m3_controllers{
	
using namespace m3rt;
using namespace std;
using namespace m3;
using namespace tools;
using namespace Eigen;
using namespace KDL;

bool TorqueController::LinkDependentComponents()
{
	M3Controller::LinkDependentComponents();
	
	dyn_ = (M3Dynamatics*) factory->GetComponent(dyn_name_);
	if (dyn_==NULL){
		M3_INFO("M3Dynamatics component %s not found for component %s\n",dyn_name_.c_str(),GetName().c_str());
		return false;
	}
	
	return true;
}

void TorqueController::Startup()
{
	M3Controller::Startup();
	
	Ndof_controlled_ = 2; // HACK
	
	torque_.resize(Ndof_controlled_);
	force_.resize(Ndof_controlled_);
	torque_des_.resize(Ndof_controlled_);
	torque_grav_.resize(Ndof_controlled_);
	force_des_.resize(Ndof_controlled_);
	q_.resize(Ndof_controlled_);
	qdot_.resize(Ndof_controlled_);
	qddot_.resize(Ndof_controlled_);
	Pi_.resize(2,1);
	Pf_.resize(2,1);
	Pi_(0) = 0.6; //x
	Pi_(1) = -0.3; //z
	Pf_(0) = 0.6; //x
	Pf_(1) = 0.3; //z
	c_ = 1.2; // 1.2
	J_.resize(2,Ndof_controlled_);
	
	if(file_length_sec_ > 0.0){
		file_length_samples_ = static_cast<int>(file_length_sec_*RT_TASK_FREQUENCY);
		// Allocate memory for the matrix to dump into the output file	
		for(int i = 0; i<file_length_samples_; i++){
			torque_mat_.push_back(joints_t(2));
			force_mat_.push_back(joints_t(2));
			pos_mat_.push_back(joints_t(2));
			vel_mat_.push_back(joints_t(2));
			acc_mat_.push_back(joints_t(2));
		}
	}
	else
		file_length_samples_ = 0;
}
	
void TorqueController::Shutdown()
{
	WriteTxtFile(torque_out_file_name_.c_str(),torque_mat_);
	WriteTxtFile(force_out_file_name_.c_str(),force_mat_);
	WriteTxtFile(pos_out_file_name_.c_str(),pos_mat_);
	WriteTxtFile(vel_out_file_name_.c_str(),vel_mat_);
	WriteTxtFile(acc_out_file_name_.c_str(),acc_mat_);
	
	M3Controller::Shutdown();
}
						  
bool TorqueController::ReadConfig(const char* cfg_filename)
{	
	YAML::Node doc;

	if(!M3Controller::ReadConfig(cfg_filename))
		return false;
	
	GetYamlDoc(cfg_filename, doc);
	
	doc["dyn_name"] >> dyn_name_;
	doc["sec_to_record"] >> file_length_sec_;
	doc["torque_output_file"] >> torque_out_file_name_;
	doc["force_output_file"] >> force_out_file_name_;
	doc["pos_output_file"] >> pos_out_file_name_;
	doc["vel_output_file"] >> vel_out_file_name_;
	doc["acc_output_file"] >> acc_out_file_name_;
	
	return true;
}

void TorqueController::StepStatus()
{	
	torque_(0) = bot_->GetTorque_mNm(chain_,0)/1000; //mNm -> Nm
	torque_(1) = bot_->GetTorque_mNm(chain_,3)/1000;
	
	/*std::cout<<"FROM BOT"<<std::endl;
	torque_grav_(0) = bot_->GetGravity(chain_,0)/1000;
	torque_grav_(1) = bot_->GetGravity(chain_,3)/1000;
	std::cout<<"0)"<<torque_grav_(0)<<std::endl;
	std::cout<<"1)"<<torque_grav_(1)<<std::endl;*/
	
	/*std::cout<<"FROM DYN"<<std::endl;
	torque_grav_(0) = dyn_->GetG(0)/1000;
	torque_grav_(1) = dyn_->GetG(3)/1000;
	std::cout<<"0)"<<torque_grav_(0)<<std::endl;
	std::cout<<"1)"<<torque_grav_(1)<<std::endl;*/
	
	torque_grav_(0) = dyn_->GetG(0)/1000; // It differs from the one computed in the bot ~0.02
	torque_grav_(1) = dyn_->GetG(3)/1000;
	
	q_(0) = DEG2RAD(bot_->GetThetaDeg(chain_,0));
	q_(1) = DEG2RAD(bot_->GetThetaDeg(chain_,3));
	
	qdot_(0) = DEG2RAD(bot_->GetThetaDotDeg(chain_,0));
	qdot_(1) = DEG2RAD(bot_->GetThetaDotDeg(chain_,3));
	
	qddot_(0) = DEG2RAD(bot_->GetThetaDotDotDeg(chain_,0));
	qddot_(1) = DEG2RAD(bot_->GetThetaDotDotDeg(chain_,3));
	
	torque_ = torque_ + torque_grav_;
	
	ComputeJacobian(q_(0),q_(1),l_upper_,l_lower_,J_);
	
	//ForwardKinematics(q_(0),q_(1),l_upper_,l_lower_,px_,pz_);
	
	// IK
	Eigen::JacobiSVD<Eigen::MatrixXd> svd;
	svd.compute(J_.transpose(), ComputeThinU | ComputeThinV);
	Eigen::VectorXd svd_vect = svd.singularValues();
	//double svd_min = svd_vect.minCoeff();
	double svd_curr, damp, damp_max, epsilon;
	damp_max = 0.8;
	epsilon = 0.01;
	for (int i = 0; i < svd_vect.size(); i++)
	{
		svd_curr = svd_vect[i];
		damp = std::exp(-4/epsilon*svd_vect[i])*damp_max;
		svd_vect[i] = svd_curr/(svd_curr*svd_curr+damp*damp);
	}
	Eigen::MatrixXd Jt_pinv_ = svd.matrixV() * svd_vect.asDiagonal() * svd.matrixU().transpose();	
	// END IK
	
	force_ = Jt_pinv_ * torque_;
	
	if(loop_cnt_ < file_length_samples_)
		for(int i = 0; i<2; i++)
		{
			torque_mat_[loop_cnt_][i] = torque_(i);
			force_mat_[loop_cnt_][i] = force_(i);
			pos_mat_[loop_cnt_][i] = q_(i);
			vel_mat_[loop_cnt_][i] = qdot_(i);
			acc_mat_[loop_cnt_][i] = qddot_(i);
		}
	
	if (loop_cnt_%100==0){
		M3_INFO("** STATUS **\n");
		M3_INFO("Fx: %f\n",force_(0));
		M3_INFO("Fz: %f\n",force_(1));
		M3_INFO("T0: %f\n",torque_(0));
		M3_INFO("T3: %f\n",torque_(1));
	}
	
	M3Controller::StepStatus(); // Update status in the sds
}

void TorqueController::StepCommand()
{	
	M3Controller::StepCommand(); // Take cmd from sds
	
	//s_ = (Pc_ - Pi_).norm();
	//P_ = Pi_ + s_/(Pf_-Pi_).norm() * (Pf_-Pi_);
	
	T_ = (Pf_-Pi_)/(Pf_-Pi_).norm();
	
	//P_[0] = Pi_[0] + s_/(Pf_-Pi_).norm() * (Pf_[0]-Pi_[0]);
	//P_[1] = Pi_[1] + s_/(Pf_-Pi_).norm() * (Pf_[1]-Pi_[1]);
	
	// Trajectory
	//s_ =  abs(sin(2.0 * M_PI * static_cast<double>(loop_cnt_) / (period_*4 * RT_TASK_FREQUENCY)));
	
	//P_[0] = Pi_[0] + s_ * (Pf_[0]-Pi_[0]);
	//P_[1] = Pi_[1] + s_ * (Pf_[1]-Pi_[1]);
	
	Eigen::MatrixXd D = T_*(T_.transpose()*T_).inverse() * T_.transpose();
	Eigen::MatrixXd I = MatrixXd::Identity(2,2);
	
	force_des_ = (c_*D*force_ + (1-c_)*(I-D)*force_);

	torque_des_ = J_.transpose() * force_des_;
	
	torque_des_ = torque_des_*1000; // Nm -> mNm
		
	bot_->SetMotorPowerOn();
	if (command_.enable())
	{	
		for(int i=0; i<Ndof_; i++)
		{
			
			bot_->SetStiffness(chain_,i,1.0);
			bot_->SetSlewRateProportional(chain_,i,1.0);
			
			if(i==0) // Upper arm
			{
				bot_->SetModeTorqueGc(chain_,i);
				bot_->SetTorque_mNm(chain_,i,torque_des_(0));
			}
			else if(i==3) // Lower arm
			{
				bot_->SetModeTorqueGc(chain_,i);
				bot_->SetTorque_mNm(chain_,i,torque_des_(1));
			}
			else
			{	
				bot_->SetModeThetaGc(chain_,i);
				bot_->SetThetaDeg(chain_,i,0.0);
			}
		}
	}
	else
	{
		/*bot_->SetModeThetaGc(chain_,3);
		bot_->SetStiffness(chain_,3,1.0);
		bot_->SetSlewRateProportional(chain_,3,1.0);
		bot_->SetThetaDeg(chain_,3,RAD2DEG(1.5));*/
		for(int i=0; i<Ndof_; i++)
		{
			bot_->SetStiffness(chain_,i,1.0);
			bot_->SetSlewRateProportional(chain_,i,1.0);
			
			if(i==0) // Upper arm
			{
				bot_->SetModeTorqueGc(chain_,i);
				//bot_->SetTorque_mNm(chain_,i,torque_des_(0));
			}
			else if(i==3) // Lower arm
			{
				bot_->SetModeTorqueGc(chain_,i);
				//bot_->SetTorque_mNm(chain_,i,torque_des_(1));
			}
			else
			{	
				bot_->SetModeThetaGc(chain_,i);
				bot_->SetThetaDeg(chain_,i,0.0);
			}
		}
	}
	if (loop_cnt_%100==0){
			M3_INFO("** COMMAND **\n");
			M3_INFO("Fx: %f\n",force_des_(0));
			M3_INFO("Fz: %f\n",force_des_(1));
			M3_INFO("T0: %f\n",torque_des_(0)/1000);
			M3_INFO("T3: %f\n",torque_des_(1)/1000);
	}
	loop_cnt_++;
}


}
