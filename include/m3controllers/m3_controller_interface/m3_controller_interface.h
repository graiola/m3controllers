#ifndef M3_CONTROLLER_H
#define M3_CONTROLLER_H

////////// M3
#include <m3/chains/arm.h>
#include <m3/robots/humanoid.h>

////////// M3RT
#include <m3rt/base/component_shm.h>
#include <m3rt/base/m3rt_def.h>
#include <m3rt/base/component_factory.h>

////////// Google protobuff
#include <google/protobuf/message.h>
#include "m3controllers/m3_controller_interface/m3_controller_interface.pb.h"

////////// BOOST
#include <boost/tokenizer.hpp>
//#include <boost/filesystem.hpp>

////////// STD
#include <cmath>

////////// M3_CONTROLLERS
#include "m3controllers/tools/tools.h"

namespace m3controllers
{
	typedef struct
	{
		bool enable;
		
	}M3ControllerSdsCommand;
	
	typedef struct
	{
		bool enable;
		
	}M3ControllerSdsStatus;
	
	typedef Eigen::VectorXd joints_t;
	typedef std::vector<mReal> joints_std_t;
	typedef Eigen::VectorXd cart_t;
	typedef std::vector<mReal> cart_std_t;
	typedef std::vector<int> mask_t;
	
class M3Controller : public m3rt::M3CompShm	
{
	public:
		M3Controller(): m3rt::M3CompShm(MAX_PRIORITY),bot_(NULL),Ndof_(0),Ndof_controlled_(0),loop_cnt_(0),chain_(RIGHT_ARM),joints_mask_cnt_(0),m3_controller_interface_sds_cmd_size_(0),m3_controller_interface_sds_status_size_(0){
			RegisterVersion("default",DEFAULT);
			m3_controller_interface_sds_cmd_size_ = sizeof(M3ControllerSdsCommand);
			m3_controller_interface_sds_status_size_ = sizeof(M3ControllerSdsStatus);
		}
		
		~M3Controller(){
#ifdef USE_ROS_RT_PUBLISHER
			if(ros_nh_ptr_!=NULL) delete ros_nh_ptr_;
#endif
		}
		
		google::protobuf::Message * GetCommand(){return &m3_controller_interface_command_;}
		google::protobuf::Message * GetStatus(){return &m3_controller_interface_status_;}
		google::protobuf::Message * GetParam(){return &m3_controller_interface_param_;}
		
	protected: /// Component Interface
		void Startup()
		{
			if (bot_==NULL)
				SetStateError();
			else
				m3rt::M3CompShm::Startup(); // Allocate sds memory and create the semaphores for external sync
			
			// Set the number of dof
			Ndof_ = bot_->GetNdof(chain_);
			
			// Set the mask and set Ndof_controlled_
			Ndof_controlled_ = SetMask(joints_mask_str_,Ndof_);
			
			// Resize the status vectors
			joints_pos_status_.resize(Ndof_);
			joints_vel_status_.resize(Ndof_);
			joints_acc_status_.resize(Ndof_);
			joints_torques_status_.resize(Ndof_);
			joints_torques_dot_status_.resize(Ndof_);

			// Resize the command vectors
			joints_pos_cmd_.resize(Ndof_);
			joints_vel_cmd_.resize(Ndof_);
			joints_acc_cmd_.resize(Ndof_);
			joints_torques_cmd_.resize(Ndof_);
			// Clear
			joints_pos_status_.fill(0.0);
			joints_vel_status_.fill(0.0);
			joints_acc_status_.fill(0.0);
			joints_torques_status_.fill(0.0);
			joints_torques_dot_status_.fill(0.0);
			
			joints_pos_cmd_.fill(0.0);
			joints_vel_cmd_.fill(0.0);
			joints_acc_cmd_.fill(0.0);
			joints_torques_cmd_.fill(0.0);
			
			file_length_samples_ = static_cast<int>(file_length_sec_*RT_TASK_FREQUENCY);
			if(file_length_samples_ > 0.0){
				file_path_ = static_cast<std::string>(M3_CONTROLLERS_LOG)+GetName();
				file_dumpers_.AddDumper(file_length_samples_,&joints_pos_status_,"joints_pos_status",file_path_);
				file_dumpers_.AddDumper(file_length_samples_,&joints_vel_status_,"joints_vel_status",file_path_);
				file_dumpers_.AddDumper(file_length_samples_,&joints_acc_status_,"joints_acc_status",file_path_);
				file_dumpers_.AddDumper(file_length_samples_,&joints_torques_status_,"joints_torques_status",file_path_);
				file_dumpers_.AddDumper(file_length_samples_,&joints_pos_cmd_,"joints_pos_cmd",file_path_);
				file_dumpers_.AddDumper(file_length_samples_,&joints_torques_cmd_,"joints_torques_cmd",file_path_);
				file_dumpers_.AddDumper(file_length_samples_,&joints_torques_dot_status_,"joints_torques_dot_status",file_path_);
			}
			else
				file_length_samples_ = 0;
			
#ifdef USE_ROS_RT_PUBLISHER	
			// Start the ros node and the publishers
			RosInit();
#endif
		}
		
		void Shutdown()
		{
#ifdef USE_ROS_RT_PUBLISHER	
			RosShutdown();
#endif
			// Motors off
			bot_->SetMotorPowerOff();
			for (int i=0;i<Ndof_;i++)
				bot_->SetModeOff(chain_,i);
			
			m3rt::M3CompShm::Shutdown(); // Free the sds memory and delete the semaphores
			
			file_dumpers_.DumpAll();
		}
		
		//void StepMotorsCommand(Eigen::Ref<Eigen::VectorXd> in)
		template<typename input_vector_t>
		inline void StepMotorsCommand(const input_vector_t& in)
		{
			//M3Controller::StepCommand();
			
			// Motors on
			if (m3_controller_interface_command_.enable()){
				bot_->SetMotorPowerOn();
				joints_mask_cnt_ = 0;
				for (int i=0; i<Ndof_; i++)
				{
					if(joints_mask_[i]){	

						bot_->SetStiffness(chain_,i,1.0);
						bot_->SetSlewRateProportional(chain_,i,1.0);
						if(controller_type_=="pos"){
							bot_->SetModeThetaGc(chain_,i);
							bot_->SetThetaDeg(chain_,i,RAD2DEG(in[joints_mask_cnt_]));
							joints_pos_cmd_[joints_mask_cnt_] = in[joints_mask_cnt_]; 
						}
						else if(controller_type_=="torques"){
							bot_->SetModeTorqueGc(chain_,i);
							bot_->SetTorque_mNm(chain_,i,m2mm(in[joints_mask_cnt_]));
							joints_torques_cmd_[joints_mask_cnt_] = in[joints_mask_cnt_];
						}
						joints_mask_cnt_++;
					}
				}
			}
		}
		
		void StepCommand()
		{	
			m3rt::M3CompShm::StepCommand();   // Take cmd from sds
		}
		
		void StepStatus()
		{	
			m3rt::M3CompShm::StepStatus();   // Update status in the sds
		}
		
		inline void StepMotorsStatus()
		{	
			//m3rt::M3CompShm::StepStatus();  // Update status in the sds
			
			for(int i=0;i<Ndof_;i++){
				joints_pos_status_[i] = DEG2RAD(bot_->GetThetaDeg(chain_,i));
				joints_vel_status_[i] = DEG2RAD(bot_->GetThetaDotDeg(chain_,i));		
				joints_acc_status_[i] = DEG2RAD(bot_->GetThetaDotDotDeg(chain_,i));
				joints_torques_status_[i] = mm2m(bot_->GetTorque_mNm(chain_,i));// mNm -> Nm
				joints_torques_dot_status_[i] = mm2m(bot_->GetTorqueDot_mNm(chain_,i));// mNm -> Nm
			}
			
			if(m3_controller_interface_command_.enable())
			{
				file_dumpers_.SaveAll();
			}
					
#ifdef USE_ROS_RT_PUBLISHER
			if(ros_nh_ptr_!=NULL){
				rt_publishers_.PublishAll();
			}
#endif
			// Increase the loop_cnt_
			loop_cnt_++;
		}
		
		bool ReadConfig(const char* cfg_filename)
		{
			//YAML::Node doc;
			if (!M3CompShm::ReadConfig(cfg_filename)) 
				return false;
			//m3rt::GetYamlDoc(cfg_filename, doc);
			doc["humanoid"] >> bot_name_;
			doc["chain"] >> chain_name_;
			if(chain_name_== "RIGHT_ARM")
				chain_ = RIGHT_ARM;

			else if(chain_name_=="LEFT_ARM")
				chain_ = LEFT_ARM;
			else
			{
				m3rt::M3_ERR("Wrong chain name %s for component %s\n",chain_name_.c_str(),GetName().c_str());
				return false;
			}
			doc["joints_mask"] >> joints_mask_str_;
			doc["controller_type"] >> controller_type_;
			if(controller_type_=="torques" ||  controller_type_=="pos")
				m3rt::M3_INFO("Selected controller %s for component %s\n",controller_type_.c_str(),GetName().c_str());
			else
			{
				m3rt::M3_INFO("Wrong controller type %s for component %s\n",controller_type_.c_str(),GetName().c_str());
				return false;
			}
			
			try
			{
				doc["sec_to_dump"] >> file_length_sec_;
				
			} 
			catch(YAML::TypedKeyNotFound<std::string> e)
			{	
				file_length_sec_ = 0;
			}
			
			return true;
		}
		
		bool LinkDependentComponents()
		{
			//Need to find at least one arm
			bot_=(m3::M3Humanoid*) factory->GetComponent(bot_name_);
			if (bot_==NULL)
				m3rt::M3_INFO("M3Humanoid component %s not found for component %s\n",bot_name_.c_str(),GetName().c_str());
			if (bot_==NULL)
				return false;
			return true;
		}
		
		//void SetMask(std::string& mask_str,const int& Ndof, int& Ndof_controlled)
		int SetMask(std::string& mask_str,const int Ndof)
		{
			int n_tokens = 0;
			int Ndof_controlled = 0;
			// Initialize mask
			joints_mask_.resize(Ndof);
			fill (joints_mask_.begin(),joints_mask_.end(),0);
			boost::char_separator<char> sep(",");
			boost::tokenizer<boost::char_separator<char> > tokens(mask_str, sep);
			for (const auto& t : tokens)
			{
				if(t == "1" || t == "0")
					n_tokens++;
			}
			// First check the mask_str size
			if(n_tokens!=Ndof){
				m3rt::M3_INFO("Invalid mask format");
				m3rt::M3_INFO("It will be used the default mask (1,1...,1)\n");
				for (size_t i = 0; i<joints_mask_.size(); i++) 
					joints_mask_[i] = 1;
				Ndof_controlled = Ndof;
			}
			else
			{
				n_tokens = 0;
				for (const auto& t : tokens){
					if(t == "1"){
						joints_mask_[n_tokens] = 1;
						Ndof_controlled++;
					}
					else
						joints_mask_[n_tokens] = 0;
					n_tokens++;
				}	
			}
			return Ndof_controlled;
		}
		
	protected:
		joints_t joints_pos_status_;
		joints_t joints_vel_status_;
		joints_t joints_acc_status_;
		joints_t joints_torques_status_;
		joints_t joints_torques_dot_status_;
		joints_t joints_pos_cmd_;
		joints_t joints_vel_cmd_;
		joints_t joints_acc_cmd_;
		joints_t joints_torques_cmd_;

#ifdef USE_ROS_RT_PUBLISHER
		ros::NodeHandle* ros_nh_ptr_;
		tools::RealTimePublishers<tools::RealTimePublisherJoints> rt_publishers_;
		
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
				rt_publishers_.AddPublisher(*ros_nh_ptr_,"joints_status_pos",Ndof_,&joints_pos_status_);
				rt_publishers_.AddPublisher(*ros_nh_ptr_,"joints_status_vel",Ndof_,&joints_vel_status_);
				rt_publishers_.AddPublisher(*ros_nh_ptr_,"joints_status_acc",Ndof_,&joints_acc_status_);
				rt_publishers_.AddPublisher(*ros_nh_ptr_,"joints_status_torques",Ndof_,&joints_torques_status_);
				rt_publishers_.AddPublisher(*ros_nh_ptr_,"joints_status_torques_dot",Ndof_,&joints_torques_dot_status_);
				rt_publishers_.AddPublisher(*ros_nh_ptr_,"joints_cmd_pos",Ndof_,&joints_pos_cmd_);
				rt_publishers_.AddPublisher(*ros_nh_ptr_,"joints_cmd_torques",Ndof_,&joints_torques_cmd_);
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
	protected:
			
		size_t GetStatusSdsSize()
		{return m3_controller_interface_sds_status_size_;}
		size_t GetCommandSdsSize()
		{return m3_controller_interface_sds_cmd_size_;}
		void SetCommandFromSds(unsigned char* data)
		{
			M3ControllerSdsCommand* sds = (M3ControllerSdsCommand*) data;
			
			// Lock the semaphore and copy the input data
			request_command();
			memcpy(&m3_controller_interface_cmd_from_sds_, sds, m3_controller_interface_sds_cmd_size_);
			release_command();
			
			// Convert from sds to protobuf message
			m3_controller_interface_command_.set_enable(m3_controller_interface_cmd_from_sds_.enable);
		}
		void SetSdsFromStatus(unsigned char* data)
		{
			// Convert from protobuf message to sds
			
			//TODO empty
			
			M3ControllerSdsStatus* sds = (M3ControllerSdsStatus*) data;
			
			// Lock the semaphore and copy the output data
			request_status();
			memcpy(sds, &m3_controller_interface_status_to_sds_, m3_controller_interface_sds_status_size_);
			release_status();	
		}
		
		M3BaseStatus* GetBaseStatus(){return m3_controller_interface_status_.mutable_base();}

		/** Google protobuf messages */
		M3ControllerStatus m3_controller_interface_status_;
		M3ControllerCommand m3_controller_interface_command_;
		M3ControllerParam m3_controller_interface_param_;
		
		enum {DEFAULT};
		std::string bot_name_, chain_name_, joints_mask_str_, controller_type_, file_path_;
		m3::M3Humanoid* bot_;
		/** Number of degree of freedom */
		int Ndof_;
		/** Number of controlled degree of freedom */
		int Ndof_controlled_;
		/** Loops counter*/
		long long loop_cnt_;
		/** Controlled chain */
		M3Chain chain_;
		/** Mask to select the controlled joints */
		mask_t joints_mask_;
		/** Counter used with the mask */
		int joints_mask_cnt_;
		
		/** Sds structures */
		M3ControllerSdsCommand m3_controller_interface_cmd_from_sds_;
		M3ControllerSdsStatus m3_controller_interface_status_to_sds_;
		size_t m3_controller_interface_sds_cmd_size_;
		size_t m3_controller_interface_sds_status_size_;
		
		/** Used by the dumping */
		double file_length_sec_;
		int file_length_samples_;
		
		tools::FileDumpers file_dumpers_;
};


}

#endif
