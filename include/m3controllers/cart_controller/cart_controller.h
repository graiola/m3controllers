#ifndef CART_CONTROLLER_H
#define CART_CONTROLLER_H

////////// M3_CONTROLLER_INTERFACE
#include "m3controllers/m3_controller_interface/m3_controller_interface.h"

////////// Google protobuff
#include "m3controllers/cart_controller/cart_controller.pb.h"

////////// Eigen
#include <eigen3/Eigen/Core>

////////// KDL_KINEMATICS
#include <kdl_kinematics/kdl_kinematics.h>

namespace m3controllers
{
	typedef struct
	{
		double x;
		double y;
		double z;
		double roll;
		double pitch;
		double yaw;
		bool enable;
		
	}CartControllerSdsCommand;
	
	typedef struct
	{
		double x;
		double y;
		double z;
		double roll;
		double pitch;
		double yaw;
		
	}CartControllerSdsStatus;
	
class AdaptiveGain
{
	public:
		
		AdaptiveGain(std::vector<mReal> gains_params)
		{
			assert(gains_params.size()==3);
			AdaptiveGain(gains_params[0],gains_params[1],gains_params[2]);
		}
		
		AdaptiveGain(mReal gain_at_zero, mReal gain_at_inf=0.0, mReal zero_slope_error_value=0.0)
		{
			if(gain_at_inf==0.0) // Constant gain
				b_ = 0.0;
			else // Adaptive gain
			{	
				// Checks
				assert(gain_at_inf > 0.0);
				assert(gain_at_zero > gain_at_inf);
				assert(zero_slope_error_value > 0.0);
				b_ = 6/zero_slope_error_value;
			}
			c_ = gain_at_inf;
			a_ = gain_at_zero - gain_at_inf;			
		}
		
		mReal ComputeGain(mReal error)
		{
			return a_ * std::exp(-b_ * std::abs(error)) + c_;
		}
		
	private:
		mReal a_;
		mReal b_;
		mReal c_;
};
	
class CartController : public M3Controller
{
	public:
		CartController():M3Controller(),component_communication_(false),samples_to_execute_(0),samples_executed_(0),cart_controller_sds_cmd_size_(0){
			cart_controller_sds_cmd_size_ = sizeof(CartControllerSdsCommand);
			cart_controller_sds_status_size_ = sizeof(CartControllerSdsStatus);
		}
		~CartController(){if(kin_!=NULL) delete kin_;};
		
		/* Get and sets for the pose, used externaly */
		void SetCommand(const Eigen::Ref<const Eigen::VectorXd>& pose_in)
		{
			assert(pose_in.size() == 6);
			cart_controller_command_.set_x(pose_in[0]);
			cart_controller_command_.set_y(pose_in[1]);
			cart_controller_command_.set_z(pose_in[2]);
			cart_controller_command_.set_roll(pose_in[3]);
			cart_controller_command_.set_pitch(pose_in[4]);
			cart_controller_command_.set_yaw(pose_in[5]);
		}
		void GetStatus(Eigen::Ref<Eigen::VectorXd> pose_out)
		{
			assert(pose_out.size() == 6);
			pose_out[0] = cart_controller_status_.x();
			pose_out[1] = cart_controller_status_.y();
			pose_out[2] = cart_controller_status_.z();
			pose_out[3] = cart_controller_status_.roll();
			pose_out[4] = cart_controller_status_.pitch();
			pose_out[5] = cart_controller_status_.yaw();	
		}
		
		int GetCartSize()
		{	
			return cart_size_;
		}
		
		void EnableController() //FIXME I don't like it
		{
			m3_controller_interface_command_.set_enable(true);
		}
		
		void DisableController() //FIXME I don't like it
		{
			m3_controller_interface_command_.set_enable(false);
		}
		
		void DisableInternalCommunication() // The communication is done through the shared memory. No internal communication with other components.
		{
			component_communication_ = false;
		}
		
		void EnableInternalCommunication() // Skip the communication with the external world. It allows other components to comunicate with it.
		{
			component_communication_ = true;
		}
		
	protected:
		bool ReadConfig(const char* filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		
		size_t GetStatusSdsSize()
		{return cart_controller_sds_status_size_;}
		size_t GetCommandSdsSize()
		{return cart_controller_sds_cmd_size_;}
		
		void SetCommandFromSds(unsigned char* data)
		{
			if(!component_communication_){
				CartControllerCommand* sds = (CartControllerCommand*) data;
				
				// Lock the semaphore and copy the output data
				request_command();
				memcpy(&cart_controller_cmd_from_sds_, sds, cart_controller_sds_cmd_size_);
				release_command();
				
				//M3Controller::SetCommandFromSds(data);
				
				// Convert from sds to protobuf message
				cart_controller_command_.set_x(cart_controller_cmd_from_sds_.x);
				cart_controller_command_.set_y(cart_controller_cmd_from_sds_.y);
				cart_controller_command_.set_z(cart_controller_cmd_from_sds_.z);
				cart_controller_command_.set_roll(cart_controller_cmd_from_sds_.roll);
				cart_controller_command_.set_pitch(cart_controller_cmd_from_sds_.pitch);
				cart_controller_command_.set_yaw(cart_controller_cmd_from_sds_.yaw);
				m3_controller_interface_command_.set_enable(cart_controller_cmd_from_sds_.enable); //HACK: Right now I am not able to include m3_controller_interface.proto inside cart_controller.proto due problem with the include command in protobuf
			}
		}
		
		void SetSdsFromStatus(unsigned char* data)
		{
			if(!component_communication_){
				// Convert from protobuf message to sds
				cart_controller_cmd_from_sds_.x = cart_controller_status_.x();
				cart_controller_cmd_from_sds_.y = cart_controller_status_.y();
				cart_controller_cmd_from_sds_.z = cart_controller_status_.z();
				cart_controller_cmd_from_sds_.roll = cart_controller_status_.roll();
				cart_controller_cmd_from_sds_.pitch = cart_controller_status_.pitch();
				cart_controller_cmd_from_sds_.yaw = cart_controller_status_.yaw();
				
				CartControllerSdsStatus* sds = (CartControllerSdsStatus*) data;
				
				// Lock the semaphore and copy the output data
				request_status();
				memcpy(sds, &cart_controller_status_from_sds_, cart_controller_sds_status_size_);
				release_status();
			}
		}
		
	private:
		
		/* Get and sets for the pose, used internaly */
		void SetStatus(const Eigen::Ref<const Eigen::VectorXd>& pose_in)
		{
			assert(pose_in.size() == 6);
			cart_controller_status_.set_x(pose_in[0]);
			cart_controller_status_.set_y(pose_in[1]);
			cart_controller_status_.set_z(pose_in[2]);
			cart_controller_status_.set_roll(pose_in[3]);
			cart_controller_status_.set_pitch(pose_in[4]);
			cart_controller_status_.set_yaw(pose_in[5]);
		}
		void GetCommand(Eigen::Ref<Eigen::VectorXd> pose_out)
		{
			assert(pose_out.size() == 6);
			pose_out[0] = cart_controller_command_.x();
			pose_out[1] = cart_controller_command_.y();
			pose_out[2] = cart_controller_command_.z();
			pose_out[3] = cart_controller_command_.roll();
			pose_out[4] = cart_controller_command_.pitch();
			pose_out[5] = cart_controller_command_.yaw();
		}
		
		bool component_communication_;
		
		enum ref_t {CONST_REF,SDS_REF,TRJ_REF};
		ref_t reference_source_;
		std::string cmd_input_file_name_;
		std::vector<cart_std_t> cart_cmd_mat_;
		
		int samples_to_execute_, samples_executed_;
		
		cart_t cart_pos_status_;
		cart_t cart_pos_cmd_;
		cart_t cart_vel_status_;
		
		bool enable_sds_;
		cart_std_t ef_pose_;
		
		int cart_size_;
		std::string cart_mask_str_;
		kdl_kinematics::KDLClik* kin_;
		
		/** Google protobuf messages */
		CartControllerStatus cart_controller_status_;
		CartControllerCommand cart_controller_command_;
		CartControllerParam cart_controller_param_;
		
		/** Sds structures */
		CartControllerSdsCommand cart_controller_cmd_from_sds_;
		CartControllerSdsStatus cart_controller_status_from_sds_;
		size_t cart_controller_sds_cmd_size_, cart_controller_sds_status_size_;
};


}

#endif


