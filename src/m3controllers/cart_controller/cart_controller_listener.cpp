////////// M3_CONTROLLER_INTERFACE
#include "m3controllers/cart_controller/cart_controller.h"

////////// RT DEFS
#define NANO2SEC(a)	a/1e9
#define SEC2NANO(a)	a*1e9
#define CNT_SHM "TSHMM"

////////// STD
#include <stdio.h>
#include <signal.h>

////////// ROS
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

sig_atomic_t stop = 0;
void shutdown(int dummy) {stop = 1;}

using namespace m3controllers;
using namespace std;

static SEM* command_sem;
static M3Sds* m3_sds;
CartControllerSdsCommand cmd;
static string controller_acronym = "CRT";

bool getSemAddr(const char* sem_name,SEM* &sem){
	sem = (SEM*)rt_get_adr(nam2num(sem_name));
	if (!sem)
		return false;
	return true;
}

bool getShmAddr(const char* shm_name,M3Sds* &sds){
	if ((sds = (M3Sds*)rt_shm_alloc(nam2num(shm_name),sizeof(M3Sds),USE_VMALLOC)))
		return true;
	else
		return false;
}

class ShmMonitor
{
	public:
		ShmMonitor(){}
		
		bool init()
		{
			// Create the RT task
			rt_allow_nonroot_hrt();
			RT_TASK* task;
			//Args: Name, Priority, Stack Size, max_msg_size, Policy, cpus_allowed
			if (!(task = rt_task_init_schmod(nam2num( "M3CRT" ),0,0,0,SCHED_FIFO,0xF)))
			{
				m3rt::M3_INFO("ERROR: Cannot initialize task manager\n");
				return false;
			}
			
			string command_sem_name = "M3"+controller_acronym+"C";
			string shared_mem_name = "M3"+controller_acronym+"M";
			if(getSemAddr(command_sem_name.c_str(),command_sem))
				m3rt::M3_INFO("Command semaphore taken\n");
			else
			{
				m3rt::M3_INFO("Unable to find semphore %s\n",command_sem_name.c_str());
				rt_task_delete(task);
				return false;
			}
			
			if(getShmAddr(shared_mem_name.c_str(),m3_sds))
				m3rt::M3_INFO("Shared memory taken\n");
			else
			{
				m3rt::M3_INFO("Unable to find shared memory %s\n",shared_mem_name.c_str());
				rt_task_delete(task);
				return false;
			}
		}
		void startController()
		{
			m3rt::M3_INFO("Start the controller\n");
			cmd.enable = true;
			/*cmd.x = 0.0;
			cmd.y = 0.0;
			cmd.z = 0.0;
			cmd.roll = 0.0;
			cmd.pitch = 0.0;
			cmd.yaw = 0.0;*/
			writeCommands();
		}
		void stopController()
		{
			m3rt::M3_INFO("Stop the controller\n");
			cmd.enable = false;
			writeCommands();
		}
		
		void commandController(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw)
		{
			cmd.x = x;
			cmd.y = y;
			cmd.z = z;
			cmd.roll = roll;
			cmd.pitch = pitch;
			cmd.yaw = yaw;
			writeCommands();
		}
		
	protected:
		void writeCommands()
		{
			rt_sem_wait(command_sem); //FIXME this can lock if the server is killed before the manager task
			memcpy(m3_sds->cmd, &cmd, sizeof(CartControllerSdsCommand));
			rt_sem_signal(command_sem);
		}
};

static ShmMonitor mnt;	// FIXME

void commandCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  ROS_INFO("x: [%f]", msg->transform.translation.x);
  ROS_INFO("y: [%f]", msg->transform.translation.y);
  ROS_INFO("z: [%f]", msg->transform.translation.z);
  
  const double q0 = msg->transform.rotation.w;
  const double q1 = msg->transform.rotation.x;
  const double q2 = msg->transform.rotation.y;
  const double q3 = msg->transform.rotation.z;
  
  mnt.commandController(msg->transform.translation.x,
			 msg->transform.translation.y,
			 msg->transform.translation.z,
			 atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)),
			 asin(2*(q0*q2-q3*q1)),
			 atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
			);
}

int main(int argc, char *argv[])
{
	// Handle ctrl-c
	//signal(SIGINT, shutdown);

	ros::init(argc, argv, "cart_controller_listener");
	ros::NodeHandle nodeh;
	
	std::string topic_name = "/hand_pose_publisher/left_hand_ref_pose"; //FIXME
	
	ros::Subscriber sub = nodeh.subscribe(topic_name, 1000, commandCallback);
	
	if(mnt.init())
	{
		mnt.startController();
		//while (ros::ok()){usleep(200);}// Wait for the kill signal
			//usleep(200);
			
		ros::spin();
			
		if(!ros::ok())
			mnt.stopController();
	}

	return 0;
}
