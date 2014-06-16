////////// M3_CONTROLLER_INTERFACE
#include "m3controllers/m3_controller_interface/m3_controller_interface.h"

////////// RT DEFS
#define NANO2SEC(a)	a/1e9
#define SEC2NANO(a)	a*1e9
#define CNT_SHM "TSHMM"

////////// STD
#include <stdio.h>
#include <signal.h>

sig_atomic_t stop = 0;
void shutdown(int dummy) {stop = 1;}

using namespace m3controllers;
using namespace std;

static SEM* status_sem;
static SEM* command_sem;
static M3Sds* m3_sds;
M3ControllerSdsCommand cmd;
M3ControllerSdsStatus status;

static int controller_id;
static string controller_acronym;

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
			if (!(task = rt_task_init_schmod(nam2num( "M3MNGR" ),0,0,0,SCHED_FIFO,0xF)))
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
			writeCommands();
		}
		void stopController()
		{
			m3rt::M3_INFO("Stop the controller\n");
			cmd.enable = false;
			writeCommands();
		}
	protected:
		void writeCommands()
		{
			rt_sem_wait(command_sem); //FIXME this can lock if the server is killed before the manager task
			memcpy(m3_sds->cmd, &cmd, sizeof(M3ControllerSdsCommand));
			rt_sem_signal(command_sem);
		}
};

bool interface()
{
	cout<<"Select the controller: "<<endl;
	cout<<"1): sin_controller"<<endl;
	cout<<"2): torque_controller"<<endl;
	cout<<"3): cart_controller"<<endl;
	cout<<"4): joints_controller"<<endl;
	cout<<"selection: ";
	cin>>controller_id;
	if(controller_id == 1)
	{
		controller_acronym = "SIN";
		return true;
	}
	else if(controller_id == 2)
	{
		controller_acronym = "TRQ";
		return true;
	}
	else if(controller_id == 3)
	{
		controller_acronym = "CRT";
		return true;
	}
	else if(controller_id == 4)
	{
		controller_acronym = "JNT";
		return true;
	}
	else
	{
		m3rt::M3_INFO("ERROR: Wrong selection %d\n",controller_id);
		return false;
	}
}

int main(int argc, char *argv[])
{
	// Handle ctrl-c
	signal(SIGINT, shutdown);
		
	ShmMonitor mnt;	
		
	if(interface())
	{
		if(mnt.init())
		{
			mnt.startController();
			while (!stop) // Wait for the kill signal
				usleep(200);
			mnt.stopController();
		}
	}

	return 0;
}
