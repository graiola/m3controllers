#ifndef JOINTS_CONTROLLER_H
#define JOINTS_CONTROLLER_H

////////// M3_CONTROLLER_INTERFACE
#include "m3controllers/m3_controller_interface/m3_controller_interface.h"

namespace m3controllers
{
	
class JointsController : public M3Controller
{
	public:
		JointsController():M3Controller(),samples_to_execute_(0),samples_executed_(0){}
	protected:
		bool ReadConfig(const char* filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
	private:
		enum ref_t {CONST_REF,SDS_REF,TRJ_REF};
		ref_t reference_source_;
		
		joints_std_t joints_position_;
		int samples_to_execute_, samples_executed_;
		std::string cmd_input_file_name_;
		std::vector<joints_std_t> joints_cmd_mat_;
};

}

#endif


