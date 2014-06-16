#ifndef SIN_CONTROLLER_H
#define SIN_CONTROLLER_H

////////// M3_CONTROLLER_INTERFACE
#include "m3controllers/m3_controller_interface/m3_controller_interface.h"

namespace m3controllers
{
	
class SinController : public M3Controller
{
	public:
		SinController():M3Controller(){}
	protected:
		bool ReadConfig(const char* filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
	private:
		/** Sin parameters */
		std::vector<mReal> centers_, periods_, magnitudes_;
		
		/** Sin commands */
		joints_t sin_waves_;
};


}

#endif


