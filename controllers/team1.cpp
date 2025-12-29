#include "team1.hpp"

namespace argos {

	/****************************************/
	/****************************************/

	void Controller1::Init(TConfigurationNode& t_tree) {
		ForagingController::Init(t_tree);
		m_pcColoredLEDs->SetRingLEDs(CColor::BLUE);

		/* Your Init code goes here */
	}

	void Controller1::ControlStep() {
		/* Your ControlStep code goes here*/
	}

	/****************************************/
	/****************************************/

	REGISTER_CONTROLLER(Controller1, "controller1");

}

