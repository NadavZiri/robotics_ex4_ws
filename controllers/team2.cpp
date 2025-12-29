#include "team2.hpp"

namespace argos {

    /****************************************/
    /****************************************/
    
    void Controller2::Init(TConfigurationNode& t_tree) {
        ForagingController::Init(t_tree);
        m_pcColoredLEDs->SetRingLEDs(CColor::RED);
        
        /* Your Init code goes here */
    }

    void Controller2::ControlStep() {
        /* Your ControlStep code goes here */

    }

    /****************************************/
    /****************************************/

    REGISTER_CONTROLLER(Controller2, "controller2");
}
