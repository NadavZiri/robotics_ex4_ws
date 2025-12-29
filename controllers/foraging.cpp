#include "foraging.hpp"

namespace argos {
    
    void ForagingController::Init(TConfigurationNode& t_tree) {
        /* Get the actuators and sensors */
        m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
        m_pcColoredLEDs = GetActuator<CCI_PiPuckColorLEDsActuator>("pipuck_leds");
        m_pcSystem = GetSensor<CCI_PiPuckSystemSensor>("pipuck_system");
        m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
        m_pcCamera->Enable();
        m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
        m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");

        // initialize rng
        m_rng = CRandom::CreateRNG("argos");
    }
    
}