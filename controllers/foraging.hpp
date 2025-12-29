#pragma once

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_color_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_system_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/math/rng.h>
#include <optional>


namespace argos {

class ForagingController : public CCI_Controller {
public:

    ForagingController() {}
    virtual ~ForagingController() {}

    void Init(TConfigurationNode& t_tree) override;

    bool hasFood() { return m_carriedFoodId.has_value(); }
    uint32_t getCarriedFoodId() { return m_carriedFoodId.value(); }
    void SetCarriedFoodId(uint32_t foodId) { m_carriedFoodId = foodId; }
    void ClearCarriedFoodId() { m_carriedFoodId = std::nullopt; }
    void addBasePosition(const CVector3& basePos) { m_basePositions.push_back(basePos); } // for use by loop function
    virtual uint8_t getTeamId() const = 0;

protected:
    // Actuators and sensors
    CCI_PiPuckDifferentialDriveActuator* m_pcWheels = nullptr;
    CCI_PiPuckColorLEDsActuator* m_pcColoredLEDs = nullptr;
    CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera = nullptr;
    CCI_PiPuckRangefindersSensor* m_pcRangefinders = nullptr;
    CCI_PiPuckSystemSensor* m_pcSystem = nullptr;
    CCI_PositioningSensor* m_pcPositioning = nullptr;
    // rng
    CRandom::CRNG* m_rng;

    // Foraging variables
    std::optional<uint32_t> m_carriedFoodId = std::nullopt;
    std::vector<CVector3> m_basePositions;
};

} // namespace argos