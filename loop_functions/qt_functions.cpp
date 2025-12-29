#include "qt_functions.hpp"

namespace argos {

/****************************************/
/****************************************/

CQTFunctions::CQTFunctions() :
    m_cForagingLF(dynamic_cast<CForagingLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {
        RegisterUserFunction<CQTFunctions,CPiPuckEntity>(&CQTFunctions::Draw);
        RegisterUserFunction<CQTFunctions,CLEDEntity>(&CQTFunctions::Draw);
}

/****************************************/
/****************************************/

void CQTFunctions::Draw(CPiPuckEntity& c_entity) {
    ForagingController& controller = dynamic_cast<ForagingController&>(c_entity.GetControllableEntity().GetController());
    
    // Draw() is invoked in world coordinates for composable entities; use the embodied origin.
    const CVector3& cRobotPos = c_entity.GetEmbodiedEntity().GetOriginAnchor().Position;

   if(controller.hasFood()) {
      FoodItem& foodItem = m_cForagingLF.getFoodItem(controller.getCarriedFoodId());
      if(&foodItem) {
          // Render the carried food as a cylinder above the robot.
          DrawCylinder(
              cRobotPos + CVector3(0.0f, 0.0f, 0.1f), // position slightly above robot
              CQuaternion(),
              FOOD_RADIUS,
              FOOD_HEIGHT,
              CColor::GRAY80
          );
      } else {
          THROW_ARGOSEXCEPTION("Robot " << controller.GetId() << " thinks it has food, but no food entity is mapped to it in loop functions.");
      }
   }
}

void CQTFunctions::Draw(CLEDEntity& c_entity) {
    FoodItem& cFoodItem = m_cForagingLF.getFoodItem(c_entity.GetIndex());
    // only draw uncarried food items (on the ground), as carried food items are drawn by the pipuck Draw()
    if (cFoodItem.carrier_id.has_value()) {
        return;
    }

    // Draw food items as cylinders on the ground
    CVector3 cFoodPos = c_entity.GetPosition();
    DrawCylinder(
        CVector3(cFoodPos.GetX(), cFoodPos.GetY(), FOOD_HEIGHT / 2), // position on ground
        CQuaternion(),
        FOOD_RADIUS,
        FOOD_HEIGHT,
        CColor::GRAY80
    );
}

void CQTFunctions::DrawInWorld() {
    // draw base positions
    for (const CVector3& basePos : m_cForagingLF.getBases()) {
        DrawCircle(
            CVector3(basePos.GetX(), basePos.GetY(), 0.001f), // slightly above ground
            CQuaternion(),
            m_cForagingLF.BASE_RADIUS,
            CColor::PURPLE
        );
        DrawText(
            basePos + CVector3(0.0f, 0.0f, 0.05f), // slightly above base
            "Base",
            CColor::PURPLE
        );
    }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CQTFunctions, "qt_functions")

} // namespace argos