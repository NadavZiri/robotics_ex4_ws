#pragma once

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/plugins/simulator/entities/led_entity.h>

#include "foraging_loop_functions.hpp"
#include <foraging.hpp>

namespace argos {

class CQTFunctions : public CQTOpenGLUserFunctions {

public:

   CQTFunctions();      
   virtual ~CQTFunctions() {}

   void Draw(CPiPuckEntity& c_entity);
   void Draw(CLEDEntity& c_entity);
   virtual void DrawInWorld();

   constexpr static Real FOOD_HEIGHT = 0.02f; // height of food for rendering
   constexpr static Real FOOD_RADIUS = 0.02f; // radius of food for rendering

private:
    CForagingLoopFunctions& m_cForagingLF;
};

} // namespace argos