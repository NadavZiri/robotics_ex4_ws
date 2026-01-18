#include "foraging.hpp"

namespace argos {

   class Controller1 : public ForagingController {

   public:

      Controller1() {}
      virtual ~Controller1() {}

      void Init(TConfigurationNode& t_tree) override;

      void ControlStep() override;

      uint8_t getTeamId() const override { return 1; }
      enum EState
      {
         RANDOM_WALK,
         GO_TO_BASE,
         INTERRUPT_OPPONENT,
         AVOID_OBSTACLE,
         AVOID_FRIENDLY
      };

   private:
      void RandomWalk();
      void GoToBase();
      void InterruptOpponent();
      void AvoidObstacle();
      void AvoidFriendly();

      EState  m_eState;
      CRadians m_cTargetAngle; // the angle we want to reach
      bool m_bIsTurning;    // whether the robot is currently turning or moving forward
      int m_nDriveTimer; // timer for driving forward
      bool m_bIsHoldingFood; // whether the robot is holding food
      Real k_p_follow = 1.0; // proportional gain for interrupting opponent
      Real k_d_follow = 0.1; // derivative gain for interrupting opponent
      Real max_speed = m_pcWheels->MAX_WHEEL_VELOCITY;
      Real m_previous_error_follow = 0.0; // previous error for derivative control
   };
}
