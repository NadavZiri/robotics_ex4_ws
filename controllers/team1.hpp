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
         COLLECT_FOOD,
         INTERRUPT_OPPONENT,
         AVOID_OBSTACLE,
         AVOID_FRIENDLY
      };

   private:
      void RandomWalk();
      void GoToBase();
      void CollectFood();
      void InterruptOpponent();
      void AvoidObstacle();
      void AvoidFriendly();
      Real getSensorProximity(UInt8 sensorLabel);
      CRandom::CRNG *m_pcRNG;

      EState  m_eState;
      CRadians m_cTargetAngle; // the angle we want to reach
      bool m_bIsTurning;    // whether the robot is currently turning or moving forward
      int m_nDriveTimer; // timer for driving forward
      bool m_bIsHoldingFood; // whether the robot is holding food
      UInt32 nRandomInt;
      Real k_p_follow = 1.0; // proportional gain for interrupting opponent
      Real k_d_follow = 0.1; // derivative gain for interrupting opponent
      Real max_speed = m_pcWheels->MAX_WHEEL_VELOCITY;
      Real turn_duration = 10.0; // duration to turn when avoiding obstacle
      Real turn_start_time = 0;
      Real m_previous_error_follow = 0.0; // previous error for derivative control
      Real m_previous_error_collect = 0.0; // previous error for derivative control in CollectFood
      Real m_previous_distance_to_food = 0.0; // previous distance to food for derivative control in InterruptOpponent
      Real m_time_interrupt_start = 0.0; // time when interrupting opponent started
   };
}
