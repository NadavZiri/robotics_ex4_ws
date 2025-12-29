#include "foraging.hpp"

namespace argos {

   class Controller1 : public ForagingController {

   public:

      Controller1() {}
      virtual ~Controller1() {}

      void Init(TConfigurationNode& t_tree) override;

      void ControlStep() override;

      uint8_t getTeamId() const override { return 1; }
   };
}
