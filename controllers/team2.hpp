#include "foraging.hpp"

namespace argos {

   class Controller2 : public ForagingController {

   public:

      Controller2() {}
      virtual ~Controller2() {}

      void Init(TConfigurationNode& t_tree) override;

      void ControlStep() override;

      uint8_t getTeamId() const override { return 2; }
   };
}
