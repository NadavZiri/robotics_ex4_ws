#include "foraging_loop_functions.hpp"


namespace argos {
	
	void CForagingLoopFunctions::Init(TConfigurationNode& t_tree) {
		TConfigurationNode& paramsNode = GetNode(t_tree, "params");
		
		// initialize random number generator
		m_pcRNG = CRandom::CreateRNG("argos");
		
		// read arena size from configuration (for entity distribution)
		CVector3 arenaSize = GetSpace().GetArenaSize();
		Real halfX = arenaSize.GetX() / 2.0;
		Real halfY = arenaSize.GetY() / 2.0;

		// parse & add bases
		GetNodeAttribute(paramsNode, "base1", m_Bases1.emplace_back());
		GetNodeAttribute(paramsNode, "base2", m_Bases2.emplace_back());
		
		// distribute food items in the environment
		GetNodeAttribute(paramsNode, "num_food_items", m_uNumFoodItems);
		for (uint32_t i = 0; i < m_uNumFoodItems; ++i) {
			// sample a random position within the arena bounds
			CVector3 position(
				m_pcRNG->Uniform(CRange<Real>(-halfX, halfX)),
				m_pcRNG->Uniform(CRange<Real>(-halfY, halfY)),
				0
			);
			// create a new LED entity to represent a food item
			CLEDEntity* ledEntity = new CLEDEntity(
				nullptr,
				"food_item_" + std::to_string(i),
				position,
				CColor::GRAY80
			);
			AddEntity(*ledEntity);
			CLEDMedium& ledMedium = CSimulator::GetInstance().GetMedium<CLEDMedium>("leds");
			ledEntity->SetMedium(ledMedium);
			ledEntity->Enable();
			m_FoodItems[ledEntity->GetIndex()] = FoodItem(ledEntity->GetIndex(), std::nullopt, ledEntity);
		}
	}
	
	void CForagingLoopFunctions::PreStep() {
		CSpace::TMapPerType& pipucks = GetSpace().GetEntitiesByType("pipuck");

		// lazy initialization to add base positions to controllers (so pipuck entities are already created when we do this)
		if (!m_bLazyInitialized) {
			for (auto& pipuck_pair : pipucks) {
				// cast the entity to CPiPuckEntity & the controller to ForagingController
				CPiPuckEntity* pipuckEntity = any_cast<CPiPuckEntity*>(pipuck_pair.second);
				ForagingController& pipuckController = dynamic_cast<ForagingController&>(pipuckEntity->GetControllableEntity().GetController());
				// add base positions to the controller
				const std::vector<CVector3>& m_Bases = (pipuckController.getTeamId() == 1) ? m_Bases1 : m_Bases2;
				for (const CVector3& basePos : m_Bases) {
					pipuckController.addBasePosition(basePos);
				}
			}
			m_bLazyInitialized = true;
		}

		for (auto& pipuck_pair : pipucks) {
			// cast the entity to CPiPuckEntity & the controller to ForagingController
			CPiPuckEntity* pipuckEntity = any_cast<CPiPuckEntity*>(pipuck_pair.second);
			ForagingController& pipuckController = dynamic_cast<ForagingController&>(pipuckEntity->GetControllableEntity().GetController());
			
			// get the current position of the pipuck
			CVector3 pipuckPosition = pipuckEntity->GetEmbodiedEntity().GetOriginAnchor().Position;
			
			// check if the pipuck is carrying food & handle logic accordingly
			if (pipuckController.hasFood()) {
				uint32_t carriedFoodId = pipuckController.getCarriedFoodId();
				FoodItem& foodItem = getFoodItem(carriedFoodId);
				// update food item's position to match pipuck's position
				foodItem.ledEntity->SetPosition(pipuckPosition);
				foodItem.carrier_id = pipuckEntity->GetIndex();
				// check if the pipuck is within a base area to drop off the food
				const std::vector<CVector3>& m_Bases = (pipuckController.getTeamId() == 1) ? m_Bases1 : m_Bases2;
				for (const CVector3& basePos : m_Bases) {
					Real distanceSqrd = (pipuckPosition - basePos).SquareLength();
					if (distanceSqrd <= BASE_RADIUS * BASE_RADIUS) {
						// pipuck is within base area, drop off the food
						// reset the food item carrier
						foodItem.carrier_id = std::nullopt;
						// respawn the food item at a new random position
						CVector3 arenaSize = GetSpace().GetArenaSize();
						Real halfX = arenaSize.GetX() / 2.0;
						Real halfY = arenaSize.GetY() / 2.0;
						CVector3 newPosition(
							m_pcRNG->Uniform(CRange<Real>(-halfX + 0.1, halfX - 0.1)),
							m_pcRNG->Uniform(CRange<Real>(-halfY + 0.1, halfY - 0.1)),
							0
						);
						foodItem.ledEntity->SetPosition(newPosition);
						// inform the controller that the food has been dropped
						pipuckController.ClearCarriedFoodId();
						break; // exit loop after dropping off
					}
				}
			} else { // not carrying food
				// check if the pipuck is close to a food item
				for (auto& food_pair : m_FoodItems) {
					FoodItem& foodItem = food_pair.second;
					if (foodItem.carrier_id.has_value()) continue; // skip if food already carried
					CVector3 foodPosition = foodItem.ledEntity->GetPosition();
					Real distanceSqrd = (pipuckPosition - foodPosition).SquareLength();
					// check if within pickup radius
					if (distanceSqrd <= PICKUP_RADIUS_SQRD) {
						// pipuck can pick up this food item
						foodItem.carrier_id = pipuckEntity->GetIndex();
						// inform the controller about the picked food item
						pipuckController.SetCarriedFoodId(foodItem.food_id);
						break; // exit loop after picking up one item
					}
				}
			}
		}
	}
	
	FoodItem& CForagingLoopFunctions::getFoodItem(uint32_t id) {
		return m_FoodItems[id];
	}
	
	REGISTER_LOOP_FUNCTIONS(CForagingLoopFunctions, "foraging_loop_functions");
	
}