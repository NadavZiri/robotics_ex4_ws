#include "team1.hpp"

namespace argos {

	/****************************************/
	/****************************************/

	void Controller1::Init(TConfigurationNode& t_tree) {
		ForagingController::Init(t_tree);

		m_eState = RANDOM_WALK;
        m_bIsTurning = false;
        m_nDriveTimer = 50;
        srand(m_pcSystem->GetTime() + GetId().at(6));
		ClearCarriedFoodId();
		/* Your Init code goes here */
	}

	void Controller1::ControlStep() {
		switch (m_eState) {
			case RANDOM_WALK:
				RandomWalk();
				break;
			case GO_TO_BASE:
				GoToBase();
				break;
			case INTERRUPT_OPPONENT:
				InterruptOpponent();
				break;
			case AVOID_OBSTACLE:
				AvoidObstacle();
				break;
			case AVOID_FRIENDLY:
				AvoidFriendly();
				break;
		}
		/* Your ControlStep code goes here*/
	}

	void Controller1::RandomWalk() {
		if (hasFood()) { 
			m_eState = GO_TO_BASE;
			return;
		}

		/* Search for food */
		const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
		const CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* pcClosestBlob = nullptr;

		for (size_t i = 0; i < sReadings.BlobList.size(); ++i) {
			if (sReadings.BlobList[i]->Color == CColor::CYAN) {
				pcClosestBlob = sReadings.BlobList[i];
				break; 
			}
		}

		/* Target lock logic (if food is seen) */
		if (pcClosestBlob != nullptr) {
			m_bIsTurning = false; 
			double fAngle = pcClosestBlob->Angle.GetValue();
			if (fAngle > 0.15) m_pcWheels->SetLinearVelocity(0.02, 0.08);
			else if (fAngle < -0.15) m_pcWheels->SetLinearVelocity(0.08, 0.02); 
			else m_pcWheels->SetLinearVelocity(15, 15); 
			return; 
		}

		CRadians cZ, cY, cX;
		m_pcPositioning->GetReading().Orientation.ToEulerAngles(cZ, cY, cX);
		cZ.SignedNormalize();

		if (m_bIsTurning) {
			/* Calculate signed difference to choose turn direction */
			Real fAngleDiff = (m_cTargetAngle - cZ).SignedNormalize().GetValue();

			if (Abs(fAngleDiff) > 0.1) {
				/* Smooth turning WHILE moving forward (Arc) */
				if (fAngleDiff > 0) {
					m_pcWheels->SetLinearVelocity(0.03, 0.09); 
				} else {
					m_pcWheels->SetLinearVelocity(0.09, 0.03); 
				}
			} else {
				m_bIsTurning = false;
				m_nDriveTimer = 10 + (rand() % 100);
			}
		} else {
			if (m_nDriveTimer > 0) {
				m_pcWheels->SetLinearVelocity(15, 15);
				m_nDriveTimer--;
			} else {
				m_bIsTurning = true;
				/* Pick new target angle and LOG IT */
				m_cTargetAngle.SetValue(static_cast<double>(rand()) / RAND_MAX * 2.0 * CRadians::PI.GetValue() - CRadians::PI.GetValue());
				LOG << "[" << GetId() << "] Changing direction while driving!" << std::endl;
			}
		}
	}


	void Controller1::GoToBase() {
		if (!hasFood()) { 
			LOG << "Food dropped at base!" << std::endl;
			m_eState = RANDOM_WALK;
			m_bIsTurning = true;
			m_cTargetAngle.SetValue(static_cast<double>(rand()) / RAND_MAX * 2.0 * CRadians::PI.GetValue());
			return;
		}

		CVector3 cTargetBase = m_basePositions[getTeamId() - 1];
		CVector3 cPos = m_pcPositioning->GetReading().Position;
		CRadians cZ, cY, cX;
		m_pcPositioning->GetReading().Orientation.ToEulerAngles(cZ, cY, cX);
		cZ.UnsignedNormalize();

		Real fAngleToBase = atan2(cTargetBase.GetY() - cPos.GetY(), 
									cTargetBase.GetX() - cPos.GetX());
		CRadians cTargetAngle(fAngleToBase);
		cTargetAngle.UnsignedNormalize();

		Real fDistance = (cTargetBase - cPos).Length();
		Real fAngleDiff = (cTargetAngle - cZ).SignedNormalize().GetValue();

		
		if (Abs(fAngleDiff) > 0.4) { 
			if (fAngleDiff > 0) m_pcWheels->SetLinearVelocity(-8.0, 8.0);
			else m_pcWheels->SetLinearVelocity(8.0, -8.0);
		} 
		else {
			if (fDistance > 0.05) { 
				m_pcWheels->SetLinearVelocity(15.0, 15.0); 
			} else {
				m_pcWheels->SetLinearVelocity(4.0, 4.0);
			}
		}
	}

	void Controller1::InterruptOpponent() {}
	void Controller1::AvoidObstacle() {}
	void Controller1::AvoidFriendly() {}
	/****************************************/
	/****************************************/

	REGISTER_CONTROLLER(Controller1, "controller1");

	
}

