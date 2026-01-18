#include "team1.hpp"

namespace argos
{

	/****************************************/
	/****************************************/

	void Controller1::Init(TConfigurationNode &t_tree)
	{
		ForagingController::Init(t_tree);

		m_eState = RANDOM_WALK;
		m_bIsTurning = false;
		m_nDriveTimer = 50;
		srand(m_pcSystem->GetTime() + GetId().at(6));
		ClearCarriedFoodId();
		/* Your Init code goes here */
	}

	void Controller1::ControlStep()
	{
		switch (m_eState)
		{
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

	void Controller1::RandomWalk()
	{
		if (hasFood())
		{
			m_eState = GO_TO_BASE;
			return;
		}

		const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings &sReadings = m_pcCamera->GetReadings();
		const CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob *pcClosestBlob = nullptr;

		for (size_t i = 0; i < sReadings.BlobList.size(); ++i)
		{
			if (sReadings.BlobList[i]->Color == CColor::CYAN)
			{
				pcClosestBlob = sReadings.BlobList[i];
				break;
			}
			if (sReadings.BlobList[i]->Color == CColor::RED)
			{
				m_eState = INTERRUPT_OPPONENT;
				return;
			}
		}

		if (pcClosestBlob != nullptr)
		{
			if (pcClosestBlob->Angle.GetValue() > 0.1)
			{
				m_pcWheels->SetLinearVelocity(2.0, 10.0);
			}
			else if (pcClosestBlob->Angle.GetValue() < -0.1)
			{
				m_pcWheels->SetLinearVelocity(10.0, 2.0);
			}
			else
			{
				m_pcWheels->SetLinearVelocity(10.0, 10.0);
			}
			return;
		}

		CRadians cZ, cY, cX;
		m_pcPositioning->GetReading().Orientation.ToEulerAngles(cZ, cY, cX);
		cZ.UnsignedNormalize();

		if (m_bIsTurning)
		{
			Real fAngleDiff = Abs((cZ - m_cTargetAngle).GetValue());

			if (fAngleDiff > 0.05)
			{
				m_pcWheels->SetLinearVelocity(-5.0, 5.0);
			}
			else
			{
				m_bIsTurning = false;
				m_nDriveTimer = 20 + (rand() % 40);
			}
		}
		else
		{
			if (m_nDriveTimer > 0)
			{
				m_pcWheels->SetLinearVelocity(10.0, 10.0);
				m_nDriveTimer--;
			}
			else
			{
				m_bIsTurning = true;
				m_cTargetAngle.SetValue(static_cast<double>(rand()) / RAND_MAX * 2.0 * CRadians::PI.GetValue());
			}
		}
	}

	void Controller1::GoToBase()
	{
		if (!hasFood())
		{
			LOG << "Food dropped at base!" << std::endl;
			m_eState = RANDOM_WALK;
			m_bIsTurning = true;
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

		if (fDistance > 0.15)
		{
			Real fAngleDiff = (cTargetAngle - cZ).SignedNormalize().GetValue();

			if (Abs(fAngleDiff) > 0.2)
			{
				if (fAngleDiff > 0)
					m_pcWheels->SetLinearVelocity(-5.0, 5.0);
				else
					m_pcWheels->SetLinearVelocity(5.0, -5.0);
			}
			else
			{
				m_pcWheels->SetLinearVelocity(10.0, 10.0);
			}
		}
		else
		{
			m_pcWheels->SetLinearVelocity(0.0, 0.0);
			LOG << "At base" << std::endl;
		}
	}
	void Controller1::InterruptOpponent()
	{
		if (hasFood())
		{
			m_eState = GO_TO_BASE;
			return;
		}
		auto *blobList = &m_pcCamera->GetReadings().BlobList;
		if (blobList->empty())
		{
			m_eState = RANDOM_WALK;
			return;
		}
		CRadians cZ, cY, cX;
		m_pcPositioning->GetReading().Orientation.ToEulerAngles(cZ, cY, cX);
		cZ.UnsignedNormalize();
		CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob *target_blob = nullptr;
		Real angle_weight = 1.2;
		Real best_score = std::numeric_limits<Real>::max();
		for (auto &blob : *blobList)
		{
			if (blob->Color == CColor::RED)
			{
				Real distance = blob->Distance;
				CRadians angle = blob->Angle;
				Real blob_score = distance + angle_weight * Abs(angle.SignedNormalize().GetValue());
				if (blob_score < best_score)
				{
					best_score = blob_score;
					target_blob = blob;
				}
			}
		}
		if (target_blob == nullptr) {
			m_pcWheels->SetLinearVelocity(max_speed, max_speed);
			m_eState = RANDOM_WALK;
			m_previous_error_follow = 0.0;
			return;
		}
		Real distance = target_blob->Distance;
		CRadians angle = target_blob->Angle;
		Real target_distance = 0.05;
		Real current_angle_error = target_blob->Angle.SignedNormalize().GetValue();
		Real error_derivative = current_angle_error - m_previous_error_follow;
		Real turn_speed = (k_p_follow * current_angle_error) + (k_d_follow * error_derivative);
		m_previous_error_follow = current_angle_error;
		Real k_linear = 5.0;
		Real forward_speed = k_linear * (distance - target_distance);
		forward_speed = std::clamp(forward_speed, 0.0, max_speed);
		Real left_wheel_speed = forward_speed - turn_speed;
		Real right_wheel_speed = forward_speed + turn_speed;
		left_wheel_speed = std::clamp(left_wheel_speed, -max_speed, max_speed);
		right_wheel_speed = std::clamp(right_wheel_speed, -max_speed, max_speed);
		m_pcWheels->SetLinearVelocity(left_wheel_speed, right_wheel_speed);
	}
	void Controller1::AvoidObstacle() {}
	void Controller1::AvoidFriendly() {}
	/****************************************/
	/****************************************/

	REGISTER_CONTROLLER(Controller1, "controller1");

}
