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
		case COLLECT_FOOD:
			CollectFood();
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
		/* Search for food */
		const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings &sReadings = m_pcCamera->GetReadings();
		const CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob *pcClosestBlob = nullptr;
		const CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob *pcBlueBlob = nullptr;
		LOG << "Robot number: " << GetId() << "\n" << "Sensor 0: " << getSensorProximity(0) << ", Sensor 7: " << getSensorProximity(7) << std::endl;
		for (size_t i = 0; i < sReadings.BlobList.size(); ++i)
		{
			if (sReadings.BlobList[i]->Color == CColor::GRAY80)
			{
				pcClosestBlob = sReadings.BlobList[i];
				if (Abs(pcClosestBlob->Angle.GetValue()) < CRadians::PI_OVER_FOUR.GetValue())
				{
					m_eState = COLLECT_FOOD;
					return;
				}
				break;
			}
			if (sReadings.BlobList[i]->Color == CColor::RED)
			{
				m_time_interrupt_start = m_pcSystem->GetTime();
				m_eState = INTERRUPT_OPPONENT;
				return;
			}
		}

		/* Target lock logic (if food is seen) */
		if (pcClosestBlob != nullptr)
		{
			m_bIsTurning = false;
			double fAngle = pcClosestBlob->Angle.GetValue();
			if (fAngle > 0.15)
				m_pcWheels->SetLinearVelocity(0.02, 0.08);
			else if (fAngle < -0.15)
				m_pcWheels->SetLinearVelocity(0.08, 0.02);
			else
				m_pcWheels->SetLinearVelocity(15, 15);
			return;
		}

		CRadians cZ, cY, cX;
		m_pcPositioning->GetReading().Orientation.ToEulerAngles(cZ, cY, cX);
		cZ.SignedNormalize();

		if (m_bIsTurning)
		{
			/* Calculate signed difference to choose turn direction */
			Real fAngleDiff = (m_cTargetAngle - cZ).SignedNormalize().GetValue();

			if (Abs(fAngleDiff) > 0.1)
			{
				/* Smooth turning WHILE moving forward (Arc) */
				if (fAngleDiff > 0)
				{
					m_pcWheels->SetLinearVelocity(0.03, 0.09);
				}
				else
				{
					m_pcWheels->SetLinearVelocity(0.09, 0.03);
				}
			}
			else
			{
				m_bIsTurning = false;
				m_nDriveTimer = 10 + (rand() % 100);
			}
		}
		else
		{
			if (m_nDriveTimer > 0)
			{
				m_pcWheels->SetLinearVelocity(15, 15);
				m_nDriveTimer--;
			}
			else
			{
				m_bIsTurning = true;
				/* Pick new target angle and LOG IT */
				m_cTargetAngle.SetValue(static_cast<double>(rand()) / RAND_MAX * 2.0 * CRadians::PI.GetValue() - CRadians::PI.GetValue());
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

		if (Abs(fAngleDiff) > 0.4)
		{
			if (fAngleDiff > 0)
				m_pcWheels->SetLinearVelocity(-8.0, 8.0);
			else
				m_pcWheels->SetLinearVelocity(8.0, -8.0);
		}
		else
		{
			if (fDistance > 0.05)
			{
				m_pcWheels->SetLinearVelocity(15.0, 15.0);
			}
			else
			{
				m_pcWheels->SetLinearVelocity(4.0, 4.0);
			}
		}
	}
	void Controller1::InterruptOpponent()
	{
		if (hasFood())
		{
			m_eState = GO_TO_BASE;
			return;
		}
		if (m_pcSystem->GetTime() - m_time_interrupt_start > 8.0)
		{
			m_eState = RANDOM_WALK;
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
			if (blob->Color == CColor::GRAY80)
			{
				if (Abs(blob->Angle.SignedNormalize().GetValue()) < CRadians::PI_OVER_FOUR.GetValue())
				{
					m_eState = COLLECT_FOOD;
					return;
				}
			}
		}
		if (target_blob == nullptr)
		{
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
	void Controller1::AvoidObstacle()
	{
		if (hasFood())
		{
			m_eState = GO_TO_BASE;
			return;
		}
		LOG << "Avoiding obstacle!" << "Team " << (int)getTeamId() << std::endl;
		LOG << "Sensor 0: " << getSensorProximity(0) << ", Sensor 7: " << getSensorProximity(7) << std::endl;
		if (getSensorProximity(0) < 0.1 || getSensorProximity(7) < 0.1)
		{
			if (getSensorProximity(0) < getSensorProximity(7)) {
				m_pcWheels->SetLinearVelocity(0.0, max_speed);
			}
			else {
				m_pcWheels->SetLinearVelocity(max_speed, 0.0);
			}
		}
		else
		{
			m_eState = RANDOM_WALK;
		}
	}
	void Controller1::AvoidFriendly() {
		LOG << "Avoiding friendly!" << std::endl;
		auto *blobList = &m_pcCamera->GetReadings().BlobList;
		CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob *target_blob = nullptr;
		if (blobList->empty())
		{
			m_eState = RANDOM_WALK;
			return;
		}
		for (auto &blob : *blobList) {
			if (blob->Color == CColor::BLUE && blob->Distance < 0.15) {
				CRadians angle = blob->Angle.SignedNormalize();
				if (angle.GetValue() > 0 && angle.GetValue() < CRadians::PI_OVER_FOUR.GetValue()) {
					m_pcWheels->SetLinearVelocity(max_speed, 0.0);
					return;
				}
				else if (angle.GetValue() < 0 && angle.GetValue() > -CRadians::PI_OVER_FOUR.GetValue()) {
					m_pcWheels->SetLinearVelocity(0.0, max_speed);
					return;
				}
			}
		}
		m_eState = RANDOM_WALK;
	}

	void Controller1::CollectFood()
	{
		if (hasFood())
		{
			m_eState = GO_TO_BASE;
			return;
		}
		if (getSensorProximity(0) < 0.1 || getSensorProximity(7) < 0.1)
		{
			LOG << "Obstacle detected during collection, switching to AVOID_OBSTACLE state." << std::endl;
			m_eState = AVOID_OBSTACLE;
			return;
		}
		auto *blobList = &m_pcCamera->GetReadings().BlobList;
		if (blobList->empty())
		{
			m_eState = RANDOM_WALK;
			return;
		}
		CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob *target_blob = nullptr;
		Real best_distance = std::numeric_limits<Real>::max();
		for (auto &blob : *blobList) {
			if (blob->Color == CColor::BLUE && blob->Distance < 0.15
			&& Abs(blob->Angle.SignedNormalize().GetValue()) < CRadians::PI_OVER_FOUR.GetValue())
			{
				LOG << "Friendly detected during collection, switching to AVOID_FRIENDLY state." << std::endl;
				m_eState = AVOID_FRIENDLY;
				return;
			}
		}
		for (auto &blob : *blobList) {
			if (blob->Color == CColor::GRAY80 && Abs(blob->Angle.SignedNormalize().GetValue()) < CRadians::PI_OVER_FOUR.GetValue()) {
				if (blob->Distance < best_distance) {
					best_distance = blob->Distance;
					target_blob = blob;
				}
			}
		}
		if (target_blob == nullptr)
		{
			m_eState = RANDOM_WALK;
			return;
		}
		Real distance = target_blob->Distance;
		CRadians angle = target_blob->Angle;
		Real target_distance = 0.01;
		Real current_angle_error = target_blob->Angle.SignedNormalize().GetValue();
		Real error_derivative = current_angle_error - m_previous_error_collect;
		Real turn_speed = (k_p_follow * current_angle_error) + (k_d_follow * error_derivative);
		m_previous_error_collect = current_angle_error;
		Real k_linear = 5.0;
		Real forward_speed = k_linear * (distance - target_distance);
		forward_speed = std::clamp(forward_speed, 0.0, max_speed);
		Real left_wheel_speed = forward_speed - turn_speed;
		Real right_wheel_speed = forward_speed + turn_speed;
		left_wheel_speed = std::clamp(left_wheel_speed, -max_speed, max_speed);
		right_wheel_speed = std::clamp(right_wheel_speed, -max_speed, max_speed);
		m_pcWheels->SetLinearVelocity(left_wheel_speed, right_wheel_speed);
	}

	Real Controller1::getSensorProximity(UInt8 sensorLabel)
	{
		Real proximity = 0.0;
		m_pcRangefinders->Visit([&proximity, sensorLabel](const auto &sensor)
								{
            if (sensor.Label == sensorLabel) {
                proximity = sensor.Proximity;
            } });
		return proximity;
	}
	/****************************************/
	/****************************************/

	REGISTER_CONTROLLER(Controller1, "controller1");

}
