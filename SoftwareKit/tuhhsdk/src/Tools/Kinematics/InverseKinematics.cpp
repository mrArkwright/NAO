#include "InverseKinematics.h"
#include "Tools/Storage/Blackboard.h"
#include "Definitions/robotConstants.h"



vector<float> InverseKinematics::getLLegAngles(const KinematicMatrix& desired)
{
	// given is the desired position and orientation of the foot
	// but we need the desired position and rotation of the ankle
	// first transform to ankle space and shift about FOOT_HEIGTH 
	// to get the desired ankle
	KinematicMatrix ankleInv =	KinematicMatrix::transZ(-LINKS::FOOT_HEIGHT) *
								desired.invert();

	// now transform back to torso space
	KinematicMatrix ankleDesired = ankleInv.invert();	
	
	// transformation of the desired position to the Hip Space
	KinematicMatrix ankle2hip			=	KinematicMatrix::transY(-LINKS::HIP_OFFSET_Y) *
											KinematicMatrix::transZ(LINKS::HIP_OFFSET_Z) *
											ankleDesired;

	// Transformation to the rotated Hip Space
	KinematicMatrix ankle2hipOrthogonal =	KinematicMatrix::rotX( -45.0f * TO_RAD) *
											ankle2hip;

	// calculate the the distance from hip to ankle
	float l = ankle2hipOrthogonal.posV.abs();
	// normal vektor to ankle
	Vector3<float> n = ankle2hipOrthogonal.posV / l;

	// check wether the position is reachable
	float a_KneePitch;
	
	if (l > LINKS::LEG_MAX_LENGTH )
	{
		ankle2hipOrthogonal.posV = n * LINKS::LEG_MAX_LENGTH;
		l = LINKS::LEG_MAX_LENGTH;
		a_KneePitch = 0.0f;
	}
	else if ( l < LINKS::LEG_MIN_LENGTH )
	{
		ankle2hipOrthogonal.posV = n * LINKS::LEG_MIN_LENGTH;
		l = LINKS::LEG_MIN_LENGTH;
		a_KneePitch = MAX_RANGE[JOINTS::L_KNEE_PITCH];
	}
	else
	{
		// calculate the knee angle from thigh length, tibia length and hip-ankle distance
		a_KneePitch = (float) M_PI -	acos(	(	pow(LINKS::THIGH_LENGTH,2) +
														pow(LINKS::TIBIA_LENGTH,2) -
														pow(l,2) 
													) / 
													(	2 * LINKS::THIGH_LENGTH *
															LINKS::TIBIA_LENGTH
													) 
												);
	}
	
	// inverse needed
	KinematicMatrix hipOrthogonal2ankle = ankle2hipOrthogonal.invert();

	// calculate angle for ankle pitch
	float a_AnklePitch_1 =	acos(	(	pow(LINKS::TIBIA_LENGTH,2)	+
										pow(l,2)				- 
										pow(LINKS::THIGH_LENGTH,2)
									) / 
									(	2 * LINKS::TIBIA_LENGTH	*
										l
									) 
								);

	Vector3<float> v_hipAnkle = hipOrthogonal2ankle.posV;
	float a_AnklePitch2	=	atan2(	v_hipAnkle.x	, 
									sqrt(	pow(v_hipAnkle.y,2)	 + 
											pow(v_hipAnkle.z,2) 
										) 
								);

	float a_AnklePitch = - ( a_AnklePitch_1 + a_AnklePitch2);

	// calculate angle for ankle roll
	float a_AnkleRoll = atan2( v_hipAnkle.y , v_hipAnkle.z );


	// transform the desired position from ankle space to hip
	KinematicMatrix thigh2Foot	=	KinematicMatrix::rotX( -a_AnkleRoll )			*
									KinematicMatrix::rotY (-a_AnklePitch)			*
									KinematicMatrix::transZ(LINKS::TIBIA_LENGTH)*
									KinematicMatrix::rotY(-a_KneePitch)				*
									KinematicMatrix::transZ(LINKS::THIGH_LENGTH);

	// get the transformation to Hip Othorgonal
	KinematicMatrix hipOthorgonal2thigh = ankle2hipOrthogonal * thigh2Foot;

	// get angles from the transformation matrix
	float alphaX = asin(hipOthorgonal2thigh.rotM.c[1].z);
	float a_HipYawPitch	=	-atan2( -hipOthorgonal2thigh.rotM.c[1].x, hipOthorgonal2thigh.rotM.c[1].y );
	float a_HipPitch	=	 atan2( -hipOthorgonal2thigh.rotM.c[0].z, hipOthorgonal2thigh.rotM.c[2].z );
	float a_HipRoll		=	 (float) (alphaX + M_PI / 4);

	// constraints on angles

	//ankle Pitch
	if ( a_AnklePitch < MIN_RANGE[JOINTS::L_ANKLE_PITCH] )
		a_AnklePitch = MIN_RANGE[JOINTS::L_ANKLE_PITCH];
	else if (a_AnklePitch > MAX_RANGE[JOINTS::L_ANKLE_PITCH])
		a_AnklePitch = MAX_RANGE[JOINTS::L_ANKLE_PITCH];

	// ankleRoll
	if (a_AnkleRoll < MIN_RANGE_L_ANKLE_ROLL(a_AnklePitch) )
		a_AnkleRoll = MIN_RANGE_L_ANKLE_ROLL(a_AnklePitch);
	else if (a_AnkleRoll > MAX_RANGE_L_ANKLE_ROLL(a_AnklePitch) )
		a_AnkleRoll = MAX_RANGE_L_ANKLE_ROLL(a_AnklePitch);

	// hipYaw
	if ( a_HipYawPitch < MIN_RANGE[JOINTS::L_HIP_YAW_PITCH] )
		a_HipYawPitch = MIN_RANGE[JOINTS::L_HIP_YAW_PITCH];
	else if (a_HipYawPitch > MAX_RANGE[JOINTS::L_HIP_YAW_PITCH])
		a_HipYawPitch = MAX_RANGE[JOINTS::L_HIP_YAW_PITCH];

	// hipPitch
	if ( a_HipPitch < MIN_RANGE[JOINTS::L_HIP_PITCH] )
		a_HipPitch = MIN_RANGE[JOINTS::L_HIP_PITCH];
	else if (a_HipPitch > MAX_RANGE[JOINTS::L_HIP_PITCH])
		a_HipPitch = MAX_RANGE[JOINTS::L_HIP_PITCH];

	// hipRoll
	if ( a_HipRoll < MIN_RANGE[JOINTS::L_HIP_ROLL])
		a_HipRoll = MIN_RANGE[JOINTS::L_HIP_ROLL];
	else if (a_HipRoll > MAX_RANGE[JOINTS::L_HIP_ROLL])
		a_HipRoll = MAX_RANGE[JOINTS::L_HIP_ROLL];
	
	// create angle vector
	vector<float> leg;
	leg.push_back(a_HipYawPitch);
	leg.push_back(a_HipRoll);
	leg.push_back(a_HipPitch);
	leg.push_back(a_KneePitch);
	leg.push_back(a_AnklePitch);
	leg.push_back(a_AnkleRoll);

	return leg;
}



vector<float> InverseKinematics::getRLegAngles(const KinematicMatrix& desired)
{
	
	// given is the desired position and orientation of the foot
	// but we need the desired position and rotation of the ankle
	// first transform to ankle space and shift about FOOT_HEIGTH 
	// to get the desired ankle
	KinematicMatrix ankleInv =	KinematicMatrix::transZ(-LINKS::FOOT_HEIGHT) *
								desired.invert();

	// transform back to torso space
	KinematicMatrix ankleDesired = ankleInv.invert();
	
	// transformation of the desired position to the Hip Space
	KinematicMatrix ankle2hip			=	KinematicMatrix::transY(LINKS::HIP_OFFSET_Y) *
											KinematicMatrix::transZ(LINKS::HIP_OFFSET_Z) *
											ankleDesired;

	// Transformation to the rotated Hip Space
	KinematicMatrix ankle2hipOrthogonal =	KinematicMatrix::rotX( 45.0f * TO_RAD) *
											ankle2hip;

	// calculate the the distance from hip to ankle
	float l = ankle2hipOrthogonal.posV.abs();

	// normal vector to ankle
	Vector3<float> n = ankle2hipOrthogonal.posV / l;

	// check wether the position is reachable
	float a_KneePitch;
	
	if (l > LINKS::LEG_MAX_LENGTH )
	{
		ankle2hipOrthogonal.posV = n * LINKS::LEG_MAX_LENGTH;
		l = LINKS::LEG_MAX_LENGTH;
		a_KneePitch = 0.0f;
	}
	else if (l < LINKS::LEG_MIN_LENGTH)
	{
		ankle2hipOrthogonal.posV = n * LINKS::LEG_MIN_LENGTH;
		l = LINKS::LEG_MIN_LENGTH;
		a_KneePitch = MAX_RANGE[JOINTS::R_KNEE_PITCH];
	}
	else
	{
		// calculate the knee angle from thigh length, tibia length and hip-ankle distance
		a_KneePitch = (float) M_PI -	acos(	(	pow(LINKS::THIGH_LENGTH,2) +
													pow(LINKS::TIBIA_LENGTH,2) -
													pow(l,2) 
												) / 
												(	2 * LINKS::THIGH_LENGTH *
														LINKS::TIBIA_LENGTH
												) 
											  );		
	}
	
	// inverse needed
	KinematicMatrix hipOrthogonal2ankle = ankle2hipOrthogonal.invert();

	// calculate angle for ankle pitch
	float a_AnklePitch_1 =	acos(	(	pow(LINKS::TIBIA_LENGTH,2)	+
										pow(l,2)						- 
										pow(LINKS::THIGH_LENGTH,2)
									) / 
									(	2 * LINKS::TIBIA_LENGTH	*
										l
									) 
								);

	Vector3<float> v_hipAnkle = hipOrthogonal2ankle.posV;
	float a_AnklePitch2	=	atan2(	v_hipAnkle.x	, 
									sqrt(	pow(v_hipAnkle.y,2)	 + 
											pow(v_hipAnkle.z,2) 
										) 
								);

	float a_AnklePitch = - ( a_AnklePitch_1 + a_AnklePitch2);

	// calculate angle for ankle roll
	float a_AnkleRoll = atan2( v_hipAnkle.y , v_hipAnkle.z );


	// transform the desired position from ankle space to hip
	KinematicMatrix thigh2Foot	=	KinematicMatrix::rotX( -a_AnkleRoll )			*
									KinematicMatrix::rotY (-a_AnklePitch)			*
									KinematicMatrix::transZ(LINKS::TIBIA_LENGTH)*
									KinematicMatrix::rotY(-a_KneePitch)				*
									KinematicMatrix::transZ(LINKS::THIGH_LENGTH);

	// get the transformation to Hip Othorgonal
	KinematicMatrix hipOthorgonal2thigh = ankle2hipOrthogonal * thigh2Foot;

	// get angles from the transformation matrix
	float alphaX = asin(hipOthorgonal2thigh.rotM.c[1].z);
	float a_HipYawPitch	= atan2( -hipOthorgonal2thigh.rotM.c[1].x, hipOthorgonal2thigh.rotM.c[1].y ) ;
	float a_HipPitch	= atan2( -hipOthorgonal2thigh.rotM.c[0].z, hipOthorgonal2thigh.rotM.c[2].z ) ;
	float a_HipRoll		= alphaX - (float) M_PI / 4;

	// constraints on angles

	//ankle Pitch
	if ( a_AnklePitch < MIN_RANGE[JOINTS::R_ANKLE_PITCH] )
		a_AnklePitch = MIN_RANGE[JOINTS::R_ANKLE_PITCH];
	else if (a_AnklePitch > MAX_RANGE[JOINTS::R_ANKLE_PITCH])
		a_AnklePitch = MAX_RANGE[JOINTS::R_ANKLE_PITCH];

	// ankleRoll
	if (a_AnkleRoll < MIN_RANGE_R_ANKLE_ROLL(a_AnklePitch) )
		a_AnkleRoll = MIN_RANGE_R_ANKLE_ROLL(a_AnklePitch);
	else if (a_AnkleRoll > MAX_RANGE_R_ANKLE_ROLL(a_AnklePitch) )
		a_AnkleRoll = MAX_RANGE_R_ANKLE_ROLL(a_AnklePitch);

	// hipYaw
	if ( a_HipYawPitch < MIN_RANGE[JOINTS::R_HIP_YAW_PITCH])
		a_HipYawPitch = MIN_RANGE[JOINTS::R_HIP_YAW_PITCH];
	else if (a_HipYawPitch > MAX_RANGE[JOINTS::R_HIP_YAW_PITCH])
		a_HipYawPitch = MAX_RANGE[JOINTS::R_HIP_YAW_PITCH];

	// hipPitch
	if ( a_HipPitch < MIN_RANGE[JOINTS::R_HIP_PITCH])
		a_HipPitch = MIN_RANGE[JOINTS::R_HIP_PITCH];
	else if (a_HipPitch > MAX_RANGE[JOINTS::R_HIP_PITCH])
		a_HipPitch = MAX_RANGE[JOINTS::R_HIP_PITCH];

	// hipRoll
	if ( a_HipRoll < MIN_RANGE[JOINTS::R_HIP_ROLL])
		a_HipRoll = MIN_RANGE[JOINTS::R_HIP_ROLL];
	else if (a_HipRoll > MAX_RANGE[JOINTS::R_HIP_ROLL])
		a_HipRoll = MAX_RANGE[JOINTS::R_HIP_ROLL];
	
	// create angle vector
	vector<float> leg;
	leg.push_back(a_HipYawPitch);
	leg.push_back(a_HipRoll);
	leg.push_back(a_HipPitch);
	leg.push_back(a_KneePitch);
	leg.push_back(a_AnklePitch);
	leg.push_back(a_AnkleRoll);

	return leg;
}


vector<float> InverseKinematics::getLArmAngles(const KinematicMatrix& desired)
{
	
	// Transformation of the desired hand position to shoulder space
	KinematicMatrix Hand2Shoulder = KinematicMatrix::transZ(-LINKS::SHOULDER_OFFSET_Z) *
									KinematicMatrix::transY(-LINKS::SHOULDER_OFFSET_Y) *
									desired;

	// distance from shoulder to desired hand position
	float l = Hand2Shoulder.posV.abs();

	// normalized Vector from shoulder to desired hand position 
	Vector3<float> n = Hand2Shoulder.posV / l;

	// declar ElbowRoll
	float a_ElbowRoll;

	// check, if the desired position is reachable
	if ( l > LINKS::ARM_MAX_LENGTH )
	{
		Hand2Shoulder.posV = n * LINKS::ARM_MAX_LENGTH;
		l = LINKS::ARM_MAX_LENGTH;
		a_ElbowRoll = MAX_RANGE[JOINTS::L_ELBOW_ROLL];
	}
	else if ( l < LINKS::ARM_MIN_LENGTH )
	{
		Hand2Shoulder.posV = n * LINKS::ARM_MIN_LENGTH;
		l = LINKS::ARM_MIN_LENGTH;
		a_ElbowRoll = MIN_RANGE[JOINTS::L_ELBOW_ROLL];
	}
	else
	{
		// rule of cosines
		a_ElbowRoll = acos(	(	pow(LINKS::UPPER_ARM_LENGTH,2) +
								pow(LINKS::FOREARM_LENGTH,2) -
								pow(l,2) 
							)
							/
							(	2 * LINKS::UPPER_ARM_LENGTH *
								LINKS::FOREARM_LENGTH
							)
							) - (float) M_PI;
	}

	// calculation of the circles radius on which the elbow can be positioned
	float beta = acos(	(	pow(l,2) +
							pow(LINKS::UPPER_ARM_LENGTH,2) -
							pow(LINKS::FOREARM_LENGTH,2)
						)
						/
						(	2 * l * LINKS::UPPER_ARM_LENGTH )
					  );

	float r = sin(beta) * LINKS::UPPER_ARM_LENGTH;

	// distance from shoulder to circle midpoint
	float d = cos(beta) * LINKS::UPPER_ARM_LENGTH;

	// Elbow position from desired hand position and orientation
	KinematicMatrix Shoulder2Elbow = KinematicMatrix::transX (LINKS::FOREARM_LENGTH) *
									 Hand2Shoulder.invert();

	KinematicMatrix Elbow2Shoulder = Shoulder2Elbow.invert();

	// distance from desired elbow position to circle surface
	float s = n * Elbow2Shoulder.posV - d;

	// projection of desired elbow position on circle surface
	Vector3<float> p = Elbow2Shoulder.posV - n * s;

	// circle midpoint
	Vector3<float> m = n * d;

	// Vector from m to p
	Vector3<float> vecMP = p - m;
	vecMP.normalize();

	// calculate reachable elbow position
	Vector3<float> pReachable = m + vecMP * r;
	Vector3<float> pDesired = pReachable;

	/* calculation of rotation angles, such that the shoulder coordinate-system
	 * can be transformed to the circle surface. 
	 * The y- and z- axes are in the surface, the x-axis is the normal vector
	 */
	float a1 = atan2(m.y,m.x);
	float a2 = atan2(m.z, sqrtf( pow(m.x,2) + pow(m.z,2) ) );

	//Transformation matrix to circle space
	KinematicMatrix ToCirc = KinematicMatrix::rotZ(a1) *
							 KinematicMatrix::rotY(-a2);

	Vector3<float> pToCirc = ToCirc.invert() * pReachable;

	float a3 = atan2(-pToCirc.y, pToCirc.z);

	// orthogonal circle vectors
	Vector3<float> u = ToCirc * KinematicMatrix::rotX(a3) * Vector3<float>(0,r,0);
	Vector3<float> v = ToCirc * KinematicMatrix::rotX(a3) * Vector3<float>(0,0,r);

	// set step size for iteration
	int circleParts = 60;
	float step = 2 * (float) M_PI / circleParts;

	// constant for shoulder roll limits
	float k = cos(MAX_RANGE[JOINTS::L_SHOULDER_PITCH]);

	// iteration variables 
	float t = 0;
	float bestDis = std::numeric_limits<float>::infinity();
	float bestT = t;
	bool noAvailableCirclePoint = true;
	bool optimumFound = false;

	float a_ShoulderRoll;
	float a_ShoulderPitch;
	float a_ElbowYaw;
	float a_WristYaw;
	KinematicMatrix Hand2Elbow;
	KinematicMatrix Hand2HandBase;

	// iterate on circle
	for (int i = 1; i <= circleParts; i++)
	{

		// check if desired p is reachable
		if (	pReachable.y <= LINKS::L_ELBOW_MAX_Y &&
				pReachable.y >= LINKS::L_ELBOW_MIN_Y &&
				pReachable.x >= getPitchlimit(pReachable.y,k) )
		{
			noAvailableCirclePoint = false;

			a_ShoulderRoll = asin ( pReachable.y / LINKS::UPPER_ARM_LENGTH );

			a_ShoulderPitch = atan2 ( -pReachable.z, pReachable.x );

			Hand2Elbow =	KinematicMatrix::transX(-LINKS::UPPER_ARM_LENGTH) *
							KinematicMatrix::rotZ(-a_ShoulderRoll) *
							KinematicMatrix::rotY(-a_ShoulderPitch) *
							Hand2Shoulder;
			
			a_ElbowYaw =	atan2(-Hand2Elbow.posV.z, -Hand2Elbow.posV.y );

			// if ElbowYaw is in range, then optimum is found
			if ( a_ElbowYaw <= MAX_RANGE[JOINTS::L_ELBOW_YAW] &&
				 a_ElbowYaw >= MIN_RANGE[JOINTS::L_ELBOW_YAW])
			{
				optimumFound = true;
				break;
			}
			else
			{
				// store distance to desired hand position
				// set ElbowYaw in Range
				if ( a_ElbowYaw > MAX_RANGE[JOINTS::L_ELBOW_YAW])
					a_ElbowYaw = MAX_RANGE[JOINTS::L_ELBOW_YAW];
				else
					a_ElbowYaw = MIN_RANGE[JOINTS::L_ELBOW_YAW];

				// transform to handbase space
				Hand2HandBase = KinematicMatrix::transX(-LINKS::FOREARM_LENGTH) *
								KinematicMatrix::rotZ(-a_ElbowRoll) * 
								KinematicMatrix::rotX(-a_ElbowYaw) *
								Hand2Elbow;

				float dis = Hand2HandBase.posV.abs();
				
				// check if distance to desired hand is better than the best found solution yet
				if ( dis < bestDis )
				{
					bestT = t;
					bestDis = dis;
				}
			}
		}
		
		// step
		t = t + i*step;
		// alternate
		step = -step;
		pReachable = m + u * sin(t) + v * cos(t);
	}


	// if no optimum could be found
	if (!optimumFound)
	{
		// if there was a possible elbow position on the circle
		if (!noAvailableCirclePoint)
			// take best t
			pReachable = m + u * sin(bestT) + v * cos(bestT);
		else
			// take the desired elbow position ( not on circle)
			pReachable = pDesired;

		a_ShoulderRoll = asin ( pReachable.y / LINKS::UPPER_ARM_LENGTH);

		a_ShoulderPitch = atan2 (-pReachable.z, pReachable.x);

		// check if the angles are in range
		if ( a_ShoulderRoll > MAX_RANGE[JOINTS::L_SHOULDER_ROLL])
			a_ShoulderRoll = MAX_RANGE[JOINTS::L_SHOULDER_ROLL];
		else if ( a_ShoulderRoll < MIN_RANGE[JOINTS::L_SHOULDER_ROLL])
			a_ShoulderRoll = MIN_RANGE[JOINTS::L_SHOULDER_ROLL];

		if ( a_ShoulderPitch > MAX_RANGE[JOINTS::L_SHOULDER_PITCH])
			a_ShoulderPitch = MAX_RANGE[JOINTS::L_SHOULDER_PITCH];
		else if ( a_ShoulderPitch < MIN_RANGE[JOINTS::L_SHOULDER_PITCH])
			a_ShoulderPitch = MIN_RANGE[JOINTS::L_SHOULDER_PITCH];

		Hand2Elbow =	KinematicMatrix::transX(-LINKS::UPPER_ARM_LENGTH) *
							KinematicMatrix::rotZ(-a_ShoulderRoll) *
							KinematicMatrix::rotY(-a_ShoulderPitch) *
							Hand2Shoulder;
		
		a_ElbowYaw =	atan2(-Hand2Elbow.posV.z, -Hand2Elbow.posV.y );

		// check elbowYaw
		if ( a_ElbowYaw > MAX_RANGE[JOINTS::L_ELBOW_YAW])
			a_ElbowYaw = MAX_RANGE[JOINTS::L_ELBOW_YAW];
		else if (a_ElbowYaw < MIN_RANGE[JOINTS::L_ELBOW_YAW])
			a_ElbowYaw = MIN_RANGE[JOINTS::L_ELBOW_YAW];

		
	}
	// transform to handbase space
	Hand2HandBase = KinematicMatrix::transX(-LINKS::FOREARM_LENGTH) *
					KinematicMatrix::rotZ(-a_ElbowRoll) * 
					KinematicMatrix::rotX(-a_ElbowYaw) *
					Hand2Elbow;

	// calculate WristYaw
	a_WristYaw = atan2 ( Hand2HandBase.rotM.c[1].z, Hand2HandBase.rotM.c[2].z );

	if ( a_WristYaw > MAX_RANGE[JOINTS::L_WRIST_YAW])
		a_WristYaw = MAX_RANGE[JOINTS::L_WRIST_YAW];
	else if ( a_WristYaw < MIN_RANGE[JOINTS::L_WRIST_YAW])
		a_WristYaw = MIN_RANGE[JOINTS::L_WRIST_YAW];

	vector<float> arm;
	arm.push_back(a_ShoulderPitch);
	arm.push_back(a_ShoulderRoll);
	arm.push_back(a_ElbowYaw);
	arm.push_back(a_ElbowRoll);
	arm.push_back(a_WristYaw);

	return arm;
}



vector<float> InverseKinematics::getRArmAngles(const KinematicMatrix& desired)
{
	
	// Transformation of the desired hand position to shoulder space
	KinematicMatrix Hand2Shoulder = KinematicMatrix::transZ(-LINKS::SHOULDER_OFFSET_Z) *
									KinematicMatrix::transY(LINKS::SHOULDER_OFFSET_Y) *
									desired;

	// distance from shoulder to desired hand position
	float l = Hand2Shoulder.posV.abs();

	// normalized Vector from shoulder to desired hand position 
	Vector3<float> n = Hand2Shoulder.posV / l;

	// declare ElbowRoll
	float a_ElbowRoll;

	// check, if the desired position is reachable
	if ( l > LINKS::ARM_MAX_LENGTH )
	{
		Hand2Shoulder.posV = n * LINKS::ARM_MAX_LENGTH;
		l = LINKS::ARM_MAX_LENGTH;
		a_ElbowRoll = MIN_RANGE[JOINTS::R_ELBOW_ROLL];
	}
	else if ( l < LINKS::ARM_MIN_LENGTH )
	{
		Hand2Shoulder.posV = n * LINKS::ARM_MIN_LENGTH;
		l = LINKS::ARM_MIN_LENGTH;
		a_ElbowRoll = MAX_RANGE[JOINTS::R_ELBOW_ROLL];
	}
	else
	{
		// rule of cosines
		a_ElbowRoll = -acos((	pow(LINKS::UPPER_ARM_LENGTH,2) +
								pow(LINKS::FOREARM_LENGTH,2) -
								pow(l,2) 
							)
							/
							(	2 * LINKS::UPPER_ARM_LENGTH *
								LINKS::FOREARM_LENGTH
							)
							) + (float) M_PI;
	}

	// calculation of the circles radius on which the elbow can be positioned
	float beta = acos(	(	pow(l,2) +
							pow(LINKS::UPPER_ARM_LENGTH,2) -
							pow(LINKS::FOREARM_LENGTH,2)
						)
						/
						(	2 * l * LINKS::UPPER_ARM_LENGTH )
					  );

	float r = sin(beta) * LINKS::UPPER_ARM_LENGTH;

	// distance from shoulder to circle midpoint
	float d = cos(beta) * LINKS::UPPER_ARM_LENGTH;

	// Elbow position from desired hand position and orientation
	KinematicMatrix Shoulder2Elbow = KinematicMatrix::transX (LINKS::FOREARM_LENGTH) *
									 Hand2Shoulder.invert();

	KinematicMatrix Elbow2Shoulder = Shoulder2Elbow.invert();

	// distance from desired elbow position to circle surface
	float s = n * Elbow2Shoulder.posV - d;

	// projection of desired elbow position on circle surface
	Vector3<float> p = Elbow2Shoulder.posV - n * s;

	// circle midpoint
	Vector3<float> m = n * d;

	// Vector from m to p
	Vector3<float> vecMP = p - m;
	vecMP.normalize();

	// calculate reachable elbow position
	Vector3<float> pReachable = m + vecMP * r;
	Vector3<float> pDesired = pReachable;

	/* calculation of rotation angles, such that the shoulder coordinate-system
	 * can be transformed to the circle surface. 
	 * The y- and z- axes are in the surface, the x-axis is the normal vector
	 */
	float a1 = atan2(m.y,m.x);
	float a2 = atan2(m.z, sqrtf( pow(m.x,2) + pow(m.z,2) ) );

	//Transformation matrix to circle space
	KinematicMatrix ToCirc = KinematicMatrix::rotZ(a1) *
							 KinematicMatrix::rotY(-a2);

	Vector3<float> pToCirc = ToCirc.invert() * pReachable;

	float a3 = atan2(-pToCirc.y, pToCirc.z);

	// orthogonal circle vectors
	Vector3<float> u = ToCirc * KinematicMatrix::rotX(a3) * Vector3<float>(0,r,0);
	Vector3<float> v = ToCirc * KinematicMatrix::rotX(a3) * Vector3<float>(0,0,r);

	// set step size for iteration
	int circleParts = 60;
	float step = 2 * (float) M_PI / circleParts;

	// constant for shoulder roll limits
	float k = cos(MAX_RANGE[JOINTS::R_SHOULDER_PITCH]);

	// iteration variables 
	float t = 0;
	float bestDis = std::numeric_limits<float>::infinity();
	float bestT = t;
	bool noAvailableCirclePoint = true;
	bool optimumFound = false;

	float a_ShoulderRoll;
	float a_ShoulderPitch;
	float a_ElbowYaw;
	float a_WristYaw;
	KinematicMatrix Hand2Elbow;
	KinematicMatrix Hand2HandBase;

	// iterate on circle
	for (int i = 1; i <= circleParts; i++)
	{

		// check if desired p is reachable
		if (	pReachable.y <= LINKS::R_ELBOW_MAX_Y &&
				pReachable.y >= LINKS::R_ELBOW_MIN_Y &&
				pReachable.x >= getPitchlimit(pReachable.y,k) )
		{
			noAvailableCirclePoint = false;

			a_ShoulderRoll = asin ( pReachable.y / LINKS::UPPER_ARM_LENGTH );

			a_ShoulderPitch = atan2 ( -pReachable.z, pReachable.x );

			Hand2Elbow =	KinematicMatrix::transX(-LINKS::UPPER_ARM_LENGTH) *
							KinematicMatrix::rotZ(-a_ShoulderRoll) *
							KinematicMatrix::rotY(-a_ShoulderPitch) *
							Hand2Shoulder;
			
			a_ElbowYaw =	atan2(Hand2Elbow.posV.z, Hand2Elbow.posV.y );

			// if ElbowYaw is in range, then optimum is found
			if ( a_ElbowYaw <= MAX_RANGE[JOINTS::R_ELBOW_YAW] &&
				 a_ElbowYaw >= MIN_RANGE[JOINTS::R_ELBOW_YAW])
			{
				optimumFound = true;
				break;
			}
			else
			{
				// store distance to desired hand position
				// set ElbowYaw in Range
				if ( a_ElbowYaw > MAX_RANGE[JOINTS::R_ELBOW_YAW])
					a_ElbowYaw = MAX_RANGE[JOINTS::R_ELBOW_YAW];
				else
					a_ElbowYaw = MIN_RANGE[JOINTS::R_ELBOW_YAW];

				// transform to handbase space
				Hand2HandBase = KinematicMatrix::transX(-LINKS::FOREARM_LENGTH) *
								KinematicMatrix::rotZ(-a_ElbowRoll) * 
								KinematicMatrix::rotX(-a_ElbowYaw) *
								Hand2Elbow;

				float dis = Hand2HandBase.posV.abs();
				
				// check if distance to desired hand is better than the best found solution yet
				if ( dis < bestDis )
				{
					bestT = t;
					bestDis = dis;
				}
			}
		}
		
		// step
		t = t + i*step;
		// alternate
		step = -step;
		pReachable = m + u * sin(t) + v * cos(t);
	}


	// if no optimum could be found
	if (!optimumFound)
	{
		// if there was a possible elbow position on the circle
		if (!noAvailableCirclePoint)
			// take best t
			pReachable = m + u * sin(bestT) + v * cos(bestT);
		else
			// take the desired elbow position ( not on circle)
			pReachable = pDesired;

		a_ShoulderRoll = asin ( pReachable.y / LINKS::UPPER_ARM_LENGTH);

		a_ShoulderPitch = atan2 (-pReachable.z, pReachable.x);

		// check if the angles are in range
		if ( a_ShoulderRoll > MAX_RANGE[JOINTS::R_SHOULDER_ROLL])
			a_ShoulderRoll = MAX_RANGE[JOINTS::R_SHOULDER_ROLL];
		else if ( a_ShoulderRoll < MIN_RANGE[JOINTS::R_SHOULDER_ROLL])
			a_ShoulderRoll = MIN_RANGE[JOINTS::R_SHOULDER_ROLL];

		if ( a_ShoulderPitch > MAX_RANGE[JOINTS::R_SHOULDER_PITCH])
			a_ShoulderPitch = MAX_RANGE[JOINTS::R_SHOULDER_PITCH];
		else if ( a_ShoulderPitch < MIN_RANGE[JOINTS::R_SHOULDER_PITCH])
			a_ShoulderPitch = MIN_RANGE[JOINTS::R_SHOULDER_PITCH];

		Hand2Elbow =		KinematicMatrix::transX(-LINKS::UPPER_ARM_LENGTH) *
							KinematicMatrix::rotZ(-a_ShoulderRoll) *
							KinematicMatrix::rotY(-a_ShoulderPitch) *
							Hand2Shoulder;
		
		a_ElbowYaw =	atan2(Hand2Elbow.posV.z, Hand2Elbow.posV.y );

		// check elbowYaw
		if ( a_ElbowYaw > MAX_RANGE[JOINTS::R_ELBOW_YAW])
			a_ElbowYaw = MAX_RANGE[JOINTS::R_ELBOW_YAW];
		else if (a_ElbowYaw < MIN_RANGE[JOINTS::R_ELBOW_YAW])
			a_ElbowYaw = MIN_RANGE[JOINTS::R_ELBOW_YAW];
		
	}

	// transform to handbase space
	Hand2HandBase = KinematicMatrix::transX(-LINKS::FOREARM_LENGTH) *
					KinematicMatrix::rotZ(-a_ElbowRoll) * 
					KinematicMatrix::rotX(-a_ElbowYaw) *
					Hand2Elbow;

	// calculate WristYaw
	a_WristYaw = atan2 ( Hand2HandBase.rotM.c[1].z, Hand2HandBase.rotM.c[2].z );

	if ( a_WristYaw > MAX_RANGE[JOINTS::R_WRIST_YAW])
		a_WristYaw = MAX_RANGE[JOINTS::R_WRIST_YAW];
	else if ( a_WristYaw < MIN_RANGE[JOINTS::R_WRIST_YAW])
		a_WristYaw = MIN_RANGE[JOINTS::R_WRIST_YAW];

	vector<float> arm;
	arm.push_back(a_ShoulderPitch);
	arm.push_back(a_ShoulderRoll);
	arm.push_back(a_ElbowYaw);
	arm.push_back(a_ElbowRoll);
	arm.push_back(a_WristYaw);

	return arm;
}



vector<float> InverseKinematics::getFixedLLegAngles(const KinematicMatrix& desired, const float& a_HipYawPitch)
{
	
	// store hipyawpitch
	float hyp = a_HipYawPitch;
	
	// check if given HipYawPitch is in range
	if (hyp > MAX_RANGE[JOINTS::L_HIP_YAW_PITCH])
		hyp = MAX_RANGE[JOINTS::L_HIP_YAW_PITCH];
	else if (hyp < MIN_RANGE[JOINTS::L_HIP_YAW_PITCH])
		hyp = MIN_RANGE[JOINTS::L_HIP_YAW_PITCH];
	
	
	// First we need the torso position from foot space to calculate the desired angle position
	KinematicMatrix torso2ankle =	KinematicMatrix::transZ(-LINKS::FOOT_HEIGHT) *
									desired.invert();

	// Invert to get the desired annkle position
	KinematicMatrix ankleDesired = torso2ankle.invert();

	// transformation of the desired ankle position to rotated hip space
	KinematicMatrix ankle2hipOrthogonal =	KinematicMatrix::rotX(-45.0f * TO_RAD) *
											KinematicMatrix::transY(-LINKS::HIP_OFFSET_Y) *
											KinematicMatrix::transZ(LINKS::HIP_OFFSET_Z) *
											ankleDesired;

	// transformation to space rotated about fixed HipYawPitch angle
	KinematicMatrix ankle2RotatedHipOrthogonal = KinematicMatrix::rotZ(hyp) * ankle2hipOrthogonal;

	// distance from ankle to hip
	float l = ankle2RotatedHipOrthogonal.posV.abs();

	// normal vector to ankle
	Vector3<float> n = ankle2RotatedHipOrthogonal.posV / l;

	float a_KneePitch;

	// reachability check
	if (l > LINKS::LEG_MAX_LENGTH)
	{
		ankle2RotatedHipOrthogonal.posV = n * LINKS::LEG_MAX_LENGTH;
		l = LINKS::LEG_MAX_LENGTH;
		a_KneePitch = 0.0f;
	}
	else if ( l < LINKS::LEG_MIN_LENGTH)
	{
		ankle2RotatedHipOrthogonal.posV = n * LINKS::LEG_MIN_LENGTH;
		l = LINKS::LEG_MIN_LENGTH;
		a_KneePitch = MAX_RANGE[JOINTS::L_KNEE_PITCH];
	}
	else
	{
		// calculation of kneePitch with rule of cosines
		a_KneePitch = (float)	M_PI - 
								acos( (	pow(LINKS::THIGH_LENGTH,2) +
										pow(LINKS::TIBIA_LENGTH,2)-
										pow(l,2) )
										/
										( 2 * LINKS::THIGH_LENGTH *
										LINKS::TIBIA_LENGTH)
									 );
	}

	// calculation of HipPitch from triangle and position of ankle
	float a_HipPitch = -( acos( (	pow(LINKS::THIGH_LENGTH,2)-
									pow(LINKS::TIBIA_LENGTH,2) +
									pow(l,2) )
									/
									( 2 * LINKS::THIGH_LENGTH * l) ) +
									asin(ankle2RotatedHipOrthogonal.posV.x / l) );

	// calculation of hip roll angle from position of ankle
	float a_HipRoll = atan2 (	ankle2RotatedHipOrthogonal.posV.z,
								ankle2RotatedHipOrthogonal.posV.y) +
								3.0f/4.0f * (float) M_PI;

	// hold hip angles in range
	if (a_HipPitch > MAX_RANGE[JOINTS::L_HIP_PITCH])
		a_HipPitch = MAX_RANGE[JOINTS::L_HIP_PITCH];
	else if ( a_HipPitch < MIN_RANGE[JOINTS::L_HIP_PITCH])
		a_HipPitch = MIN_RANGE[JOINTS::L_HIP_PITCH];

	if (a_HipRoll > MAX_RANGE[JOINTS::L_HIP_ROLL])
		a_HipRoll = MAX_RANGE[JOINTS::L_HIP_ROLL];
	else if (a_HipRoll < MIN_RANGE[JOINTS::L_HIP_ROLL])
		a_HipRoll = MIN_RANGE[JOINTS::L_HIP_ROLL];

	// transformation to ankle space
	KinematicMatrix ankleRotated2ankle =	KinematicMatrix::transZ(-LINKS::TIBIA_LENGTH)*
											KinematicMatrix::rotY(a_KneePitch) * 
											KinematicMatrix::transZ(-LINKS::THIGH_LENGTH) *
											KinematicMatrix::rotY(a_HipPitch) * 
											KinematicMatrix::rotX(-(a_HipRoll + 3.0f/4.0f * (float) M_PI)) *
											ankle2RotatedHipOrthogonal;

	float a_AnkleRoll = asin(ankleRotated2ankle.rotM.c[2].y);

	float a_AnklePitch = -( atan2(	-ankleRotated2ankle.rotM.c[2].x, 
									-ankleRotated2ankle.rotM.c[2].z) );


	// hold ankle angles in range
	if (a_AnklePitch > MAX_RANGE[JOINTS::L_ANKLE_PITCH])
		a_AnklePitch = MAX_RANGE[JOINTS::L_ANKLE_PITCH];
	else if (a_AnklePitch < MIN_RANGE[JOINTS::L_ANKLE_PITCH])
		a_AnklePitch = MIN_RANGE[JOINTS::L_ANKLE_PITCH];

	if (a_AnkleRoll > MAX_RANGE_L_ANKLE_ROLL(a_AnklePitch) )
		a_AnkleRoll = MAX_RANGE_L_ANKLE_ROLL(a_AnklePitch);
	else if (a_AnkleRoll < MIN_RANGE_L_ANKLE_ROLL(a_AnklePitch) )
		a_AnkleRoll = MIN_RANGE_L_ANKLE_ROLL(a_AnklePitch);


	vector<float> leg;
	leg.push_back(hyp);
	leg.push_back(a_HipRoll);
	leg.push_back(a_HipPitch);
	leg.push_back(a_KneePitch);
	leg.push_back(a_AnklePitch);
	leg.push_back(a_AnkleRoll);

	return leg;
} 
vector<float> InverseKinematics::getFixedRLegAngles(const KinematicMatrix& desired, const float& a_HipYawPitch)
{
	
	// store hipyawpitch
	float hyp = a_HipYawPitch;
	
	// check if given HipYawPitch is in range
	if (hyp > MAX_RANGE[JOINTS::R_HIP_YAW_PITCH])
		hyp = MAX_RANGE[JOINTS::R_HIP_YAW_PITCH];
	else if (hyp < MIN_RANGE[JOINTS::R_HIP_YAW_PITCH])
		hyp = MIN_RANGE[JOINTS::R_HIP_YAW_PITCH];
	
	
	// First we need the torso position from foot space to calculate the desired angle position
	KinematicMatrix torso2ankle =	KinematicMatrix::transZ(-LINKS::FOOT_HEIGHT) *
									desired.invert();

	// Invert to get the desired annkle position
	KinematicMatrix ankleDesired = torso2ankle.invert();

	// transformation of the desired ankle position to rotated hip space
	KinematicMatrix ankle2hipOrthogonal =	KinematicMatrix::rotX(45.0f * TO_RAD) *
											KinematicMatrix::transY(LINKS::HIP_OFFSET_Y) *
											KinematicMatrix::transZ(LINKS::HIP_OFFSET_Z) *
											ankleDesired;

	// transformation to space rotated about fixed HipYawPitch angle
	KinematicMatrix ankle2RotatedHipOrthogonal = KinematicMatrix::rotZ(-hyp) * ankle2hipOrthogonal;

	// distance from ankle to hip
	float l = ankle2RotatedHipOrthogonal.posV.abs();

	// normal vector to ankle
	Vector3<float> n = ankle2RotatedHipOrthogonal.posV / l;

	float a_KneePitch;

	// reachability check
	if (l > LINKS::LEG_MAX_LENGTH)
	{
		ankle2RotatedHipOrthogonal.posV = n * LINKS::LEG_MAX_LENGTH;
		l = LINKS::LEG_MAX_LENGTH;
		a_KneePitch = 0.0f;
	}
	else if ( l < LINKS::LEG_MIN_LENGTH)
	{
		ankle2RotatedHipOrthogonal.posV = n * LINKS::LEG_MIN_LENGTH;
		l = LINKS::LEG_MIN_LENGTH;
		a_KneePitch = MAX_RANGE[JOINTS::R_KNEE_PITCH];
	}
	else
	{
		// calculation of kneePitch with rule of cosines
		a_KneePitch = (float)	M_PI - 
								acos( (	pow(LINKS::THIGH_LENGTH,2) +
										pow(LINKS::TIBIA_LENGTH,2)-
										pow(l,2) )
										/
										( 2 * LINKS::THIGH_LENGTH *
										LINKS::TIBIA_LENGTH)
									 );
	}

	// calculation of HipPitch from triangle and position of ankle
	float a_HipPitch = -( acos( (	pow(LINKS::THIGH_LENGTH,2)-
									pow(LINKS::TIBIA_LENGTH,2) +
									pow(l,2) )
									/
									( 2 * LINKS::THIGH_LENGTH * l) ) +
									asin(ankle2RotatedHipOrthogonal.posV.x / l) );

	// calculation of hip roll angle from position of ankle
	float a_HipRoll = atan2 (	ankle2RotatedHipOrthogonal.posV.z,
								ankle2RotatedHipOrthogonal.posV.y) +
								1.0f/4.0f * (float) M_PI;

	// hold hip angles in range
	if (a_HipPitch > MAX_RANGE[JOINTS::R_HIP_PITCH])
		a_HipPitch = MAX_RANGE[JOINTS::R_HIP_PITCH];
	else if ( a_HipPitch < MIN_RANGE[JOINTS::R_HIP_PITCH])
		a_HipPitch = MIN_RANGE[JOINTS::R_HIP_PITCH];

	if (a_HipRoll > MAX_RANGE[JOINTS::R_HIP_ROLL])
		a_HipRoll = MAX_RANGE[JOINTS::R_HIP_ROLL];
	else if (a_HipRoll < MIN_RANGE[JOINTS::R_HIP_ROLL])
		a_HipRoll = MIN_RANGE[JOINTS::R_HIP_ROLL];

	// transformation to ankle space
	KinematicMatrix ankleRotated2ankle =	KinematicMatrix::transZ(LINKS::TIBIA_LENGTH)*
											KinematicMatrix::rotY(-a_KneePitch) * 
											KinematicMatrix::transZ(LINKS::THIGH_LENGTH) *
											KinematicMatrix::rotY(-a_HipPitch) * 
											KinematicMatrix::rotX(-(a_HipRoll + 1.0f/4.0f * (float) M_PI)) *
											ankle2RotatedHipOrthogonal;

	float a_AnkleRoll = -asin(ankleRotated2ankle.rotM.c[2].y);

	float a_AnklePitch = -( atan2(	-ankleRotated2ankle.rotM.c[2].x, 
									 ankleRotated2ankle.rotM.c[2].z) );


	// hold ankle angles in range
	if (a_AnklePitch > MAX_RANGE[JOINTS::R_ANKLE_PITCH])
		a_AnklePitch = MAX_RANGE[JOINTS::R_ANKLE_PITCH];
	else if (a_AnklePitch < MIN_RANGE[JOINTS::R_ANKLE_PITCH])
		a_AnklePitch = MIN_RANGE[JOINTS::R_ANKLE_PITCH];

	if (a_AnkleRoll > MAX_RANGE_R_ANKLE_ROLL(a_AnklePitch) )
		a_AnkleRoll = MAX_RANGE_R_ANKLE_ROLL(a_AnklePitch);
	else if (a_AnkleRoll < MIN_RANGE_R_ANKLE_ROLL(a_AnklePitch) )
		a_AnkleRoll = MIN_RANGE_R_ANKLE_ROLL(a_AnklePitch);


	vector<float> leg;
	leg.push_back(hyp);
	leg.push_back(a_HipRoll);
	leg.push_back(a_HipPitch);
	leg.push_back(a_KneePitch);
	leg.push_back(a_AnklePitch);
	leg.push_back(a_AnkleRoll);

	return leg;
} 
float InverseKinematics::getPitchlimit(const float& y, const float&k)
	{
		return k * sqrt( pow(LINKS::UPPER_ARM_LENGTH,2) - pow(y,2) );
	}
