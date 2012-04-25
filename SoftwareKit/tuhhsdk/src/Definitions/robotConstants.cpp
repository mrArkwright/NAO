#include "robotConstants.h"

// minimal range for LAnkleRoll ( depends on LAnklePitch )
extern const float MIN_RANGE_L_ANKLE_ROLL(const float &LAnklePitch)
{
	if (LAnklePitch <= LOOKUP_ANKLE[0][0])
		return LOOKUP_ANKLE[0][1];
	else if (LAnklePitch >= LOOKUP_ANKLE[LOOKUP_ANKLE_POINTS-1][0])
		return LOOKUP_ANKLE[LOOKUP_ANKLE_POINTS-1][1];
	else
	{
		int i;
		// find fitting segment
		for (i = 0; i < LOOKUP_ANKLE_POINTS -1; i++)
		{
			if (LAnklePitch > LOOKUP_ANKLE[i][0] && LAnklePitch <= LOOKUP_ANKLE[i+1][0])
				break;
		}
		// slope between points
		float m = ( LOOKUP_ANKLE[i+1][1] - LOOKUP_ANKLE[i][1] )	/
							( LOOKUP_ANKLE[i+1][0] - LOOKUP_ANKLE[i][0] );

		return	LOOKUP_ANKLE[i][1] + m * (LAnklePitch - LOOKUP_ANKLE[i][0]);
	}
}

// minimal range for RAnkleRoll ( depends on RAnklePitch )
extern const float MIN_RANGE_R_ANKLE_ROLL(const float &RAnklePitch)
{
	if (RAnklePitch <= LOOKUP_ANKLE[0][0])
		return -LOOKUP_ANKLE[0][2];
	else if (RAnklePitch >= LOOKUP_ANKLE[LOOKUP_ANKLE_POINTS-1][0])
		return -LOOKUP_ANKLE[LOOKUP_ANKLE_POINTS-1][2];
	else
	{
		int i;
		// find fitting segment
		for (i = 0; i < LOOKUP_ANKLE_POINTS -1; i++)
		{
			if (RAnklePitch > LOOKUP_ANKLE[i][0] && RAnklePitch <= LOOKUP_ANKLE[i+1][0])
				break;
		}
		// slope between points
		float m = ( -LOOKUP_ANKLE[i+1][2] + LOOKUP_ANKLE[i][2] )	/
							( LOOKUP_ANKLE[i+1][0] - LOOKUP_ANKLE[i][0] );

		return	-LOOKUP_ANKLE[i][2] + m * (RAnklePitch - LOOKUP_ANKLE[i][0]);
	}
}

// maximal range for LAnkleRoll ( depends on LAnklePitch )
extern const float MAX_RANGE_L_ANKLE_ROLL(const float &LAnklePitch)
{
	if (LAnklePitch <= LOOKUP_ANKLE[0][0])
		return LOOKUP_ANKLE[0][2];
	else if (LAnklePitch >= LOOKUP_ANKLE[LOOKUP_ANKLE_POINTS-1][0])
		return LOOKUP_ANKLE[LOOKUP_ANKLE_POINTS-1][2];
	else
	{
		int i;
		// find fitting segment
		for (i = 0; i < LOOKUP_ANKLE_POINTS -1; i++)
		{
			if (LAnklePitch > LOOKUP_ANKLE[i][0] && LAnklePitch <= LOOKUP_ANKLE[i+1][0])
				break;
		}
		// slope between points
		float m = ( LOOKUP_ANKLE[i+1][2] - LOOKUP_ANKLE[i][2] )	/
							( LOOKUP_ANKLE[i+1][0] - LOOKUP_ANKLE[i][0] );

		return	LOOKUP_ANKLE[i][2] + m * (LAnklePitch - LOOKUP_ANKLE[i][0]);
	}
}

// maximal range for RAnkleRoll ( depends on RAnklePitch )
extern const float MAX_RANGE_R_ANKLE_ROLL(const float &RAnklePitch)
{
	if (RAnklePitch <= LOOKUP_ANKLE[0][0])
		return -LOOKUP_ANKLE[0][1];
	else if (RAnklePitch >= LOOKUP_ANKLE[LOOKUP_ANKLE_POINTS-1][0])
		return -LOOKUP_ANKLE[LOOKUP_ANKLE_POINTS-1][1];
	else
	{
		int i;
		// find fitting segment
		for (i = 0; i < LOOKUP_ANKLE_POINTS -1; i++)
		{
			if (RAnklePitch > LOOKUP_ANKLE[i][0] && RAnklePitch <= LOOKUP_ANKLE[i+1][0])
				break;
		}
		// slope between points
		float m = ( -LOOKUP_ANKLE[i+1][1] + LOOKUP_ANKLE[i][1] )	/
							( LOOKUP_ANKLE[i+1][0] - LOOKUP_ANKLE[i][0] );

		return	-LOOKUP_ANKLE[i][1] + m * (RAnklePitch - LOOKUP_ANKLE[i][0]);
	}
}

// minimal range for HeadPitch ( depends on HeadYaw )
extern const float MIN_RANGE_HEAD_PITCH(const float &HeadYaw)
{
	if (HeadYaw <= LOOKUP_HEAD[0][0])
		return LOOKUP_HEAD[0][1];
	else if (HeadYaw >= LOOKUP_HEAD[LOOKUP_HEAD_POINTS-1][0])
		return LOOKUP_HEAD[LOOKUP_HEAD_POINTS-1][1];
	else
	{
		int i;
		// find fitting segment
		for (i = 0; i < LOOKUP_HEAD_POINTS -1; i++)
		{
			if (HeadYaw > LOOKUP_HEAD[i][0] && HeadYaw <= LOOKUP_HEAD[i+1][0])
				break;
		}
		// slope between points
		float m = ( LOOKUP_HEAD[i+1][1] - LOOKUP_HEAD[i][1] )	/
							( LOOKUP_HEAD[i+1][0] - LOOKUP_HEAD[i][0] );

		return	LOOKUP_HEAD[i][1] + m * (HeadYaw - LOOKUP_HEAD[i][0]);
	}
}

// maximal range for HeadPitch ( depends on HeadYaw )
extern const float MAX_RANGE_HEAD_PITCH(const float &HeadYaw)
{
	if (HeadYaw <= LOOKUP_HEAD[0][0])
		return LOOKUP_HEAD[0][2];
	else if (HeadYaw >= LOOKUP_HEAD[LOOKUP_HEAD_POINTS-1][0])
		return LOOKUP_HEAD[LOOKUP_HEAD_POINTS-1][2];
	else
	{
		int i;
		// find fitting segment
		for (i = 0; i < LOOKUP_HEAD_POINTS -1; i++)
		{
			if (HeadYaw > LOOKUP_HEAD[i][0] && HeadYaw <= LOOKUP_HEAD[i+1][0])
				break;
		}
		// slope between points
		float m = ( LOOKUP_HEAD[i+1][2] - LOOKUP_HEAD[i][2] )	/
							( LOOKUP_HEAD[i+1][0] - LOOKUP_HEAD[i][0] );

		return	LOOKUP_HEAD[i][2] + m * (HeadYaw - LOOKUP_HEAD[i][0]);
	}
}
