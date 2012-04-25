#pragma once

#include "Definitions/robotConstants.h"
#include <vector>

/// Poses Module
/**
 * This class handles robot poses
 * Poses are stored in the Poses folder. For Example <a href="Home.pose">Home.pose</a>
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */
class Poses
{
public:
	/// enumeration of predefined poses
	enum ENUM_POSE{
		HOME,
		TAKE_AWAY,
		POSE_MAX
	};

	/** Get a pose
	 * @param index The index of a pose
	 * @return A vector of joint angles
	 */
	static std::vector<float>getPose(const int& index);

private:
	const static char* poseFiles[POSE_MAX];
};
