#include "Poses.h"
#include <fstream>

const char* Poses::poseFiles[Poses::POSE_MAX] =
{
	"../tuhhsdk/Poses/Home.pose",
	"../tuhhsdk/Poses/TakeAway.pose"
};

std::vector<float> Poses::getPose(const int &index)
{
	std::ifstream file(Poses::poseFiles[index]);

	std::vector<float> poseVector;
	float val;
	while( !file.eof())
	{
		file >> val;
		poseVector.push_back(val);
	}
	return poseVector;
}


