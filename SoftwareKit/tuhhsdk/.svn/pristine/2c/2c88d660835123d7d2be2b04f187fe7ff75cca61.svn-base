#include "Alias.h"
#include "Definitions/keys.h"

std::vector<std::vector<std::string> > Alias::aliases(Alias::ALIAS_MAX);


void Alias::init()
{
	// create aliases

	// joint body
	std::vector<std::string> jab(keys::joints::JOINTS_MAX+1);
	std::vector<std::string> jhb(keys::joints::JOINTS_MAX+1);
	jab[0] = "JointActuatorBody";
	jhb[0] = "JointHardnessBody";
	for (int i = 0; i < keys::joints::JOINTS_MAX; i++)
	{
		jab[i+1] = keys::joints::actuatorKey[i];
		jhb[i+1] = keys::joints::hardnessKey[i];
	}
	aliases[JOINT_ACTUATOR_BODY] = jab;
	aliases[JOINT_HARDNESS_BODY] = jhb;

	// joint arms and legs
	std::vector<std::string> jall(7);
	std::vector<std::string> jarl(7);
	std::vector<std::string> jala(7);
	std::vector<std::string> jara(7);
	std::vector<std::string> jhll(7);
	std::vector<std::string> jhrl(7);
	std::vector<std::string> jhla(7);
	std::vector<std::string> jhra(7);
	jall[0] = "JointActuatorLLeg";
	jarl[0] = "JointActuatorRLeg";
	jala[0] = "JointActuatorLArm";
	jara[0] = "JointActuatorRArm";
	jhll[0] = "JointHardnessLLeg";
	jhrl[0] = "JointHardnessRLeg";
	jhla[0] = "JointHardnessLArm";
	jhra[0] = "JointHardnessRArm";
	for(int i = 0; i < 6; i++)
	{
		jall[i+1] = keys::joints::actuatorKey[keys::joints::L_HIP_YAW_PITCH + i];
		jarl[i+1] = keys::joints::actuatorKey[keys::joints::R_HIP_YAW_PITCH + i];
		jala[i+1] = keys::joints::actuatorKey[keys::joints::L_SHOULDER_PITCH + i];
		jara[i+1] = keys::joints::actuatorKey[keys::joints::R_SHOULDER_PITCH + i];
		jhll[i+1] = keys::joints::hardnessKey[keys::joints::L_HIP_YAW_PITCH + i];
		jhrl[i+1] = keys::joints::hardnessKey[keys::joints::R_HIP_YAW_PITCH + i];
		jhla[i+1] = keys::joints::hardnessKey[keys::joints::L_SHOULDER_PITCH + i];
		jhra[i+1] = keys::joints::hardnessKey[keys::joints::R_SHOULDER_PITCH + i];
	}
	aliases[JOINT_ACTUATOR_L_LEG] = jall;
	aliases[JOINT_ACTUATOR_R_LEG] = jarl;
	aliases[JOINT_ACTUATOR_L_ARM] = jala;
	aliases[JOINT_ACTUATOR_R_ARM] = jara;
	aliases[JOINT_HARDNESS_L_LEG] = jhll;
	aliases[JOINT_HARDNESS_R_LEG] = jhrl;
	aliases[JOINT_HARDNESS_L_ARM] = jhla;
	aliases[JOINT_HARDNESS_R_ARM] = jhra;

	// joint head
	std::vector<std::string> jah(3);
	std::vector<std::string> jhh(3);
	jah[0] = "JointActuatorHead";
	jhh[0] = "JointHardnessHead";
	jah[1] = keys::joints::actuatorKey[keys::joints::HEAD_YAW];
	jah[2] = keys::joints::actuatorKey[keys::joints::HEAD_PITCH];
	jhh[1] = keys::joints::hardnessKey[keys::joints::HEAD_YAW];
	jhh[2] = keys::joints::hardnessKey[keys::joints::HEAD_PITCH];
	aliases[JOINT_ACTUATOR_HEAD] = jah;
	aliases[JOINT_HARDNESS_HEAD] = jhh;
}

std::string Alias::aliasName(const int &index)
{
	return aliases[index][0];
}

std::vector<std::string> Alias::alias(const int &index)
{
	return aliases[index];
}
