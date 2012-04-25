#pragma once
#include <vector>
#include <string>

/// Alias Module
/**
 * This class handles the aliases of sensor or actuator chains
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */
class Alias
{
public:
	/// Enumeration of predefined aliases
	enum enumAlias{
		JOINT_ACTUATOR_BODY, JOINT_ACTUATOR_L_LEG, JOINT_ACTUATOR_R_LEG,
		JOINT_ACTUATOR_L_ARM, JOINT_ACTUATOR_R_ARM, JOINT_ACTUATOR_HEAD,
		JOINT_HARDNESS_BODY, JOINT_HARDNESS_L_LEG, JOINT_HARDNESS_R_LEG,
		JOINT_HARDNESS_L_ARM, JOINT_HARDNESS_R_ARM, JOINT_HARDNESS_HEAD,
		ALIAS_MAX
	};

	/// initializes the Alias Module. Called from TUHH::init
	static void init();

	/** Returns the Name of an Alias
	 * @param index The index of the alias (A value of enumAlias)
	 * @return A string containing the Aliases name
	 */
	static std::string aliasName(const int& index);

	/** Returns the Alias
	 * @param index The index of the alias (A value of enumAlias)
	 * @return A vector of strings containing the aliases name and
	 * the contents of the alias
	 */
	static std::vector<std::string> alias(const int& index);

private:

	static std::vector<std::vector<std::string> > aliases;
};
