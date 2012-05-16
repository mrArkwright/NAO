#pragma once
/**
* Sample class for NAO */
class Sample {
public:

	Sample (){};

	static void moveUpperBody(const float& dx, const float& dy, const float& dz, const int& time);
	static void statBalance(const bool& moveRightFootFlag, const int& time);
	static void moveFoot(const float& heigth, const int& time);
	static void moveLArm(const float& angle, const int& time);
};
