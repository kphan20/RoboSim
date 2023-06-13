#pragma once
#include "CenteredShape.h"
#include "Trajectory.h"

class Robot : public CenteredCircle
{
public:
	Robot();
	Robot(float radius, size_t pointCount = 30);
	void followTrajectory();
	void followTrajectory(float);
	void followTrajectory(Trajectory&);
	void followTrajectory(Trajectory&, float);
	void setTrajectory(Trajectory*);
	void addToCurrTrajectory(Node);
private:
	Trajectory* currTrajectory;
};