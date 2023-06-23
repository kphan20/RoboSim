#pragma once
#include "CenteredShape.h"
#include "Planning.h"

class Robot : public CenteredCircle
{
public:
	Robot(Planner&);
	Robot(Planner&, float radius, size_t pointCount = 30);
	void plan(GuiManager& gui, sf::Vector2u windowSize, bool visualize = false);
	void followTrajectory();
	void followTrajectory(float);
	void followTrajectory(Trajectory&);
	void followTrajectory(Trajectory&, float);
	void setTrajectory(Trajectory*);
	void addToCurrTrajectory(Node);
	float getRadius();
	void update(int);
private:
	Trajectory* currTrajectory;
	Planner& planner;
};