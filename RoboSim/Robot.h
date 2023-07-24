#pragma once
#include "CenteredShape.h"
#include "Planning.h"
#include <mutex>

class Robot : public CenteredCircle
{
public:
	Robot(Planner&, std::mutex&);
	Robot(Planner&, std::mutex&, float radius, size_t nodeSize, size_t pointCount = 30);
	void plan(GuiManager& gui, sf::Vector2u windowSize, bool visualize = false);
	void schedulePlan(GuiManager&, sf::Vector2u, std::atomic_flag&, std::atomic_flag&);
	void followTrajectory();
	void followTrajectory(float);
	void followTrajectory(Trajectory&);
	void followTrajectory(Trajectory&, float);
	void setTrajectory(Trajectory*);
	void addToCurrTrajectory(Node);
	float getRadius();
	bool trajectoryDone();
	bool destinationChanged();
	void setDestination(Node dest);
	void update(int);
	const sf::Vector2f& getPosition() const override;
private:
	Trajectory* currTrajectory = nullptr;
	Planner& planner;
	std::mutex& m;
	size_t nodeSize;
	Node currDest;
	bool destChanged;
};