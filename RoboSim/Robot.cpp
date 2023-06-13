#include "Robot.h"
#include <cmath>

Robot::Robot() : CenteredCircle()
{
	this->setFillColor(sf::Color::Red);
	currTrajectory = new (std::nothrow) Trajectory;
}

Robot::Robot(float radius, size_t pointCount) : CenteredCircle(radius, pointCount)
{
	this->setFillColor(sf::Color::Red);
	currTrajectory = new (std::nothrow) Trajectory;
}

// probably useless
void Robot::followTrajectory()
{
	if (currTrajectory)
		followTrajectory(*currTrajectory);
}

void Robot::followTrajectory(float dist)
{
	if (currTrajectory)
		followTrajectory(*currTrajectory, dist);
}

void Robot::followTrajectory(Trajectory& trajectory)
{
	Path extractedPath = trajectory.getPath();
	for (sf::Vector2f waypoint : extractedPath)
		this->setPosition(waypoint);
}

void Robot::followTrajectory(Trajectory& trajectory, float dist)
{
	Path& extractedPath = trajectory.getPath();
	if (extractedPath.size() < 1) return;
	Node test = extractedPath.front();
	auto diff = extractedPath[0] - this->getPosition();
	float ratio = dist * dist / (diff.x * diff.x + diff.y * diff.y);
	if (ratio >= 1.0) {
		this->setPosition(extractedPath[0]);
		trajectory.removeFront();
		return;
	}
	// this->setPosition(extractedPath[0] + sqrt(ratio) * diff);
	this->move(sqrt(ratio) * diff);
}

void Robot::setTrajectory(Trajectory* traj)
{
	delete currTrajectory;
	currTrajectory = traj;
}

void Robot::addToCurrTrajectory(Node node)
{
	currTrajectory->addToPath(node);
}