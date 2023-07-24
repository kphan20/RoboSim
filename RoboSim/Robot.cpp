#include "Robot.h"
#include <cmath>

Robot::Robot(Planner& plan, std::mutex& lock) : CenteredCircle(), planner(plan), m(lock)
{
	this->setFillColor(sf::Color::Red);
	currTrajectory = new (std::nothrow) Trajectory;
}

Robot::Robot(Planner& plan, std::mutex& lock, float radius, size_t nodeSize, size_t pointCount) :
	CenteredCircle(radius, pointCount), planner(plan), m(lock), nodeSize(nodeSize)
{
	this->setFillColor(sf::Color::Red);
	currTrajectory = new (std::nothrow) Trajectory;
}

void Robot::plan(GuiManager& gui, sf::Vector2u windowSize, bool visualize)
{
	planner.setUp(getRadius(), getPosition(), gui, windowSize);
	setTrajectory(planner.findPath(getPosition(), gui.nodeSize, gui.getMousePos(), gui, windowSize, visualize));
}

void Robot::schedulePlan(GuiManager& gui, sf::Vector2u windowSize, std::atomic_flag& pause, std::atomic_flag& ended)
{
	while (!ended.test())
	{
		pause.wait(false);
		if (ended.test()) return;
		sf::Vector2f pos;
		int nodeSize;
		Node end;
		{
			std::lock_guard<std::mutex> lock(m);
			pos = getPosition();
			nodeSize = gui.nodeSize;
			end = gui.getMousePos();
			planner.setUp(getRadius(), pos, gui, windowSize);
		}
		setTrajectory(planner.findPath(pos, nodeSize, end, gui, windowSize, false));
		//sf::sleep(sf::milliseconds(100));
	}
}

// probably useless
void Robot::followTrajectory()
{
	if (currTrajectory)
		followTrajectory(*currTrajectory);
}

void Robot::followTrajectory(float dist)
{
	const std::lock_guard<std::mutex> lock(m);
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
	sf::Vector2f diff;
	// attempt to reduce jittering
	while (extractedPath.size() > 0) {
		diff = extractedPath.front() - this->getPosition();
		if (diff.x * diff.x + diff.y * diff.y > 2 * nodeSize * nodeSize) {
			trajectory.removeFront();
		}
		else {
			break;
		}
	}
	if (extractedPath.size() < 1) return;

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
	std::lock_guard<std::mutex> lock(m);
	delete currTrajectory;
	currTrajectory = traj;
}

void Robot::addToCurrTrajectory(Node node)
{
	std::lock_guard<std::mutex> lock(m);
	currTrajectory->addToPath(node);
}

float Robot::getRadius()
{
	return this->shape.getRadius();
}

void Robot::update(int elapsedTime)
{
	followTrajectory(elapsedTime);
}

const sf::Vector2f& Robot::getPosition() const {
	//std::lock_guard<std::mutex> lock(m);
	// TODO potentially need to lock individual calls to getPosition
	return CenteredCircle::getPosition();
}