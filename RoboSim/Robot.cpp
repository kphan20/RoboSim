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

bool Robot::trajectoryDone()
{
	return currTrajectory->getPath().empty();
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
		bool isNewDest = false, isNewSpace = false;
		{
			std::lock_guard<std::mutex> lock(m);
			isNewDest = destinationChanged();
			isNewSpace = gui.shapesChanged();
			if (isNewDest || isNewSpace) {
				pos = getPosition();
				nodeSize = gui.nodeSize;
				end = Node(currDest.x, currDest.y); //gui.getMousePos();
				planner.setUp(getRadius(), pos, gui, windowSize);
			}
		}
		if (isNewDest || isNewSpace)
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
		followTrajectory(*currTrajectory, dist / 2);
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

	sf::Vector2f diff = extractedPath.front() - this->getPosition();
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
	if (currTrajectory != nullptr) delete currTrajectory;
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

void Robot::setDestination(Node dest)
{
	std::lock_guard<std::mutex> lock(m);
	currDest = Node(dest.x, dest.y);
	destChanged = true;
}

bool Robot::destinationChanged()
{
	if (destChanged) {
		destChanged = false;
		return true;
	}
	return false;
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