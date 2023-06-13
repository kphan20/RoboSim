#include "Trajectory.h"

//Segment::Segment(sf::Vector2f startP, sf::Vector2f endP)
//{
//	start = startP;
//	end = endP;
//};
//
//Segment::Segment(Segment prevSeg, sf::Vector2f endP)
//{
//	start = prevSeg.end;
//	end = endP;
//};

std::deque<sf::Vector2f>& Trajectory::getPath()
{
	return path;
}

void Trajectory::removeFront()
{
	path.pop_front();
}

void Trajectory::addToPath(Node& node)
{
	path.emplace_back(node);
}

void Trajectory::addToFront(const Node& node)
{
	path.emplace_front(node);
}