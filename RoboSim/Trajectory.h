#pragma once
#include <SFML/Graphics.hpp>
#include <deque>

typedef sf::Vector2f Node;
typedef std::deque<Node> Path;

class Trajectory
{
public:
	Path& getPath();
	void removeFront();
	void addToPath(Node&);
	void addToFront(const Node&);
private:
	Path path{};
};