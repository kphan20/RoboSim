#pragma once
#include "Trajectory.h"
#include <SFML/Graphics.hpp>


class AStarNode
{
public:
	AStarNode(sf::Vector2i& pos, int g = 0, int h = 0);
	AStarNode(int x, int y, AStarNode* parent, int g = 0, int h = 0);
	AStarNode(sf::Vector2i&, AStarNode* parent, int g = 0, int h = 0);
	AStarNode* parent;
	bool operator<(const AStarNode& other) const {
		if (x == other.x && y == other.y) return false;
		if ((g + h) == (other.g + other.h)) {
			if (h == other.h) {
				if (x == other.x) {
					return y < other.y;
				}
				return x < other.x;
			}
			return h < other.h;
		}
		return (g + h) < (other.g + other.h);
	}
	bool operator==(const AStarNode& other)
	{
		return x == other.x && y == other.y;
	}
	bool operator!=(const AStarNode& other)
	{
		return x != other.x || y != other.y;
	}
	const Node& getPathNode();
	const int x, y;
	int g = 0, h = 0;
};

class AStar
{
public:
	static Trajectory* findPath(Node, Node, sf::Vector2u, const ShapeList*);
private:
	static Trajectory* constructPath(const AStarNode&, AStarNode&);
};