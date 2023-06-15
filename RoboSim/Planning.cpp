#include "GuiManager.h"
#include "Planning.h"
#include <iostream>
#include <set>

AStarNode::AStarNode(sf::Vector2i& pos, int g, int h) : x(pos.x), y(pos.y)
{
	this->g = g;
	this->h = h;
	this->parent = nullptr;
}

AStarNode::AStarNode(int xPos, int yPos, AStarNode* parent, int g, int h) : x(xPos), y(yPos)
{
	this->g = g;
	this->h = h;
	this->parent = parent;
}

AStarNode::AStarNode(sf::Vector2i& pos, AStarNode* parent, int g, int h) : x(pos.x), y(pos.y)
{
	this->g = g;
	this->h = h;
	this->parent = parent;
}

const Node& AStarNode::getPathNode()
{
	return sf::Vector2f(x, y);
}

Trajectory* AStar::constructPath(const AStarNode& begin, AStarNode& end)
{
	Trajectory* trajectory = new Trajectory();
	AStarNode* curr = &end;

	while (*curr != begin)
	{
		std::cout << curr->x << ", " << curr->y << '\n';
		trajectory->addToFront(curr->getPathNode());
		curr = curr->parent;
	}
	return trajectory;
}



Trajectory* AStar::findPath(Node start, Node end, sf::Vector2u windowSize, const ShapeList* obstacles)
{
	Trajectory* result = new Trajectory();
	const int xMax = windowSize.x, yMax = windowSize.y;
	bool** grid = new bool* [xMax];
	for (int i = 0; i < xMax; i++)
	{
		grid[i] = new bool[yMax];
	}

	for (auto& shape : *obstacles)
	{
		sf::IntRect bounds(shape->getGlobalBounds());
		for (int i = 0; i < bounds.width; i++) {
			for (int j = 0; j < bounds.height; j++) {
				int xCoord = bounds.left + i;
				int yCoord = bounds.top + j;
				if (xCoord < 0 || xCoord >= xMax
					|| yCoord < 0 || yCoord >= yMax || !grid[xCoord][yCoord])
					continue;
				grid[xCoord][yCoord] = !shape->contains(sf::Vector2f(xCoord, yCoord));
			}
		}
	}
	auto istart = sf::Vector2i(start);
	auto iend = sf::Vector2i(end);
	auto cmp = [](const AStarNode* a, const AStarNode* b) {
		return *a < *b;
	};
	std::set<AStarNode*, decltype(cmp)> openNodes;
	std::set<AStarNode*, decltype(cmp)> closedNodes;

	AStarNode* startNode = new AStarNode(istart);
	const AStarNode endNode(iend);

	openNodes.emplace(startNode);

	while (!openNodes.empty())
	{
		auto currIter = openNodes.begin();
		AStarNode* curr = *currIter;
		std::cout << curr->x << ", " << curr->y << '\n';
		if (*curr == endNode)
		{
			std::cout << "Start construction";
			delete result;
			result = constructPath(*startNode, *curr);
			std::cout << "End construction";
			break;
		}

		openNodes.erase(currIter);
		closedNodes.emplace(curr);

		for (int dx = -1; dx < 2; dx++) {
			for (int dy = -1; dy < 2; dy++) {
				if (dx == 0 && dy == 0) continue;
				int newX = curr->x + dx;
				if (newX < 0 || newX >= xMax) continue;
				int newY = curr->y + dy;
				if (newY < 0 || newY >= yMax || !grid[newX][newY]) continue;
				int gScore = curr->g + 1;

				int diffX = endNode.x - newX;
				int diffY = endNode.y - newY;
				AStarNode* neighbor = new AStarNode(newX, newY, curr, gScore, diffX * diffX + diffY * diffY);

				const auto neighborCheck = openNodes.find(neighbor);
				const auto closedCheck = closedNodes.find(neighbor);

				if (neighborCheck != openNodes.end() && closedCheck != closedNodes.end())
					std::cout << "Something wrong\n";

				if (neighborCheck != openNodes.end()) {
					if ((*neighborCheck)->g <= neighbor->g)
					{
						delete neighbor;
						continue;
					}
					AStarNode* obsolete = *neighborCheck;
					openNodes.erase(neighborCheck);
					delete obsolete;
				}
				else if (closedCheck != closedNodes.end())
				{
					if ((*closedCheck)->g <= neighbor->g)
					{
						delete neighbor;
						continue;
					}
					AStarNode* obsolete = *closedCheck;
					closedNodes.erase(closedCheck);
					delete obsolete;
				}
				openNodes.emplace(neighbor);
			}
		}
	}
	// frees memory allocated by grid
	for (int i = 0; i < xMax; i++)
		delete[] grid[i];
	delete[] grid;

	for (AStarNode* node : openNodes) delete node;
	for (AStarNode* node : closedNodes) delete node;
	std::cout << "Finished";
	return result;
}