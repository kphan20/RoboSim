#include "RRT.h"
#include "VisualizerConstants.h"
#include <array>

RRT::RRT(int k, int goalBias, int steerRes)
{
	this->k = k;
	this->goalBias = goalBias;
	this->steerRes = steerRes;
}

void RRT::setUp(float robotRad, sf::Vector2f robotPos, GuiManager& gui, sf::Vector2u windowSize)
{
	if (cSpace != nullptr)
	{
		cSpace->cleanUp();
		delete cSpace;
	}
	cSpace = new RRTCSpace(robotRad, robotPos, gui, windowSize);
}

Trajectory* RRT::findPath(sf::Vector2f robotPos, int nodeSize, Node end, GuiManager& gui, sf::Vector2u windowSize, bool visualize) const
{
	Trajectory* result = new Trajectory();
	// RRTCSpace cSpace(robotRad, robotPos, gui, windowSize);
	const std::pair<int, int> start(robotPos.x, robotPos.y);
	// auto end = gui.getMousePos();
	auto endCoords = std::pair<int, int>(end.x, end.y);
	if (!cSpace->contains(endCoords)) {
		// cSpace.cleanUp();
		return result;
	}
	if (visualize) {
		gui.addNode(endCoords, sf::Color::Green);
		gui.drawNodes();
	}
	KDTree<KDTreeNode> tree(start, cSpace->maxDims());

	std::array<std::pair<int, int>, 8> directions
	{ {std::pair<int, int>(1, 1),std::pair<int, int>(0, 1),
	std::pair<int, int>(-1, 1),std::pair<int, int>(-1, 0),
	std::pair<int, int>(-1, -1),std::pair<int, int>(0, -1),
	std::pair<int, int>(1, -1),std::pair<int, int>(1, 0)} };
	//{ {std::pair<int, int>(1, -1),std::pair<int, int>(0, -1),
	//	std::pair<int, int>(-1, -1),std::pair<int, int>(-1, 0),
	//	std::pair<int, int>(-1, 1),std::pair<int, int>(0, 1),
	//	std::pair<int, int>(1, 1),std::pair<int, int>(1, 0)} };

	for (int i = 0; i < k; i++)
	{
		std::pair<int, int> randPoint;
		// RPP fails with euclidean heuristic and obstacles
		// branching strategy could also be the issue
		if (std::rand() % 100 < goalBias) {
			randPoint = endCoords;
		}
		else {
			randPoint = cSpace->sample();
		}
		KDTreeNode* nearestNode = tree.nearest(randPoint);
		std::pair<int, int> nearestPoint = nearestNode->c;
		if (nearestPoint == randPoint) continue; // want to avoid sampling repeat coords
		int dx = randPoint.first - nearestPoint.first;
		int dy = randPoint.second - nearestPoint.second;
		int angle = ((int)((std::atan2(dy, dx) + 2 * PI) * 180 / PI)) % 360;
		size_t directionInd = 0;
		if (angle >= 22 && angle < 67) {
			directionInd = 0;
		}
		else if (angle >= 67 && angle < 112) {
			directionInd = 1;
		}
		else if (angle >= 112 && angle < 157) {
			directionInd = 2;
		}
		else if (angle >= 157 && angle < 202) {
			directionInd = 3;
		}
		else if (angle >= 202 && angle < 247)
		{
			directionInd = 4;
		}
		else if (angle >= 247 && angle < 292)
		{
			directionInd = 5;
		}
		else if (angle >= 292 && angle < 337) {
			directionInd = 6;
		}
		else {
			directionInd = 7;
		}

		auto newDirection = directions[directionInd];
		auto newCoords = incrementCoord(nearestPoint, nodeSize, newDirection);
		/*std::pair<int, int>(nearestPoint.first + newDirection.first * gui.nodeSize,
			nearestPoint.second + newDirection.second * gui.nodeSize);*/

		bool valid = false;

		const size_t len = directions.size();
		size_t p1 = (directionInd - 1 + len) % len;
		size_t p2 = (directionInd + 1) % len;
		bool useP1 = true;
		for (int j = 0; j < len; j++) {
			if (cSpace->contains(newCoords) && tree.put(newCoords, nearestNode)) {
				valid = true;
				auto steerTest = std::pair<int, int>(newCoords);
				for (int step = 1; step < steerRes; step++) {
					steerTest = incrementCoord(steerTest, nodeSize, newDirection);
					if (!cSpace->contains(steerTest) || !tree.put(steerTest, nearestNode)) break;
					if (visualize) {
						gui.addNode(newCoords, sf::Color::Blue);
					}
					newCoords = steerTest;
					if (cSpace->endReached(steerTest, newCoords)) break;
				}
				break;
			}
			if (useP1) {
				newDirection = directions[p1];
				p1 = (p1 - 1 + len) % len;
			}
			else {
				newDirection = directions[p2];
				p2 = (p2 + 1) % len;
			}
			newCoords = incrementCoord(nearestPoint, nodeSize, newDirection);//std::pair<int, int>(nearestPoint.first + newDirection.first * gui.nodeSize,
			//nearestPoint.second + newDirection.second * gui.nodeSize);

			useP1 = !useP1;
		}

		if (valid) {
			if (cSpace->endReached(newCoords, endCoords)) {
				break;
			}
			if (visualize) {
				gui.addNode(newCoords, NODE_COLOR);
				if (i % 10 == 0) {
					gui.drawNodes();
					sf::sleep(sf::milliseconds(1));
				}
			}
		}
	}
	auto curr = tree.nearest(endCoords);
	while (curr != nullptr)
	{
		result->addToFront(curr->getPathNode());
		curr = curr->parent;
	}
	result->removeFront();
	// cSpace.cleanUp();
	tree.cleanUp();
	if (visualize) gui.freeNodes();
	return result;
}