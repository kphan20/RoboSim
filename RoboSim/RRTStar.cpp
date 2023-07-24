#include "RRTStar.h"
#include "VisualizerConstants.h"
#include <array>

RRTStar::RRTStar(int queryRad, int k, int goalBias, int steerRes) : RRT(k, goalBias, steerRes) {
	this->queryRad = queryRad;
}

void RRTStar::setUp(float robotRad, sf::Vector2f robotPos, GuiManager& gui, sf::Vector2u windowSize)
{
	if (cSpace != nullptr)
	{
		cSpace->cleanUp();
		delete cSpace;
	}
	cSpace = new RRTStarCSpace(robotRad, robotPos, gui, windowSize);
}

Trajectory* RRTStar::findPath(sf::Vector2f robotPos, int nodeSize, Node end, GuiManager& gui, sf::Vector2u windowSize, bool visualize) const
{
	Trajectory* result = new Trajectory();
	//RRTStarCSpace cSpace(robotRad, robotPos, gui, windowSize);
	const std::pair<int, int> start(robotPos.x, robotPos.y);
	// auto end = gui.getMousePos();
	auto endCoords = std::pair<int, int>(end.x, end.y);
	if (visualize) {
		gui.addNode(endCoords, sf::Color::Green);
		gui.drawNodes();
	}
	RRTStarTree tree(cSpace->quantizeCoords(start), cSpace->maxDims());

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
		RRTStarNode* nearestNode = tree.nearest(randPoint);
		std::pair<int, int> nearestPoint = nearestNode->c;
		if (nearestPoint == randPoint) continue; // want to avoid sampling repeat coords

		auto steerPath = steer(nearestNode, randPoint, endCoords, nodeSize, *cSpace);
		std::pair<int, int> newCoords = cSpace->quantizeCoords(steerPath.back());
		if (newCoords != nearestPoint && obstacleFree(steerPath, *cSpace)) {
			bool isValid = tree.put(newCoords, nearestNode);
			if (cSpace->endReached(newCoords, endCoords)) {
				break;
			}
			if (isValid) {
				std::vector<RRTStarNode*> nearbyNodes =
					tree.near(newCoords, queryRad * nodeSize);
				RRTStarNode* champ = nearestNode;
				int champCost = champ->cost + champ->distSquaredTo(newCoords);
				int nodeInd = 0;
				bool* nodeObsFree = new bool[nearbyNodes.size()];
				for (auto node : nearbyNodes) {
					nodeObsFree[nodeInd] = false;
					if (newCoords == node->c) continue;
					auto path = steer(node, newCoords, endCoords, nodeSize, *cSpace, true);
					if (obstacleFree(path, *cSpace)) {
						nodeObsFree[nodeInd] = true;
						int newCost = node->cost + node->distSquaredTo(newCoords);
						if (newCost < champCost) {
							champ = node;
							champCost = newCost;
						}
					}
					nodeInd++;
				}
				nodeInd = 0;
				if (champ != nearestNode)
					tree.recentPutNode->changeParent(champ);
				for (auto node : nearbyNodes) {
					if (!nodeObsFree[nodeInd++]) continue;
					if (tree.recentPutNode->cost + node->distSquaredTo(newCoords) < node->cost) {
						node->changeParent(tree.recentPutNode);
					}
				}
				delete[] nodeObsFree;

				if (visualize) {
					gui.addNode(newCoords, NODE_COLOR);
					tree.recentPutNode->visualizerInit();
					gui.addEdge(*tree.recentPutNode->parentVert, *tree.recentPutNode->currVert);
					if (i % 1 == 0) {
						gui.drawNodes();
						sf::sleep(sf::milliseconds(10));
					}
				}
			}
		}
	}
	auto curr = tree.nearest(endCoords);
	while (curr->parent != nullptr)
	{
		result->addToFront(curr->getPathNode());
		curr = curr->parent;
	}

	// cSpace.cleanUp();
	tree.cleanUp();
	if (visualize) gui.freeNodes();
	return result;
}