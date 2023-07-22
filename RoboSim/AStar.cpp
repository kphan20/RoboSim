#include "AStar.h"
#include "VisualizerConstants.h"
#include <iostream>
#include <set>
#include <chrono>

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
	return sf::Vector2f((float)x, (float)y);
}

OpenList::OpenList(int nodeSize, int xR, int yR, sf::Vector2u windowSize) : nodeSize(nodeSize), xRem(xR), yRem(yR), windowSize(windowSize)
{
	heap = std::vector<AStarNode*>();
	heap.reserve(1024);
	AStarNode* filler = nullptr;
	heap.emplace_back(filler);
	indexMapping = std::unordered_map<std::pair<int, int>, size_t, hash_pair>();
	gValues = std::unordered_map<std::pair<int, int>, int, hash_pair>();
}

std::pair<int, int> OpenList::convertCoords(const std::pair<int, int>& coords)
{
	const int x = coords.first / nodeSize + (coords.first >= (int)windowSize.x - xRem ? 1 : 0);
	const int y = coords.second / nodeSize + (coords.second >= (int)windowSize.y - yRem ? 1 : 0);
	return std::pair<int, int>(x, y);
}

bool OpenList::less(size_t ind1, size_t ind2)
{
	return heap[ind1]->f() < heap[ind2]->f();
}

void OpenList::swap(size_t ind1, size_t ind2)
{
	auto node1 = heap[ind1];
	auto node2 = heap[ind2];
	indexMapping[convertCoords(node1->coords())] = ind2;
	indexMapping[convertCoords(node2->coords())] = ind1;
	AStarNode* temp = node1;
	heap[ind1] = node2;
	heap[ind2] = temp;
}

void OpenList::siftUp(size_t ind)
{
	while (ind > 1 && less(ind, ind / 2)) {
		swap(ind, ind / 2);
		ind /= 2;
	}
}

void OpenList::siftDown(size_t ind)
{
	while (2 * ind < heap.size()) {
		size_t child_ind = 2 * ind;
		if (child_ind < heap.size() - 1 && less(child_ind + 1, child_ind))
			child_ind++;
		if (less(ind, child_ind)) break;
		swap(ind, child_ind);
		ind = child_ind;
	}
}

void OpenList::update(size_t ind, const AStarNode* update) {
	auto node = heap[ind];
	gValues[convertCoords(node->coords())] = update->g;
	node->g = update->g;
	node->h = update->h;
	siftUp(ind);
}

void OpenList::insert(AStarNode* node) {
	auto newCoords = convertCoords(node->coords());
	auto result = gValues.find(newCoords);

	if (result != gValues.end()) {
		// no replacement needed
		if (result->second <= node->g) {
			delete node;
			return;
		}
		auto index = indexMapping.find(newCoords);
		if (index != indexMapping.end())
		{
			update(index->second, node);
			delete node;
			return;
		}
	}
	// swim up logic
	heap.emplace_back(node);
	size_t ind = heap.size() - 1;
	indexMapping[newCoords] = ind;
	gValues[newCoords] = node->g;
	siftUp(ind);
}

bool OpenList::empty()
{
	return heap.size() < 2;
}

AStarNode* OpenList::pop()
{
	// pop() should never be called when empty
	auto node = heap[1];
	indexMapping.erase(convertCoords(node->coords()));
	heap[1] = heap[heap.size() - 1];
	heap.pop_back();
	if (heap.size() > 2)
		siftDown(1);
	return node;
}

void OpenList::cleanUp()
{
	for (auto node : heap) delete node;
}

Trajectory* AStar::constructPath(const AStarNode& begin, AStarNode& end) const
{
	Trajectory* trajectory = new Trajectory();
	AStarNode* curr = &end;

	while (*curr != begin)
	{
#if DEBUG
		std::cout << curr->x << ", " << curr->y << '\n';
#endif
		trajectory->addToFront(curr->getPathNode());
		curr = curr->parent;
	}
	return trajectory;
}

void AStar::setUp(float robotRad, sf::Vector2f robotPos, GuiManager& gui, sf::Vector2u windowSize)
{
	if (cSpace != nullptr) {
		cSpace->cleanUp();
		delete cSpace;
	}
	cSpace = new CSpace(robotRad, robotPos, gui, windowSize);
}

Trajectory* AStar::findPath(sf::Vector2f robotPos, int nodeSize, Node end, GuiManager& gui, sf::Vector2u windowSize, bool visualize) const
{
	using milli = std::chrono::milliseconds;
	auto start = std::chrono::high_resolution_clock::now();
	size_t nodesExpanded = 0;
	Trajectory* result = new Trajectory();
	// CSpace cSpace(robotRad, robotPos, gui, windowSize);
	bool xRemainder = windowSize.x % nodeSize, yRemainder = windowSize.y % nodeSize;
	auto istart = sf::Vector2i(robotPos);
	// Node end = gui.getMousePos();
	auto iend = sf::Vector2i(end);
	auto cmp = [](const AStarNode* a, const AStarNode* b) {
		return *a < *b;
	};
	OpenList openNodes(nodeSize, xRemainder, yRemainder, windowSize);

	AStarNode* startNode = new AStarNode(istart);
	const AStarNode endNode(iend);

	if (visualize) {
		gui.addNode(endNode.coords(), sf::Color::Green);
		gui.drawNodes();
	}

	auto endCoords = endNode.coords();
	if (cSpace->contains(endNode.coords()))
		openNodes.insert(startNode);
	auto finish = std::chrono::high_resolution_clock::now();
	std::cout << "Setup took " << std::chrono::duration_cast<milli>(finish - start).count() << " milliseconds\n";

	start = std::chrono::high_resolution_clock::now();
	while (!openNodes.empty())
	{
		AStarNode* curr = openNodes.pop();
		auto currCoords = curr->coords();
		nodesExpanded++;
#if DEBUG
		std::cout << curr->x << ", " << curr->y << '\n';
#endif
		if (cSpace->endReached(currCoords, endCoords))
		{
			delete result;
			result = constructPath(*startNode, *curr);
			break;
		}

		if (visualize) {
			gui.addNode(curr->coords(), NODE_COLOR);
			if (nodesExpanded % 10 == 0) {
				gui.drawNodes();
				sf::sleep(sf::milliseconds(10));
			}
		}

		for (int dx = -1; dx < 2; dx++) {
			for (int dy = -1; dy < 2; dy++) {
				if (dx == 0 && dy == 0) continue;
				int newX = currCoords.first + dx * nodeSize;
				int newY = currCoords.second + dy * nodeSize;
				if (!cSpace->contains(newX, newY)) continue;
				int gScore = curr->g + 1;

				int diffX = endCoords.first - newX;
				int diffY = endCoords.second - newY;
				AStarNode* neighbor = new AStarNode(newX, newY, curr, gScore, diffX * diffX + diffY * diffY);

				openNodes.insert(neighbor);
			}
		}
	}
	finish = std::chrono::high_resolution_clock::now();
	std::cout << "Search took " << std::chrono::duration_cast<milli>(finish - start).count() << " milliseconds\n";
	start = std::chrono::high_resolution_clock::now();
	// frees memory allocated by grid
	// cSpace->cleanUp();
	openNodes.cleanUp();
	if (visualize) gui.freeNodes();

	finish = std::chrono::high_resolution_clock::now();
	std::cout << "Cleanup took " << std::chrono::duration_cast<milli>(finish - start).count() << " milliseconds\n";
	std::cout << "Finished after " << nodesExpanded << " nodes in ";
	return result;
}