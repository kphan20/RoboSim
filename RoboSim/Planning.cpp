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
#if DEBUG
		std::cout << curr->x << ", " << curr->y << '\n';
#endif
		trajectory->addToFront(curr->getPathNode());
		curr = curr->parent;
	}
	return trajectory;
}

OpenList::OpenList()
{
	heap = std::vector<AStarNode*>();
	AStarNode* filler = nullptr;
	heap.emplace_back(filler);
	indexMapping = std::unordered_map<std::pair<int, int>, size_t, hash_pair>();
	gValues = std::unordered_map<std::pair<int, int>, int, hash_pair>();
}

bool OpenList::less(size_t ind1, size_t ind2)
{
	return heap[ind1]->f() < heap[ind2]->f();
}

void OpenList::swap(size_t ind1, size_t ind2)
{
	auto node1 = heap[ind1];
	auto node2 = heap[ind2];
	indexMapping[node1->coords()] = ind2;
	indexMapping[node2->coords()] = ind1;
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
	gValues[node->coords()] = update->g;
	node->g = update->g;
	node->h = update->h;
	siftUp(ind);
}

void OpenList::insert(AStarNode* node) {
	auto result = gValues.find(node->coords());

	if (result != gValues.end()) {
		// no replacement needed
		if (result->second <= node->g) {
			delete node;
			return;
		}
		auto index = indexMapping.find(node->coords());
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
	indexMapping[node->coords()] = ind;
	gValues[node->coords()] = node->g;
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
	indexMapping.erase(node->coords());
	heap[1] = heap[heap.size() - 1];
	heap.pop_back();
	if (heap.size() > 2)
		siftDown(1);
	return node;
}

void OpenList::clear()
{
	for (auto node : heap) delete node;
}

Trajectory* AStar::findPath(Node start, Node end, sf::Vector2u windowSize, const ShapeList* obstacles, float robotRad)
{
	size_t nodesExpanded = 0;
	Trajectory* result = new Trajectory();
	const int xMax = windowSize.x, yMax = windowSize.y;
	bool** grid = new bool* [xMax];
	for (int i = 0; i < xMax; i++)
	{
		grid[i] = new bool[yMax];
	}

	for (auto& shape : *obstacles)
	{
		sf::IntRect bounds(shape->getNewGlobalBounds(robotRad));
		for (int i = 0; i < bounds.width; i++) {
			for (int j = 0; j < bounds.height; j++) {
				int xCoord = bounds.left + i;
				int yCoord = bounds.top + j;
				if (xCoord < 0 || xCoord >= xMax
					|| yCoord < 0 || yCoord >= yMax || !grid[xCoord][yCoord])
					continue;
				grid[xCoord][yCoord] = !shape->contains(sf::Vector2f(xCoord, yCoord), robotRad);
			}
		}
	}
	auto istart = sf::Vector2i(start);
	auto iend = sf::Vector2i(end);
	auto cmp = [](const AStarNode* a, const AStarNode* b) {
		return *a < *b;
	};
	OpenList openNodes;

	AStarNode* startNode = new AStarNode(istart);
	const AStarNode endNode(iend);

	//openNodes.emplace(startNode);
	if (grid[endNode.x][endNode.y])
		openNodes.insert(startNode);

	while (!openNodes.empty())
	{
		AStarNode* curr = openNodes.pop();
		nodesExpanded++;
#if DEBUG
		std::cout << curr->x << ", " << curr->y << '\n';
#endif
		if (*curr == endNode)
		{
			delete result;
			result = constructPath(*startNode, *curr);
			break;
	}

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

				openNodes.insert(neighbor);
			}
		}
}
	// frees memory allocated by grid
	for (int i = 0; i < xMax; i++)
		delete[] grid[i];
	delete[] grid;

	openNodes.clear();

	std::cout << "Finished after " << nodesExpanded << " nodes in ";
	return result;
}