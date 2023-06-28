#pragma once
#include "RRT.h"
#include <set>

constexpr float ROOT2 = 1.8f;

class RRTStarCSpace : public RRTCSpace
{
public:
	RRTStarCSpace(float robotRad, sf::Vector2f robotPos, GuiManager& gui, sf::Vector2u windowSize) :
		RRTCSpace(robotRad, robotPos, gui, windowSize) {}
	std::pair<int, int> quantizeCoords(const std::pair<int, int>& coords)
	{
		const int x = (int)(coords.first / nodeSize) * nodeSize;
		const int y = (int)(coords.second / nodeSize) * nodeSize;
		return std::pair<int, int>(x, y);
	}
};

class RRTRect : public KDRect
{
public:
	bool contains(const std::pair<int, int>& coords)
	{
		return coords.first >= l && coords.first <= r &&
			coords.second >= t && coords.second <= b;
	}
	bool intersects(int left, int right, int top, int bot)
	{
		if (left == right && top == bot) return false;
		return !(left > r || right < l || top > b || bot < t);
	}
};

class RRTStarNode {
public:
	RRTStarNode* left;
	RRTStarNode* right;
	RRTStarNode* parent;
	sf::Vertex* parentVert = nullptr;
	sf::Vertex* currVert = nullptr;
	std::set<RRTStarNode*> children;
	const std::pair<int, int> c;
	int cost;
	RRTStarNode(const std::pair<int, int>& coords, RRTStarNode* parent = nullptr) :
		c(coords), parent(parent) {
		left = nullptr;
		right = nullptr;
		cost = 0;
		if (parent != nullptr) {
			parent->children.emplace(this);
		}
	}
	void visualizerInit() {
		auto parentCoords = sf::Vector2f((float)parent->c.first, (float)parent->c.second);
		parentVert = new sf::Vertex(parentCoords, sf::Color::Black);
		currVert = new sf::Vertex(sf::Vector2f((float)c.first, (float)c.second));
	}
	bool lessThan(std::pair<int, int> other, bool isX)
	{
		return isX ? c.first >= other.first : c.second >= other.second;
	}
	int distSquaredTo(const std::pair<int, int>& other) const
	{
		const int dx = other.first - c.first;
		const int dy = other.second - c.second;
		return dx * dx + dy * dy;
	}
	const Node& getPathNode() const
	{
		return sf::Vector2f((float)c.first, (float)c.second);
	}
	void changeParent(RRTStarNode* newParent) {
		parent->children.erase(this);
		parent = newParent;
		parent->children.emplace(this);
		setCost(parent->cost + distSquaredTo(parent->c));
		if (parentVert != nullptr) {
			parentVert->position = sf::Vector2f((float)parent->c.first, (float)parent->c.second);
		}
	}
	void setCost(int cost) {
		this->cost = cost;
		for (auto child : children) {
			child->setCost(cost + distSquaredTo(child->c));
		}
	}
	void cleanUp() {
		if (parentVert == nullptr) return;
		delete parentVert;
		delete currVert;
	}
};

class RRTStarTree : public KDTree<RRTStarNode>
{
public:
	int size;
	RRTStarTree(const std::pair<int, int>& start, std::pair<int, int> maxDims) :
		KDTree(start, maxDims)
	{
		size = 1;
	}

	bool put(const std::pair<int, int>& coords, RRTStarNode* parent) override {
		bool result = KDTree::put(coords, parent);
		int newCost = parent->cost + parent->distSquaredTo(coords);
		if (!result) {
			if (newCost >= recentPutNode->cost)
				return false;
			recentPutNode->changeParent(parent);
			size++;
			return true;
		}
		recentPutNode->setCost(newCost);
		size++;
		return true;
	}

	std::vector<RRTStarNode*> near(std::pair<int, int>& coords, int queryRadius) {
		std::vector<RRTStarNode*> results;
		RRTRect queryRect;
		queryRect.Init(coords.first - queryRadius, coords.first + queryRadius,
			coords.second - queryRadius, coords.second + queryRadius);
		_near(root, queryRect, results, true, 0, xMax, 0, yMax);
		return results;
	}
private:
	void _near(RRTStarNode* curr, RRTRect& queryRect, std::vector<RRTStarNode*>& results,
		bool isX, int left, int right, int top, int bottom) {
		if (curr == nullptr || !queryRect.intersects(left, right, top, bottom))
			return;
		if (queryRect.contains(curr->c))
			results.emplace_back(curr);
		if (isX) {
			_near(curr->left, queryRect, results, !isX, left, curr->c.first, top, bottom);
			_near(curr->right, queryRect, results, !isX, curr->c.first, right, top, bottom);
		}
		else {
			_near(curr->left, queryRect, results, !isX, left, right, top, curr->c.second);
			_near(curr->right, queryRect, results, !isX, left, right, curr->c.second, bottom);
		}
	}
};

class RRTStar : public RRT
{
public:
	RRTStar(int queryRad, int k, int goalBias, int steerRes = 1);
	Trajectory* findPath(float robotRad, sf::Vector2f robotPos, GuiManager& gui, sf::Vector2u windowSize, bool visualize = false) const;
private:
	int queryRad;
	std::pair<float, float> incrementCoord(std::pair<float, float> coord, float stepSize, float sin, float cos) const
	{
		return std::pair<float, float>(coord.first + cos * stepSize,
			coord.second + sin * stepSize);
	}

	std::vector<std::pair<int, int>> steer(RRTStarNode* from, std::pair<int, int>& to,
		std::pair<int, int>& end, int nodeSize, CSpace& cSpace, bool obsCheck = false) const {
		int dx = to.first - from->c.first;
		int dy = to.second - from->c.second;
		//int angle = ((int)((std::atan2(dy, dx) + 2 * PI) * 180 / PI)) % 360;

		float angle = std::atan2((float)dy, (float)dx) + 2 * (float)PI;
		float sinTheta = std::sin(angle);
		float cosTheta = std::cos(angle);
		float maxDist = (float)from->distSquaredTo(to);
		float dist = nodeSize * steerRes * ROOT2;
		float distTraveled = (float)nodeSize / 2;
		if (obsCheck || dist * dist > maxDist) {
			dist = std::sqrt(maxDist);
		}

		auto newCoords = std::pair<float, float>(from->c);
		auto results = std::vector<std::pair<int, int>>();
		results.reserve(10);
		results.push_back(newCoords);
		do {
			if (distTraveled > dist) distTraveled = dist;
			newCoords = incrementCoord(newCoords, distTraveled, sinTheta, cosTheta);
			// if (!cSpace.contains(newCoords)) break;
			results.emplace_back(std::pair<int, int>(newCoords));
			dist -= distTraveled;
		} while (dist >= FLT_EPSILON && (obsCheck || !cSpace.endReached(newCoords, end)));
		return results;
	}

	bool obstacleFree(std::vector<std::pair<int, int>>& steerPath, CSpace& cSpace) const {
		for (std::pair<int, int> coord : steerPath) {
			if (!cSpace.contains(coord)) return false;
		}
		return true;
	}
};