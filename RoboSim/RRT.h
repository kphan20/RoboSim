#pragma once
#include "Planning.h"
#include "CSpace.h"
#include <random>

class RRTCSpace : public CSpace
{
public:
	RRTCSpace(float robotRad, sf::Vector2f robotPos, GuiManager& gui, sf::Vector2u windowSize) :
		CSpace(robotRad, robotPos, gui, windowSize)
	{
		for (int x = 0; x < xMax; x++)
		{
			for (int y = 0; y < yMax; y++)
			{
				if (!grid[x][y]) continue;
				randCoords.emplace_back(new std::pair<int, int>(x * nodeSize, y * nodeSize));
			}
		}
		auto rng = std::default_random_engine{};
		std::shuffle(std::begin(randCoords), std::end(randCoords), rng);
		bufferInd = 0;
	}
	const std::pair<int, int> sample()
	{
		const std::pair<int, int> result = *randCoords[bufferInd];
		bufferInd = (bufferInd + 1) % randCoords.size();
		return result;
	}
	void cleanUp() override {
		for (auto coord : randCoords)
			delete coord;
		CSpace::cleanUp();
	}

	std::pair<int, int> maxDims() {
		return std::pair<int, int>(windowX, windowY);
	}
private:
	size_t bufferInd;
	std::vector<std::pair<int, int>*> randCoords;
};

class KDTreeNode {
public:
	KDTreeNode* left;
	KDTreeNode* right;
	const KDTreeNode* parent;
	const std::pair<int, int> c;
	KDTreeNode(const std::pair<int, int>& coords, const KDTreeNode* parent = nullptr) :
		c(coords), parent(parent) {
		left = nullptr;
		right = nullptr;
	}
	bool lessThan(std::pair<int, int> other, bool isX)
	{
		return isX ? c.first < other.first : c.second < other.second;
	}
	int distSquaredTo(const std::pair<int, int>& other)
	{
		const int dx = other.first - c.first;
		const int dy = other.second - c.second;
		return dx * dx + dy * dy;
	}

	const Node& getPathNode() const
	{
		return sf::Vector2f(c.first, c.second);
	}
};

class KDRect
{
public:
	KDRect() {};
	void Init(int left, int right, int top, int bottom)
	{
		l = left;
		r = right;
		t = top;
		b = bottom;
	}
	int distSquaredTo(const std::pair<int, int>& coords)
	{
		const int dx = std::max({ l - coords.first, 0, coords.first - r });
		const int dy = std::max({ t - coords.second, 0, coords.second - b });
		return dx * dx + dy * dy;
	}
private:
	int l, r, t, b;
};

class KDTree
{
public:
	KDTree(const std::pair<int, int>& start, std::pair<int, int> maxDims) :
		xMax(maxDims.first), yMax(maxDims.second)
	{
		root = new KDTreeNode(start);
	}
	bool put(const std::pair<int, int>& coords, const KDTreeNode* parent) {
		bool isX = true;
		KDTreeNode* curr = root;
		while (curr != nullptr)
		{
			if (curr->c == coords) {
				return false;
			}
			if (curr->lessThan(coords, isX))
			{
				if (curr->left == nullptr) {
					curr->left = new KDTreeNode(coords, parent);
					return true;
				}
				curr = curr->left;
			}
			else
			{
				if (curr->right == nullptr) {
					curr->right = new KDTreeNode(coords, parent);
					return true;
				}
				curr = curr->right;
			}
			isX = !isX;
		}
		return false;
	}
	const KDTreeNode* nearest(const std::pair<int, int>& coords) {
		KDTreeNode* champ = root;
		champ = _nearest(root, coords, champ, true, 0, xMax, 0, yMax);
		return champ;
	}
	void cleanUp()
	{
		_cleanUp(root);
	}
private:
	KDTreeNode* root;
	const int xMax, yMax;
	KDTreeNode* _nearest(KDTreeNode* curr, const std::pair<int, int>& coords, KDTreeNode* champ, bool isX,
		int left, int right, int top, int bottom)
	{
		int champDist = champ->distSquaredTo(coords);
		KDRect leftRect, rightRect;
		if (isX)
		{
			leftRect.Init(left, curr->c.first, top, bottom);
			rightRect.Init(curr->c.first, right, top, bottom);
		}
		else
		{
			leftRect.Init(left, right, top, curr->c.second);
			rightRect.Init(left, right, curr->c.second, bottom);
		}

		int checkDist = curr->distSquaredTo(coords);
		if (checkDist < champDist) {
			champDist = checkDist;
			champ = curr;
		}

		if (curr->lessThan(coords, isX))
		{
			if (curr->left != nullptr && leftRect.distSquaredTo(coords) < champDist) {
				champ = _nearest(curr->left, coords, champ, !isX, left, right, top, bottom);
				champDist = champ->distSquaredTo(coords);
			}
			if (curr->right != nullptr && rightRect.distSquaredTo(coords) < champDist)
			{
				champ = _nearest(curr->right, coords, champ, !isX, left, right, top, bottom);
			}
		}
		else
		{
			if (curr->right != nullptr && rightRect.distSquaredTo(coords) < champDist)
			{
				champ = _nearest(curr->right, coords, champ, !isX, left, right, top, bottom);
				champDist = champ->distSquaredTo(coords);
			}
			if (curr->left != nullptr && leftRect.distSquaredTo(coords) < champDist) {
				champ = _nearest(curr->left, coords, champ, !isX, left, right, top, bottom);
			}
		}
		return champ;
	}
	void _cleanUp(KDTreeNode* curr)
	{
		if (curr == nullptr) return;
		_cleanUp(curr->left);
		_cleanUp(curr->right);
		delete curr;
	}
};


class RRT : public Planner
{
public:
	RRT(int k, int goalBias, int steerRes = 1);
	Trajectory* findPath(float robotRad, sf::Vector2f robotPos, GuiManager& gui, sf::Vector2u windowSize, bool visualize = false) const;
private:
	int k, goalBias, steerRes;
	std::pair<int, int> incrementCoord(std::pair<int, int> coord, int stepSize, std::pair<int, int> direction) const
	{
		return std::pair<int, int>(coord.first + direction.first * stepSize,
			coord.second + direction.second * stepSize);
	}
};