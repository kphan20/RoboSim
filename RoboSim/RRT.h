#pragma once
#include "Planning.h"
#include "CSpace.h"
#include <random>

#ifndef PI
#define PI 3.1415
#endif

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
protected:
	size_t bufferInd;
	std::vector<std::pair<int, int>*> randCoords;
};

class KDTreeNode {
public:
	KDTreeNode* left;
	KDTreeNode* right;
	KDTreeNode* parent;
	const std::pair<int, int> c;
	KDTreeNode(const std::pair<int, int>& coords, KDTreeNode* parent = nullptr) :
		c(coords), parent(parent) {
		left = nullptr;
		right = nullptr;
	}
	bool lessThan(std::pair<int, int> other, bool isX)
	{
		return isX ? c.first >= other.first : c.second >= other.second;
	}
	int distSquaredTo(const std::pair<int, int>& other)
	{
		const int dx = other.first - c.first;
		const int dy = other.second - c.second;
		return dx * dx + dy * dy;
	}

	const Node& getPathNode() const
	{
		return sf::Vector2f((float)c.first, (float)c.second);
	}

	void cleanUp() {}
};

class KDRect
{
public:
	KDRect() {
		l = r = t = b = 0;
	};
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
	int l, r, t, b;
};

template <typename TreeNode>
class KDTree
{
public:
	TreeNode* recentPutNode = nullptr;
	KDTree(const std::pair<int, int>& start, std::pair<int, int> maxDims) :
		xMax(maxDims.first), yMax(maxDims.second)
	{
		root = new TreeNode(start);
	}
	virtual bool put(const std::pair<int, int>& coords, TreeNode* parent) {
		bool isX = true;
		TreeNode* curr = root;
		while (curr != nullptr)
		{
			if (curr->c == coords) {
				recentPutNode = curr;
				return false;
			}
			if (curr->lessThan(coords, isX))
			{
				if (curr->left == nullptr) {
					curr->left = new TreeNode(coords, parent);
					recentPutNode = curr->left;
					return true;
				}
				curr = curr->left;
			}
			else
			{
				if (curr->right == nullptr) {
					curr->right = new TreeNode(coords, parent);
					recentPutNode = curr->right;
					return true;
				}
				curr = curr->right;
			}
			isX = !isX;
		}
		return false;
	}
	TreeNode* nearest(const std::pair<int, int>& coords) {
		TreeNode* champ = root;
		champ = _nearest(root, coords, champ, true, 0, xMax, 0, yMax);
		return champ;
	}
	void cleanUp()
	{
		_cleanUp(root);
	}
protected:
	TreeNode* root;
	const int xMax, yMax;
	TreeNode* _nearest(TreeNode* curr, const std::pair<int, int>& coords, TreeNode* champ, bool isX,
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
				champ = _nearest(curr->left, coords, champ, !isX, leftRect.l, leftRect.r, leftRect.t, leftRect.b);
				champDist = champ->distSquaredTo(coords);
			}
			if (curr->right != nullptr && rightRect.distSquaredTo(coords) < champDist)
			{
				champ = _nearest(curr->right, coords, champ, !isX, rightRect.l, rightRect.r, rightRect.t, rightRect.b);
			}
		}
		else
		{
			if (curr->right != nullptr && rightRect.distSquaredTo(coords) < champDist)
			{
				champ = _nearest(curr->right, coords, champ, !isX, rightRect.l, rightRect.r, rightRect.t, rightRect.b);
				champDist = champ->distSquaredTo(coords);
			}
			if (curr->left != nullptr && leftRect.distSquaredTo(coords) < champDist) {
				champ = _nearest(curr->left, coords, champ, !isX, leftRect.l, leftRect.r, leftRect.t, leftRect.b);
			}
		}
		return champ;
	}
	void _cleanUp(TreeNode* curr)
	{
		if (curr == nullptr) return;
		_cleanUp(curr->left);
		_cleanUp(curr->right);
		curr->cleanUp();
		delete curr;
	}
};


class RRT : public Planner
{
public:
	RRT(int k, int goalBias, int steerRes = 1);
	virtual Trajectory* findPath(float robotRad, sf::Vector2f robotPos, GuiManager& gui, sf::Vector2u windowSize, bool visualize = false) const;
protected:
	int k, goalBias, steerRes;
	std::pair<int, int> incrementCoord(std::pair<int, int> coord, int stepSize, std::pair<int, int> direction) const
	{
		return std::pair<int, int>(coord.first + direction.first * stepSize,
			coord.second + direction.second * stepSize);
	}
};