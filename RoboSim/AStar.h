#pragma once
#include "Planning.h"
#include <SFML/Graphics.hpp>
#include <tuple>
#include <unordered_map>

class AStarNode
{
public:
	AStarNode(sf::Vector2i& pos, int g = 0, int h = 0);
	AStarNode(int x, int y, AStarNode* parent, int g = 0, int h = 0);
	AStarNode(sf::Vector2i&, AStarNode* parent, int g = 0, int h = 0);
	AStarNode* parent;
	bool operator<(const AStarNode& other) const {
		if (x == other.x && y == other.y) return false;
		const int f = g + h;
		const int otherF = other.g + other.h;
		return std::tie(f, h, x, y) < std::tie(otherF, other.h, other.x, other.y);
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
	int f() {
		return g + h;
	}
	const std::pair<int, int> coords() const
	{
		return std::pair<int, int>(x, y);
	}
};

struct hash_pair {
	template <class T1, class T2>
	size_t operator()(const std::pair<T1, T2>& p) const
	{
		return std::hash<T1>{}(p.first * 2000 + p.second);
	}
};

class OpenList
{
public:
	OpenList(int nodeSize, int xR, int yR, sf::Vector2u);
	void insert(AStarNode*);
	AStarNode* pop();
	bool empty();
	void cleanUp();
	std::pair<int, int> convertCoords(const std::pair<int, int>&);
private:
	const int nodeSize, xRem, yRem;
	const sf::Vector2u windowSize;
	bool less(size_t, size_t);
	void swap(size_t, size_t);
	void update(size_t, const AStarNode*);
	void siftUp(size_t);
	void siftDown(size_t);
	std::vector<AStarNode*> heap;
	std::unordered_map<std::pair<int, int>, size_t, hash_pair> indexMapping;
	std::unordered_map<std::pair<int, int>, int, hash_pair> gValues;
};

class AStar : public Planner
{
public:
	Trajectory* findPath(float robotRad, sf::Vector2f robotPos, GuiManager& gui, sf::Vector2u windowSize, bool visualize = false) const;
private:
	Trajectory* constructPath(const AStarNode&, AStarNode&) const;
};