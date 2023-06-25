#pragma once
#include "GuiManager.h"

class CSpace
{
public:
	CSpace(float robotRad, sf::Vector2f robotPos, GuiManager& gui, sf::Vector2u windowSize) :
		nodeSize(gui.nodeSize), xRemainder(windowSize.x% nodeSize), yRemainder(windowSize.y% gui.nodeSize),
		windowX(windowSize.x), windowY(windowSize.y), xMax(windowSize.x / gui.nodeSize + (xRemainder > 0)),
		yMax(windowSize.y / gui.nodeSize + (yRemainder > 0))
	{
		grid = new bool* [xMax];
		for (int i = 0; i < xMax; i++)
		{
			grid[i] = new bool[yMax];
		}

		auto obstacles = gui.getShapes();
		for (auto& shape : *obstacles)
		{
			sf::IntRect bounds(shape->getNewGlobalBounds(robotRad));
			for (int i = 0; i < bounds.width; i++) {
				for (int j = 0; j < bounds.height; j++) {
					int xCoord = bounds.left + i;
					int yCoord = bounds.top + j;
					int adjX = xCoord / gui.nodeSize;
					if (xCoord >= windowSize.x - xRemainder) adjX++;
					int adjY = yCoord / gui.nodeSize;
					if (yCoord >= windowSize.y - yRemainder) adjY++;
					if (xCoord < 0 || xCoord >= windowSize.x
						|| yCoord < 0 || yCoord >= windowSize.y || !grid[adjX][adjY])
						continue;
					grid[adjX][adjY] = !shape->contains(sf::Vector2f(xCoord, yCoord), robotRad);
				}
			}
		}
	}
	virtual void cleanUp()
	{
		for (int i = 0; i < xMax; i++)
			delete[] grid[i];
		delete[] grid;
	}

	bool contains(const std::pair<int, int>& coords)
	{
		auto adjCoords = convertCoords(coords);
		if (adjCoords.first < 0 || adjCoords.first >= xMax ||
			adjCoords.second < 0 || adjCoords.second >= yMax)
			return false;
		return grid[adjCoords.first][adjCoords.second];
	}

	bool contains(const int x, const int y)
	{
		return contains(std::pair<int, int>(x, y));
	}

	bool endReached(const std::pair<int, int>& coords, const std::pair<int, int>& end)
	{
		return convertCoords(coords) == convertCoords(end);
	}
protected:
	bool** grid;
	const int nodeSize, windowX, windowY, xMax, yMax;
	const bool xRemainder, yRemainder;
	std::pair<int, int> convertCoords(const std::pair<int, int>& coords)
	{
		const int x = coords.first / nodeSize + (coords.first >= windowX - xRemainder ? 1 : 0);
		const int y = coords.second / nodeSize + (coords.second >= windowY - yRemainder ? 1 : 0);
		return std::pair<int, int>(x, y);
	}
};