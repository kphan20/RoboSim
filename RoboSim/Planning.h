#pragma once
#include "Trajectory.h"
#include "GuiManager.h"
#include <SFML/Graphics.hpp>

class Planner
{
public:
	virtual Trajectory* findPath(sf::Vector2f robotPos, int nodeSize, Node end, GuiManager& gui, sf::Vector2u windowSize, bool visualize = false) const = 0;
	virtual void setUp(float, sf::Vector2f, GuiManager&, sf::Vector2u) = 0;
};