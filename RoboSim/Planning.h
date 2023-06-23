#pragma once
#include "Trajectory.h"
#include "GuiManager.h"
#include <SFML/Graphics.hpp>

class Planner
{
public:
	virtual Trajectory* findPath(float robotRad, sf::Vector2f robotPos, GuiManager& gui, sf::Vector2u windowSize, bool visualize = false) const = 0;
};