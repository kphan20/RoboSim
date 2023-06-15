#pragma once
#include <vector>
#include <memory>
#include <SFML/Graphics.hpp>
#include "Button.h"
#include "Robot.h"
#include "CenteredShape.h"

typedef std::vector<std::unique_ptr<CustomShape>> ShapeList;
typedef std::vector<std::unique_ptr<Button>> GuiList;
typedef std::vector<std::shared_ptr<ClickAction>> ActionList;


class GuiManager
{
public:
	GuiManager(sf::RenderWindow&);
	GuiManager(const GuiManager&) = delete;
	Robot robot;
	void addCircle();
	void addButton();
	void draw();
	void scroll();
	void handleInput();
	void moveSelected();
	void addTrajPoint();
	Node getMousePos();
	const ShapeList* getShapes();

	// game logic
	void update(int);
private:
	size_t currShapeIdx;
	size_t* currShapeIdxPtr;
	size_t numButtons = 0;
	sf::RenderWindow& window;
	ShapeList moveableShapes{};
	GuiList guiObjects{};
	ActionList actions{};
};