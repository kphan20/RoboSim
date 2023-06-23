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
typedef std::vector<sf::CircleShape*> VisualizerList;


class GuiManager
{
public:
	GuiManager(sf::RenderWindow&, int sizePerNode = 1);
	GuiManager(const GuiManager&) = delete;
	Robot robot;
	const int nodeSize;
	void addCircle();
	void addButton();
	void draw();
	void scroll();
	void handleInput();
	void moveSelected();
	void addTrajPoint();
	Node getMousePos();
	const ShapeList* getShapes();
	// pathfinding api
	void addNode(std::pair<int, int> coords, sf::Color);
	void drawNodes();
	void freeNodes();

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
	VisualizerList nodes;
};