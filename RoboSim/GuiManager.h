#pragma once
#include "Button.h"
#include "Trajectory.h"
#include <vector>
#include <memory>
#include <mutex>
#include <SFML/Graphics.hpp>

typedef std::vector<std::unique_ptr<CustomShape>> ShapeList;
typedef std::vector<std::unique_ptr<Button>> GuiList;
typedef std::vector<std::shared_ptr<ClickAction>> ActionList;
typedef std::vector<CenteredCircle*> VisualizerList;


class GuiManager
{
public:
	GuiManager(sf::RenderWindow&, std::mutex&, int sizePerNode = 1);
	GuiManager(const GuiManager&) = delete;
	const int nodeSize;
	void addCircle();
	void addButton();
	void draw();
	void scroll();
	void handleInput();
	void moveSelected();
	Node getMousePos();
	const ShapeList* getShapes();
	void toggleButtons();
	bool shapesChanged();

	// pathfinding api
	void addNode(std::pair<int, int> coords, sf::Color);
	void addEdge(sf::Vertex&, sf::Vertex&);
	void drawNodes();
	void freeNodes();

private:
	size_t currShapeIdx;
	size_t* currShapeIdxPtr;
	size_t numButtons = 0;
	sf::RenderWindow& window;
	ShapeList moveableShapes{};
	GuiList guiObjects{};
	ActionList actions{};
	VisualizerList nodes;
	std::vector<sf::Vertex> edges;
	bool buttonsVisible;
	Button dropDownToggle;
	CenteredCircle dropDownTri;
	std::mutex& m;
	bool recentlyChanged = true;
};