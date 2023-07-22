#include "GuiManager.h"
#include <iostream>

const float SELECT_OUTLINE_THICKNESS = 5.0;
const sf::Vector2f TOGGLE_BUTTON_SIZE = sf::Vector2f(50.0f, 50.0f);
const sf::Vector2f BUTTON_SIZE = sf::Vector2f(100.0f, 100.0f);
const float BUTTON_DIST = 5.0f;
const int TRIANGLE_SIZE = 20;
const int RIGHT_OFFSET = 100;
const float TOP_OFFSET = 50.0f;

GuiManager::GuiManager(sf::RenderWindow& win, std::mutex& lock, int sizePerNode) :
	window(win), nodeSize(sizePerNode), dropDownToggle(win, TOGGLE_BUTTON_SIZE),
	dropDownTri(TRIANGLE_SIZE, 3), m(lock)
{
	currShapeIdx = 0;
	currShapeIdxPtr = &currShapeIdx;
	buttonsVisible = true;
	dropDownToggle.setPosition(sf::Vector2f((float)(win.getSize().x - RIGHT_OFFSET), TOP_OFFSET));
	dropDownTri.setPosition(dropDownToggle.getPosition());
	dropDownToggle.setFillColor(sf::Color::Blue);
	dropDownTri.setFillColor(sf::Color::White);
	auto toggle = std::make_shared<ClickAction>([this](size_t filler)-> void {
		toggleButtons();
		}
	);
	dropDownToggle.setAction(toggle);
}

void GuiManager::addCircle()
{
	auto ptr1 = std::make_unique<CenteredCircle>(75.0f);
	ptr1->setFillColor(sf::Color::Yellow);
	ptr1->shape.setOutlineColor(sf::Color::White);
	std::lock_guard<std::mutex> lock(m);
	if (moveableShapes.size() == 0) ptr1->setOutlineThickness(SELECT_OUTLINE_THICKNESS);
	moveableShapes.emplace_back(std::move(ptr1));
}

void GuiManager::addButton()
{
	auto dims = window.getSize();
	auto newButton = std::make_unique<Button>(window, BUTTON_SIZE);
	newButton->setFillColor(sf::Color::Yellow);
	// for now the only gui objects are buttons
	if (++numButtons >= guiObjects.size()) {
		size_t numButtonsCopy = numButtons - 1;
		auto newAction = std::make_shared<ClickAction>([this, numButtonsCopy](size_t val) -> void {
			std::lock_guard<std::mutex> lock(m);
		moveableShapes[currShapeIdx]->setOutlineThickness(0.0);
		currShapeIdx = numButtonsCopy;
		moveableShapes[currShapeIdx]->setOutlineThickness(SELECT_OUTLINE_THICKNESS);
		return;
			});
		actions.emplace_back(newAction);
	}
	std::shared_ptr<ClickAction> action = { actions[numButtons - 1] };
	newButton->setAction(action);
	sf::Vector2f togglePos = dropDownToggle.getPosition();
	newButton->setPosition(togglePos.x, togglePos.y + (guiObjects.size() + 1) * (BUTTON_SIZE.y + BUTTON_DIST));
	guiObjects.emplace_back(std::move(newButton));
}

void GuiManager::toggleButtons() {
	float thickness = 0.0;
	buttonsVisible = !buttonsVisible;
	if (buttonsVisible) thickness = SELECT_OUTLINE_THICKNESS;
	for (const std::unique_ptr<Button>& guiObject : guiObjects)
		guiObject->isVisible = buttonsVisible;
	dropDownTri.rotate(180);
	std::lock_guard<std::mutex> lock(m);
	if (moveableShapes.size() > 0)
		moveableShapes[currShapeIdx]->setOutlineThickness(thickness);
}

void GuiManager::handleInput()
{
	for (const std::unique_ptr<Button>& guiObject : guiObjects)
		guiObject->onClick();
	dropDownToggle.onClick();
	auto pos = sf::Vector2f(sf::Mouse::getPosition(window));
}

void GuiManager::scroll()
{
	if (!moveableShapes.size()) return;
	currShapeIdx = (++currShapeIdx) % moveableShapes.size();
}

void GuiManager::draw()
{
	{
		std::lock_guard<std::mutex> lock(m);
		for (const std::unique_ptr<CustomShape>& moveableShape : moveableShapes)
			window.draw(moveableShape->getDrawable());
	}
	window.draw(dropDownToggle.getDrawable());
	window.draw(dropDownTri.getDrawable());
	if (buttonsVisible) {
		for (const std::unique_ptr<Button>& guiObject : guiObjects)
			window.draw(guiObject->shape);
	}
}

void GuiManager::moveSelected()
{
	if (sf::Mouse::isButtonPressed(sf::Mouse::Left) && moveableShapes.size() > 0) {
		Node mousePos = getMousePos();
		if (buttonsVisible)
			for (const std::unique_ptr<Button>& guiObject : guiObjects)
				if (guiObject->contains(mousePos)) return;
		if (dropDownToggle.contains(mousePos)) return;
		std::lock_guard<std::mutex> lock(m);
		moveableShapes[currShapeIdx]->setPosition(getMousePos());
	}
}

Node GuiManager::getMousePos()
{
	return Node(sf::Mouse::getPosition(window));
}

const ShapeList* GuiManager::getShapes()
{
	return &moveableShapes;
}

void GuiManager::addNode(std::pair<int, int> coords, sf::Color color)
{
	auto ptr1 = new CenteredCircle((float)nodeSize);
	ptr1->setFillColor(color);
	ptr1->setPosition((float)coords.first, (float)coords.second);
	nodes.emplace_back(ptr1);
}

void GuiManager::addEdge(sf::Vertex& from, sf::Vertex& to) {
	edges.emplace_back(from);
	edges.emplace_back(to);
}

void GuiManager::drawNodes()
{
	for (CenteredCircle* node : nodes)
		window.draw(node->getDrawable());
	if (edges.size() > 0)
		window.draw(&edges[0], edges.size(), sf::Lines);
	window.display();
}

void GuiManager::freeNodes()
{
	for (auto p : nodes)
		delete p;
	nodes.clear();
	edges.clear();
}
