#include "GuiManager.h"
#include <iostream>

GuiManager::GuiManager(sf::RenderWindow& win, int sizePerNode) : window(win), nodeSize(sizePerNode)
{
	currShapeIdx = 0;
	currShapeIdxPtr = &currShapeIdx;
}

void GuiManager::addCircle()
{
	auto ptr1 = std::make_unique<CenteredCircle>(75.0f);
	ptr1->setFillColor(sf::Color::Yellow);
	moveableShapes.emplace_back(std::move(ptr1));
}

void GuiManager::addButton()
{
	auto dims = window.getSize();
	auto newButton = std::make_unique<Button>(window, sf::Vector2f(100, 100));
	newButton->setFillColor(sf::Color::Yellow);
	// for now the only gui objects are buttons
	if (++numButtons >= guiObjects.size()) {
		size_t numButtonsCopy = numButtons - 1;
		auto newAction = std::make_shared<ClickAction>([this, numButtonsCopy](size_t val) -> void {
			std::cout << numButtonsCopy << '\n';
		*currShapeIdxPtr = numButtonsCopy;
		return;
			});
		actions.emplace_back(newAction);
	}
	std::shared_ptr<ClickAction> action = { actions[numButtons - 1] };
	newButton->setAction(action);
	size_t actionIdx = guiObjects.size();
	newButton->setPosition(sf::Vector2f(std::rand() % dims.x, std::rand() % dims.y));
	std::cout << newButton->getPosition().x << ',' << newButton->getPosition().y << '\n';
	guiObjects.emplace_back(std::move(newButton));
}

void GuiManager::handleInput()
{
	for (const std::unique_ptr<Button>& guiObject : guiObjects)
		guiObject->onClick();
	auto pos = sf::Vector2f(sf::Mouse::getPosition(window));
	std::cout << pos.x << ',' << pos.y << '\n';
}

void GuiManager::scroll()
{
	if (!moveableShapes.size()) return;
	currShapeIdx = (++currShapeIdx) % moveableShapes.size();
}

void GuiManager::draw()
{
	for (const std::unique_ptr<CustomShape>& moveableShape : moveableShapes)
		window.draw(moveableShape->getDrawable());
	for (const std::unique_ptr<Button>& guiObject : guiObjects)
		window.draw(guiObject->shape);
}

void GuiManager::moveSelected()
{
	if (sf::Mouse::isButtonPressed(sf::Mouse::Left) && moveableShapes.size() > 0)
		moveableShapes[currShapeIdx]->setPosition(sf::Vector2f(sf::Mouse::getPosition(window)));
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
	auto ptr1 = new sf::CircleShape(nodeSize);
	ptr1->setFillColor(color);
	ptr1->setPosition(coords.first, coords.second);
	nodes.emplace_back(ptr1);
}

void GuiManager::drawNodes()
{
	for (const sf::CircleShape* node : nodes)
		window.draw(*node);
	window.display();
}

void GuiManager::freeNodes()
{
	for (auto p : nodes)
		delete p;
	nodes.clear();
}
