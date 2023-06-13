#include "GuiManager.h"
#include <iostream>

GuiManager::GuiManager(sf::RenderWindow& win) : window(win)
{
	currShapeIdx = 0;
	currShapeIdxPtr = &currShapeIdx;
	robot = Robot(75);
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
	if (numButtons++ == guiObjects.size()) {
		size_t numButtonsCopy = numButtons;
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
	for (const std::unique_ptr<sf::Shape>& moveableShape : moveableShapes)
		window.draw(*moveableShape);
	for (const std::unique_ptr<Button>& guiObject : guiObjects)
		window.draw(*guiObject);

	window.draw(robot);
}

void GuiManager::moveSelected()
{
	if (sf::Mouse::isButtonPressed(sf::Mouse::Left) && moveableShapes.size() > 0)
		moveableShapes[currShapeIdx]->setPosition(sf::Vector2f(sf::Mouse::getPosition(window)));
}

void GuiManager::addTrajPoint()
{
	robot.addToCurrTrajectory(sf::Vector2f(sf::Mouse::getPosition(window)));
}

Node GuiManager::getMousePos()
{
	return Node(sf::Mouse::getPosition(window));
}

void GuiManager::update(int elapsedTime)
{
	robot.followTrajectory(elapsedTime);
}