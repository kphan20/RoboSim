#include "Robot.h"
#include "AStar.h"
#include <iostream>
#include <chrono>

#define DEBUG 0

enum GameMode { placement, trajTesting };

void handlePlacement(sf::RenderWindow& window, GuiManager& gui, sf::Event& event, GameMode* gameMode)
{
	while (window.pollEvent(event))
	{
		if (event.type == sf::Event::Closed)
			window.close();
		if (event.type == sf::Event::KeyPressed)
			switch (event.key.code) {
			case sf::Keyboard::Space:
				gui.scroll();
				break;
			case sf::Keyboard::N:
				gui.addCircle();
				break;
			case sf::Keyboard::A:
				gui.addButton();
				break;
			case sf::Keyboard::Escape:
				window.close();
				break;
			case sf::Keyboard::W:
				*gameMode = trajTesting;
				break;
			default:
				break;
			}
		if (event.type == sf::Event::MouseButtonReleased)
			gui.handleInput();
	}

	gui.moveSelected();
}

void handletrajTesting(sf::RenderWindow& window, GuiManager& gui, sf::Event& event, GameMode* gameMode, Robot& robot)
{
	auto start = std::chrono::high_resolution_clock::now();
	auto finish = std::chrono::high_resolution_clock::now();
	while (window.pollEvent(event))
	{
		if (event.type == sf::Event::Closed)
			window.close();
		if (event.type == sf::Event::KeyPressed)
			switch (event.key.code) {
			case sf::Keyboard::Escape:
				window.close();
				break;
			case sf::Keyboard::P:
				*gameMode = placement;
				using milli = std::chrono::milliseconds;
				start = std::chrono::high_resolution_clock::now();
				robot.plan(gui, window.getSize(), true);
				finish = std::chrono::high_resolution_clock::now();
				std::cout << std::chrono::duration_cast<milli>(finish - start).count() << " milliseconds\n";
				break;
			case sf::Keyboard::W:
				*gameMode = placement;
				break;
			default:
				break;
			}
		if (event.type == sf::Event::MouseButtonReleased)
			robot.addToCurrTrajectory(gui.getMousePos());
	}
}

int main()
{
	int windowWidth{ 500 }, windowLength{ 500 };
	sf::RenderWindow window(sf::VideoMode(windowWidth, windowLength), "SFML works!", sf::Style::Fullscreen);

	AStar plan;
	Robot robot(plan, 75);

	GuiManager gui(window, 4);

	std::cout << window.getSize().x << ',' << window.getSize().y << '\n';

	// TODO possibly remove this
	window.setKeyRepeatEnabled(false);

	sf::Clock clock;

	int updatesPerSec = 30;
	int deadlineMs = (int)(1000 / updatesPerSec);

	int nextUpdate = clock.getElapsedTime().asMilliseconds();
	int prevUpdate = nextUpdate;

	GameMode gameMode = placement;

	while (window.isOpen())
	{
		int currTime = clock.getElapsedTime().asMilliseconds();

		if (nextUpdate < currTime) {
			if (gameMode == placement)
				robot.update(currTime - prevUpdate);
			prevUpdate = currTime;
			nextUpdate += deadlineMs;
		}

		sf::Event event;

		if (gameMode == placement)
			handlePlacement(window, gui, event, &gameMode);
		else if (gameMode == trajTesting)
			handletrajTesting(window, gui, event, &gameMode, robot);

		window.clear();

		gui.draw();
		window.draw(robot.getDrawable());

		window.display();
	}

	return 0;
}