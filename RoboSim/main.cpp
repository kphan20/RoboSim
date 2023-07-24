#include "Robot.h"
#include "AStar.h"
#include "RRTStar.h"
#include <iostream>
#include <chrono>
#include <future>

#define DEBUG 0
#define VISUALIZE true

enum GameMode { placement, trajTesting, noInterrupt };

void handlePlacement(sf::RenderWindow& window, GuiManager& gui, sf::Event& event, GameMode* gameMode, Robot& robot, std::atomic_flag& pause)
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
				gui.addButton();
				break;
			case sf::Keyboard::Escape:
				window.close();
				break;
			case sf::Keyboard::W:
				gui.toggleButtons();
				*gameMode = trajTesting;
				pause.clear();
				robot.setTrajectory(nullptr);
				break;
			case sf::Keyboard::P:
				robot.setDestination(gui.getMousePos());
				break;
			default:
				break;
			}
		if (event.type == sf::Event::MouseButtonReleased)
			gui.handleInput();
	}

	gui.moveSelected();
}

void handletrajTesting(sf::RenderWindow& window, GuiManager& gui, sf::Event& event, GameMode* gameMode, Robot& robot, std::atomic_flag& pause)
{
	auto start = std::chrono::high_resolution_clock::now();
	auto finish = std::chrono::high_resolution_clock::now();
	robot.setTrajectory(nullptr);
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
				using milli = std::chrono::milliseconds;
				start = std::chrono::high_resolution_clock::now();
				robot.plan(gui, window.getSize(), VISUALIZE);
				finish = std::chrono::high_resolution_clock::now();
				std::cout << std::chrono::duration_cast<milli>(finish - start).count() << " milliseconds\n";
				gui.toggleButtons();
				*gameMode = noInterrupt;
				break;
			case sf::Keyboard::W:
				gui.toggleButtons();
				*gameMode = placement;
				pause.test_and_set();
				pause.notify_all();
				break;
			default:
				break;
			}
	}
}

void handleNoInterrupt(sf::RenderWindow& window, sf::Event& event, GameMode* gameMode, Robot& robot)
{
	while (window.pollEvent(event))
	{
		if (event.type == sf::Event::Closed || event.key.code == sf::Keyboard::Escape)
			window.close();
		else if (event.key.code == sf::Keyboard::W) {
			*gameMode = trajTesting;
		}
	}
	if (robot.trajectoryDone()) *gameMode = trajTesting;
}

int main()
{
	int windowWidth{ 500 }, windowLength{ 500 };
	sf::RenderWindow window(sf::VideoMode(windowWidth, windowLength), "SFML works!", sf::Style::Fullscreen);
	std::mutex m;
	std::atomic_flag flag;
	std::atomic_flag simEnded;
	flag.test_and_set();
	simEnded.clear();

	AStar plan;
	RRT plan1(50000, 30, 1);
	RRTStar plan2(3, 10000, 30, 5);
	Robot robot(plan, m, 75, 5);
	GuiManager gui(window, m, 5);

	auto f = std::async(std::launch::async, &Robot::schedulePlan, &robot, std::ref(gui), window.getSize(), std::ref(flag), std::ref(simEnded));

	std::cout << window.getSize().x << ',' << window.getSize().y << '\n';

	// TODO possibly remove this
	window.setKeyRepeatEnabled(false);

	sf::Clock clock;

	int updatesPerSec = 30;
	int deadlineMs = (int)(1000 / updatesPerSec);

	int nextUpdate = clock.getElapsedTime().asMilliseconds();
	int prevUpdate = nextUpdate;

	GameMode gameMode = placement;

	int planTimer = 0;
	int prevTime = clock.getElapsedTime().asMilliseconds();
	while (window.isOpen())
	{
		int currTime = clock.getElapsedTime().asMilliseconds();

		planTimer += currTime - prevTime;
		prevTime = currTime;
		if (planTimer > 100) {
			planTimer = 0;
		}

		if (nextUpdate < currTime) {
			//if (gameMode == placement)
			flag.notify_all();
			robot.update(currTime - prevUpdate);
			prevUpdate = currTime;
			nextUpdate += deadlineMs;
		}

		sf::Event event;

		switch (gameMode) {
		case placement:
			handlePlacement(window, gui, event, &gameMode, robot, flag);
			break;
		case trajTesting:
			handletrajTesting(window, gui, event, &gameMode, robot, flag);
			break;
		case noInterrupt:
			handleNoInterrupt(window, event, &gameMode, robot);
			break;
		default:
			break;
		}

		window.clear();

		gui.draw();
		window.draw(robot.getDrawable());

		window.display();
	}
	//f.get();
	simEnded.test_and_set();
	flag.test_and_set();
	flag.notify_all();
	return 0;
}