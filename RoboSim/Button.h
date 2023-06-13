#pragma once
#include "CenteredShape.h"
#include <functional>

typedef std::function<void(size_t)> ClickAction;

class Button : public CenteredRect
{
public:
	Button(const sf::RenderWindow& window, const sf::Vector2f& size = sf::Vector2f(0, 0));
	void setAction(std::shared_ptr<ClickAction>&);
	void onClick();
private:
	std::shared_ptr<ClickAction> onClickAction;
	const sf::RenderWindow& window;
};