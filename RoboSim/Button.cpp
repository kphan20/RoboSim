#include "Button.h"

Button::Button(const sf::RenderWindow& win, const sf::Vector2f& size) : CenteredRect(size), window(win)
{
	isVisible = true;
}
void Button::setAction(std::shared_ptr<ClickAction>& func)
{
	onClickAction = func;
}

void Button::onClick() {
	// this is for hovering
	if (!isVisible || !this->contains(sf::Vector2f(sf::Mouse::getPosition(this->window)))) {
		return;
	}
	(*onClickAction)(6);
}