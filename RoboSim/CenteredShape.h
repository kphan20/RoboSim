#pragma once
#include <SFML/Graphics.hpp>

class CenteredShape
{
public:
	virtual bool contains(sf::Vector2f) const = 0;
};

class CenteredCircle : public CenteredShape, public sf::CircleShape
{
public:
	CenteredCircle(float radius = 0, std::size_t pointCount = 30);
	bool contains(sf::Vector2f) const override;
};

class CenteredRect : public CenteredShape, public sf::RectangleShape
{
public:
	CenteredRect(const sf::Vector2f& size = sf::Vector2f(0, 0));
	bool contains(sf::Vector2f) const override;
};

class CenteredPolygon : public CenteredShape, sf::ConvexShape
{
	CenteredPolygon();
};