#pragma once
#include <SFML/Graphics.hpp>

static void expandRect(sf::FloatRect& rect, float rad)
{
	rect.left -= rad;
	rect.top -= rad;
	rect.width += 2 * rad;
	rect.height += 2 * rad;
}

class CustomShape {
public:
	virtual bool contains(sf::Vector2f) const = 0;
	virtual bool contains(sf::Vector2f, float robotRad) const = 0;
	virtual const sf::Vector2f& getPosition() const = 0;
	virtual void setPosition(float x, float y) = 0;
	virtual void setPosition(const sf::Vector2f& position) = 0;
	virtual void move(const sf::Vector2f& offset) = 0;
	virtual sf::FloatRect getLocalBounds() const = 0;
	virtual sf::FloatRect getGlobalBounds() const = 0;
	virtual sf::FloatRect getNewLocalBounds(float robotRad) const = 0;
	virtual sf::FloatRect getNewGlobalBounds(float robotRad) const = 0;
	virtual void setFillColor(const sf::Color& color) = 0;
	virtual const sf::Shape& getDrawable() = 0;
};

template <typename Shape>
class CenteredShape : public CustomShape
{
public:
	//virtual bool contains(sf::Vector2f) const = 0;
	Shape shape;
	const sf::Vector2f& getPosition() const override {
		return shape.getPosition();
	}
	void setPosition(float x, float y) override {
		shape.setPosition(x, y);
	}
	void setPosition(const sf::Vector2f& position) override {
		shape.setPosition(position);
	}
	void move(const sf::Vector2f& offset) override
	{
		shape.move(offset);
	}
	sf::FloatRect getLocalBounds() const override {
		return shape.getLocalBounds();
	}
	sf::FloatRect getGlobalBounds() const override {
		return shape.getGlobalBounds();
	}
	sf::FloatRect getNewLocalBounds(float robotRad) const override
	{
		auto oldBounds = getLocalBounds();
		expandRect(oldBounds, robotRad);
		return oldBounds;
	}
	sf::FloatRect getNewGlobalBounds(float robotRad) const override
	{
		auto oldBounds = getGlobalBounds();
		expandRect(oldBounds, robotRad);
		return oldBounds;
	}
	void setFillColor(const sf::Color& color) override {
		shape.setFillColor(color);
	}
};

class CenteredCircle : public CenteredShape<sf::CircleShape>//, public sf::CircleShape
{
public:
	CenteredCircle(float radius = 0, std::size_t pointCount = 30);
	bool contains(sf::Vector2f) const override;
	const sf::Shape& getDrawable() override;
	bool contains(sf::Vector2f, float robotRad) const override;
};

class CenteredRect : public CenteredShape<sf::RectangleShape>//, public sf::RectangleShape
{
public:
	CenteredRect(const sf::Vector2f& size = sf::Vector2f(0, 0));
	bool contains(sf::Vector2f) const override;
	const sf::Shape& getDrawable() override;
	bool contains(sf::Vector2f, float robotRad) const override;
};

class CenteredPolygon : public CenteredShape<sf::ConvexShape>//, sf::ConvexShape
{
	CenteredPolygon();
};