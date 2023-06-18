#include "CenteredShape.h"

CenteredCircle::CenteredCircle(float radius, size_t pointCount)
{
	shape = sf::CircleShape(radius, pointCount);
	sf::FloatRect bounds{ this->getLocalBounds() };
	shape.setOrigin(bounds.width / 2, bounds.height / 2);
}

bool CenteredCircle::contains(sf::Vector2f point) const
{
	sf::Vector2f diff = this->getPosition() - point;
	float rad = shape.getRadius();
	return diff.x * diff.x + diff.y * diff.y <= rad * rad;
}

const sf::Shape& CenteredCircle::getDrawable() {
	return shape;
}

bool CenteredCircle::contains(sf::Vector2f point, float robotRad) const
{
	sf::Vector2f diff = this->getPosition() - point;
	float rad = shape.getRadius() + robotRad;
	return diff.x * diff.x + diff.y * diff.y <= rad * rad;
}

CenteredRect::CenteredRect(const sf::Vector2f& size)
{
	shape = sf::RectangleShape(size);
	sf::FloatRect bounds{ this->getLocalBounds() };
	shape.setOrigin(bounds.width / 2, bounds.height / 2);
}

bool CenteredRect::contains(sf::Vector2f point) const {
	return this->getGlobalBounds().contains(point);
}

const sf::Shape& CenteredRect::getDrawable() {
	return shape;
}

bool CenteredRect::contains(sf::Vector2f point, float robotRad) const {
	return this->getGlobalBounds().contains(point);
}

// 
//sf::CircleShape centerShape(sf::CircleShape circle)
//{
//	sf::FloatRect bounds{ circle.getLocalBounds() };
//	circle.setOrigin(bounds.width / 2, bounds.height / 2);
//	return circle;
//}
//sf::RectangleShape centerShape(sf::RectangleShape rect)
//{
//	sf::FloatRect bounds{ rect.getLocalBounds() };
//	rect.setOrigin(bounds.width / 2, bounds.height / 2);
//	return rect;
//}
//sf::ConvexShape centerShape(sf::ConvexShape shape)
//{
//	sf::Vector2f accumulate{ sf::Vector2f() };
//	for (size_t i = 0; i < shape.getPointCount(); i++)
//	{
//		accumulate += shape.getPoint(i);
//	}
//	shape.setOrigin(accumulate / 2.0f);
//	return shape;
//}
//
//bool shapeContains(sf::CircleShape circle, sf::Vector2f point)
//{
//	sf::Vector2f diff = circle.getPosition() - point;
//	float rad = circle.getRadius();
//	return diff.x * diff.x + diff.y * diff.y <= rad * rad;
//}
//
//// TODO make this work for all rectangles, not just axis aligned ones
//bool shapeContains(sf::RectangleShape rect, sf::Vector2f point)
//{
//	return rect.getGlobalBounds().contains(point);
//}
//
//// TODO use some comp geo algos to fix this
//bool shapeContains(sf::ConvexShape shape, sf::Vector2f point)
//{
//	return shape.getGlobalBounds().contains(point);
//}
//
//CenteredShape::CenteredShape(sf::CircleShape circle)
//{
//	this->shape = &centerShape(circle);
//	this->shapeName = circ;
//}
//
//CenteredShape::CenteredShape(sf::RectangleShape rectangle)
//{
//	this->shape = &centerShape(rectangle);
//	this->shapeName = rect;
//}
//
//CenteredShape::CenteredShape(sf::ConvexShape shape)
//{
//	this->shape = &centerShape(shape);
//	this->shapeName = polygon;
//}
//
//bool CenteredShape::contains(sf::Vector2f point)
//{
//	switch (this->shapeName) {
//	case circ:
//		return shapeContains(*(dynamic_cast<sf::CircleShape*>(this->shape)), point);
//	case rect:
//		return shapeContains(*(dynamic_cast<sf::RectangleShape*>(this->shape)), point);
//	case polygon:
//		return shapeContains(*(dynamic_cast<sf::ConvexShape*>(this->shape)), point);
//	default:
//		return false; // maybe change this to an error code
//	}
//}