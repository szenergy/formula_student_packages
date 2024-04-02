#include "Cone.h"

Cone::Cone(const int& _id, const float _x, const float _y, const float _area)
	: Particle::Particle(_id, _x, _y), m_area(_area) {};

Cone::~Cone() {
	/*std::cout << "Destroyed >>> ";
	this->printCoords();*/
}

float& Cone::getArea() { return this->m_area; }

void Cone::printCoords() {
	std::cout << "ID:" << " " << this->getId() << " ";
	std::cout << "X:" << " " << this->m_position->getX() << " ";
	std::cout << "Y:" << " " << this->m_position->getY() << " ";
	std::cout << "Area:" << " " << this->getArea() << " ";
	std::cout << std::endl;
	
}

