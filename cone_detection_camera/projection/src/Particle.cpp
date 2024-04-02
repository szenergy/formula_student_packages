#include "Particle.h"

Particle::Particle(const int _id, const float _x, const float _y)
	: m_id(_id), m_position(new Position(_x, _y)) {};

Particle::~Particle() {
	//std::cout << "Deleted " << "X: " << this->m_position->getX() << "Y: " << this->m_position->getY() << std::endl;
	delete this->m_position;
}

Position& Particle::getPosition() const { return *this->m_position; }
int Particle::getId() const { return this->m_id; }

void Particle::printCoords() {
	std::cout << "ID:" << " " << this->getId() << " ";
	std::cout << "X:" << " " << this->m_position->getX() << " ";
	std::cout << "Y:" << " " << this->m_position->getY();
	std::cout << std::endl;
}