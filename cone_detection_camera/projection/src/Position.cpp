#include "Position.h"
#include <iostream>

Position::Position(float _x, float _y)
	: m_posX(_x), m_posY(_y) {};

float Position::getX() {
	return this->m_posX;
}

float Position::getY() {
	return this->m_posY;
}

