#pragma once
#include "Particle.h"
#include "Position.h"

class Cone : public Particle
{
public:
	Cone(const int&, const float, const float, const float);
	~Cone();

	void printCoords() override;
	float& getArea();

private:
	float m_area;
};
