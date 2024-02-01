#ifndef PARTICLE_H
#define PARTICLE_H

#include "Position.h"
#include <iostream>

class Particle
{
public:
	Particle(const int, const float, const float);
	~Particle();

	virtual Position& getPosition() const;
	virtual int getId() const;
	virtual void printCoords();
protected:
	Position* m_position;

private:
	const int m_id;


};

#endif