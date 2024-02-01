#ifndef VALIDATION_H
#define VALIDATION_H
#include "Cone.h"
#include "CheckingVariables.h"

class Validation
{
public:
	Validation();
	static bool validate(Cone&);

};

class OnLeftSide
{
public:
	OnLeftSide();
	static bool validate(Cone& _cone);

};

#endif