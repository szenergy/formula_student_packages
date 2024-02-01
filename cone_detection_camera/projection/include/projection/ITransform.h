#ifndef ITRANSFORM_H
#define ITRANSFORM_H
#include <vector>
#include "CheckingVariables.h"
#include "Validation.h"
#include "Cone.h"
#include <math.h>

template<class ArgValue,class ReturnValue> 
class ITransform
{
public:
	ITransform() {};
	virtual ReturnValue transform(ArgValue&) = 0;

};

class LateralTransform: public ITransform<Cone, float>
{
public:
	LateralTransform() {};
	float transform(Cone& _cone) override;
};

class LongitudinalTransform : public ITransform<Cone, float>
{
public:
	LongitudinalTransform() {};
	float transform(Cone& _cone) override;
};

#endif