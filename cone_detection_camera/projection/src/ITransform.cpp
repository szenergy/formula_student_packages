#include "ITransform.h"

float LateralTransform::transform(Cone& _cone){
	float ratio = _cone.getPosition().getX() - CheckingVariables::getInstance()->IMG_WIDTH_HALF;
	ratio = OnLeftSide::validate(_cone) ? ratio  : ratio * (-1);
	ratio /= (_cone.getPosition().getY() - CheckingVariables::HORIZON_HEIGHT) * CheckingVariables::getInstance()->CONST_Y0;
	ratio *= CheckingVariables::BASE_WIDTH;
	
    return OnLeftSide::validate(_cone) ? ratio * (-1) : ratio ;
}

float LongitudinalTransform::transform(Cone& _cone){
	float subResult = (_cone.getPosition().getY() - CheckingVariables::HORIZON_HEIGHT) / CheckingVariables::getInstance()->HORIZON_HEIGHT_REL;
	float subResultRatio = atan(CheckingVariables::FACTOR_VERTICAL / CheckingVariables::FACTOR_HORIZONTAL);
	
    return CheckingVariables::FACTOR_VERTICAL / tan(subResult * subResultRatio);
}