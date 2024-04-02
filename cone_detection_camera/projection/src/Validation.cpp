#include "Validation.h"

Validation::Validation() {};

bool Validation::validate(Cone& _cone) {
		return
			_cone.getPosition().getX() <= CheckingVariables::DIST_MAX &&		// Is distance fine?
			_cone.getPosition().getY() >= CheckingVariables::HORIZON_HEIGHT &&	// Is beyond horizon?
			_cone.getArea() >= CheckingVariables::CONE_MIN_AREA;				// Has min. required area?
	};


OnLeftSide::OnLeftSide() {};

bool OnLeftSide::validate(Cone& _cone) { return _cone.getPosition().getX() <= CheckingVariables::getInstance()->IMG_WIDTH_HALF ? true : false; }
