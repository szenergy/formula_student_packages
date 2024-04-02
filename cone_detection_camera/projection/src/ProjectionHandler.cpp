#include "ProjectionHandler.h"
#include <iostream>

ProjectionHandler::ProjectionHandler(ITransform<Cone,float>& _t1, ITransform<Cone, float>& _t2) 
	: m_curParticle(nullptr), m_getX(_t1), m_getY(_t2) {};


ProjectionHandler::~ProjectionHandler() {
	//delete s_instance; // Do not uncomment this! Its just a formal displaying of necessity of memory freeing.
	delete m_curParticle;
	delete &m_getX;
	delete &m_getY;

}

// Singleton's get instance method
ProjectionHandler& ProjectionHandler::get() { return *s_instance; }


// If we wanna do something with the last valid particle, we can reach it via this method
Particle* ProjectionHandler::getCurParticle() { return m_curParticle; }

void ProjectionHandler::doProjection(Cone* _cone) {

	// Free the m_curParticle first. If the validation fails the getCurParticle() results nullptr
	// otherwise points at a new Particle object.
	delete m_curParticle;
	m_curParticle = nullptr;

	if (!Validation::validate(*_cone)) { delete	_cone; return; }

	m_curParticle = new Particle(
			_cone->getId(),
			m_getX.transform(*_cone),
			m_getY.transform(*_cone)
		);
	
	// If you want to display infos.
	if (CheckingVariables::getInstance()->DISPLAY_INFO) m_curParticle->printCoords();

	// Delete cone, we dont need it anymore. More memory |("-")|
	delete _cone;
}

// Create singleton instance and initalize it with our custom transform implementations
ProjectionHandler* ProjectionHandler::s_instance = new ProjectionHandler(*new LongitudinalTransform(), *new LateralTransform());
