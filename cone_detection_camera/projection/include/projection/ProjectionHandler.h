#ifndef PROJECTIONHANDLER_H
#define PROJECTIONHANDLER_H

#include "Cone.h"
#include "Particle.h"
#include "ITransform.h"
#include "Validation.h"


class ProjectionHandler
{
public:
	void doProjection(Cone*);
	Particle* getCurParticle();
	static ProjectionHandler& get();
	~ProjectionHandler();


private:
	ProjectionHandler& operator=(const ProjectionHandler&) = delete;
	ProjectionHandler(const ProjectionHandler&) = delete;
	ProjectionHandler(ITransform<Cone, float>&, ITransform<Cone, float>&);


	static ProjectionHandler* s_instance;
	Particle* m_curParticle; // we gonne set the pointer at the one has just been progressed

	ITransform<Cone,float>& m_getX;
	ITransform<Cone,float>& m_getY;


};

#endif