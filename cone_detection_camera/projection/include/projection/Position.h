#ifndef POSITION_H
#define POSITION_H
class Position
{

public:
	Position(float, float);

	float getX();
	float getY();

private:
	const float m_posX;
	const float m_posY;
};


#endif