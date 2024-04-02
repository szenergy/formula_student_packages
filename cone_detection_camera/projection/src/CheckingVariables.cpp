#include "CheckingVariables.h"

CheckingVariables* CheckingVariables::instance = 0;

bool CheckingVariables::DISPLAY_INFO = true;
int CheckingVariables::HORIZON_HEIGHT = 188;
float CheckingVariables::FACTOR_HORIZONTAL = 220.0f;
float CheckingVariables::FACTOR_VERTICAL = 70.0f;
float CheckingVariables::BASE_WIDTH = 336.0f;
float CheckingVariables::CONE_MIN_AREA = 20.0f;
float CheckingVariables::DIST_MAX = 10000.0f;

CheckingVariables::CheckingVariables()
{
    this->IMG_WIDTH = 672;
    this->IMG_HEIGHT = 376;
    this->IMG_WIDTH_HALF = IMG_WIDTH / 2;
    this->HORIZON_HEIGHT_REL = IMG_HEIGHT - HORIZON_HEIGHT;
    this->CONST_Y0 = (float)IMG_WIDTH_HALF / HORIZON_HEIGHT_REL;

    this->COUNT_MSG = true;

}


CheckingVariables* CheckingVariables::getInstance(){
    if(instance==nullptr){
        instance = new CheckingVariables();
    }
    return instance;
}

void CheckingVariables::updateParams(){
    this->HORIZON_HEIGHT_REL = IMG_HEIGHT - HORIZON_HEIGHT;
    this->CONST_Y0 = (float)IMG_WIDTH_HALF / HORIZON_HEIGHT_REL;
}