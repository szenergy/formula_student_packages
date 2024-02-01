#include "CheckingVariables.h"

CheckingVariables* CheckingVariables::instance = 0;

bool CheckingVariables::DISPLAY_INFO = true;
int CheckingVariables::HORIZON_HEIGHT = 150;
float CheckingVariables::FACTOR_HORIZONTAL = 250.0f;
float CheckingVariables::FACTOR_VERTICAL = 100.0f;
float CheckingVariables::BASE_WIDTH = 113.0f;
float CheckingVariables::CONE_MIN_AREA = 400.0f;
float CheckingVariables::DIST_MAX = 1000.0f;

CheckingVariables::CheckingVariables()
{
    this->IMG_WIDTH = 640;
    this->IMG_HEIGHT = 480;
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