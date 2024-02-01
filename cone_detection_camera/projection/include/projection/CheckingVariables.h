#ifndef CHECKINGVARIABLES_H
#define CHECKINGVARIABLES_H
#include <iostream>


class CheckingVariables{
private:
    static CheckingVariables* instance;
public:
    static bool DISPLAY_INFO; // Use it for truning on/off debug infos
    int COUNT_MSG;

    // Image basic params
    int IMG_WIDTH;
    int IMG_HEIGHT;
    int IMG_WIDTH_HALF;
    // Horizon params
    static int HORIZON_HEIGHT;
    int HORIZON_HEIGHT_REL;
    // Other Constants
    static float FACTOR_HORIZONTAL;// lv: M1_1_Proj_x0
    static float FACTOR_VERTICAL;// lv: M1_1_Proj_z0
    static float BASE_WIDTH;// lv: M1_1_Proj_y0
    static float CONE_MIN_AREA;// lv: M1_1_Proj_area_thd

    float CONST_Y0;
    static float DIST_MAX; // Max distance a cone can be far from the camera

    // Functions
    CheckingVariables();
    static CheckingVariables* getInstance();
    void updateParams();
};
// Idk mik ezek, de majd megk�rdezem :)

/*
// lv-ban interval a neve, nem grid size
const int particleGridSize = 25; // [cm]
// ez nem ugyan az az ertek mint az imHeight???
const int anchorPixelRow = 480; // lv: M1_1_Proj_anchor_row
// ez a valtozo kell? labview-ban jelen van es ott is 0 az erteke
const int projFact = 0; // lv: M1_1_Proj_proj_fact
*/
#endif