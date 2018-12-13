///Classes
#include "mapPlanning.h"


//Key constants
const int key_left = 81;
const int key_up = 82;
const int key_down = 84;
const int key_right = 83;
const int key_esc = 27;


const std::string imagePath="../robotControl/floor_plan.png";

int main()
{
    mapPlanning mapObj(imagePath);
    mapObj.calculateMap();
    mapObj.showMap();

	return 0;
}
