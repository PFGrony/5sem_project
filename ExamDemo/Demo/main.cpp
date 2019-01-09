#include <iostream>
#include "mapPlanning.h"
#include "QLearning.h"

using namespace std;

int main()
{
    // Opret objekt med path til kortet
    mapPlanning planner("floor_plan.png");
    // beregn det
    planner.calculateMap();
    //QLearning QL;
    //QL.importMap(planner.getPathVec());
    //QL.runQLearning();
    return 0;
}
