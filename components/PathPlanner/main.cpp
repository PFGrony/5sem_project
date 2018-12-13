///Classes
#include "pathPlanner.h"
#include "fstream"


//Key constants
const int key_left = 81;
const int key_up = 82;
const int key_down = 84;
const int key_right = 83;
const int key_esc = 27;


const std::string imagePath="../../maps/floor_plan.png";
std::fstream data;

void timeMethods(pair start, pair goal);

pathPlanner pathObj(imagePath);
int main()
{

    pair start={7,7};
    pair goal1={22,9},goal2={20,23},goal3={34,41},goal4={64,39},goal5={93,32},goal6={105,56},goal7={78,70},goal8={54,70};


//    //Test 1

//    pathObj.BFSPlan(start,goal8);
//    pathObj.drawBrushfire();
//    pathObj.drawPath();
//    pathObj.viewPath();
//    cv::waitKey();

    pathObj.AStarPlan(start,goal8);
    pathObj.drawBrushfire();
    pathObj.drawPath();
    pathObj.viewPath();
    cv::waitKey();


//    //Test 2

//    data.open ("../maps/timeGraph.txt", std::fstream::in | std::fstream::out | std::fstream::app);
//    data<<"pathLenghth,AStar,BFS"<<"\n";

//    std::cout<<"Test 2"<<std::endl;
//    for(int i=0;i<5;i++)
//    {
//        timeMethods(start,goal1);
//    }
//    for(int i=0;i<5;i++)
//    {
//        timeMethods(start,goal2);
//    }
//    for(int i=0;i<5;i++)
//    {
//        timeMethods(start,goal3);
//    }
//    for(int i=0;i<5;i++)
//    {
//     timeMethods(start,goal4);
//    }
//    for(int i=0;i<5;i++)
//    {
//        timeMethods(start,goal5);
//    }
//    for(int i=0;i<5;i++)
//    {
//        timeMethods(start,goal6);
//    }
//    for(int i=0;i<5;i++)
//    {
//        timeMethods(start,goal7);
//    }
//    for(int i=0;i<5;i++)
//    {
//        timeMethods(start,goal8);
//    }

//    data.close();

	return 0;
}

void timeMethods(pair start, pair goal)
{
    auto startT = std::chrono::high_resolution_clock::now();
    auto finishT = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed1, elapsed2;

    startT = std::chrono::high_resolution_clock::now();
    pathObj.AStarPlan(start,goal);
    finishT = std::chrono::high_resolution_clock::now();
    elapsed2 = finishT - startT;

    startT = std::chrono::high_resolution_clock::now();
    pathObj.BFSPlan(start,goal);
    finishT = std::chrono::high_resolution_clock::now();
    elapsed1 = finishT - startT;

    std::cout<<"Path length: "<<pathObj.getPathLength()<<std::endl;
    std::cout << "A star Elapsed time: " << elapsed1.count() << " s\n";
    std::cout << "BFS Elapsed time: " << elapsed2.count() << " s\n\n";
    data<<pathObj.getPathLength()<<","<<elapsed1.count()<<","<< elapsed2.count()<<"\n";
}
