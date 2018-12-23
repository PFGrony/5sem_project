#include "pathPlanner.h"

pathPlanner::pathPlanner(std::string path)
{
    map = cv::imread(path, CV_LOAD_IMAGE_ANYCOLOR);
    cv::cvtColor(map, grayMap, CV_BGR2GRAY);
    cameFrom.resize(map.rows,std::vector<pair>(map.cols,{0,0}));

    intMap = new double*[map.rows];
    cameFromMap = new double*[map.rows];

    for (int i = 0; i < map.rows; i++)
    {
        intMap[i] = new double[map.cols];
        cameFromMap[i] = new double[map.cols];
    }

    for (int i = 0; i < map.rows; i++)
    {
        for (int j = 0; j < map.cols; j++)
        {
            if (grayMap.at<uchar>(i, j) == 0)
                intMap[i][j] = 1;
            else
                intMap[i][j] = 0;
        }
    }
}

std::deque<pair> pathPlanner::BFSPlan(pair start, pair goal)
{
    BFS(start,goal);
    return getPath(start,goal);
}

std::deque<pair> pathPlanner::GBFSPlan(pair start, pair goal)
{
    GBFS(start,goal);
    return getPath(start,goal);
}

std::deque<pair> pathPlanner::AStarPlan(pair start, pair goal)
{
    AStar(start,goal);
    return getPath(start,goal);
}

void pathPlanner::viewPath()
{
    cv::namedWindow("Path",CV_WINDOW_FREERATIO);
    cv::imshow("Path",mapCopy);
}

double pathPlanner::getPathLength()
{
    double pathLength=0;
    for(int i=1; i<routelist.size();i++)
    {
        //If zero both have changed, if one only one change
        if(abs(abs(routelist.at(i-1).x-routelist.at(i).x)-abs(routelist.at(i-1).y-routelist.at(i).y)))
            pathLength++;
        else
            pathLength+=1.41;
    }
    return pathLength;
}

void pathPlanner::drawBrushfire()
{
    brushfire=map.clone();
    for (size_t i = 0; i < map.rows; i++)
    {
        cv::Vec3b* pixel=brushfire.ptr<cv::Vec3b>(i);
        for (size_t j = 0; j < map.cols; j++)
        {
            int value=cameFromMap[i][j]+18;

            if(cameFromMap[i][j]==2)
                pixel[j]=cv::Vec3b(0,0,255);
            else if(cameFromMap[i][j]>1)
                pixel[j]=cv::Vec3b(value,value,value);
        }
    }
    drawedBrushfire=true;
}

void pathPlanner::drawPath()
{
    if(drawedBrushfire)
        mapCopy=brushfire.clone();
    else
        mapCopy = map.clone();

    if (routelist.empty())
        std::cout << "No path, try executing getPath()" << std::endl;

    for (int i = 0; i < routelist.size(); i++)
    {
        mapCopy.at<cv::Vec3b>(routelist.at(i).y, routelist.at(i).x) = cv::Vec3b(0, 255, 0);
    }
    //Start
    mapCopy.at<cv::Vec3b>(routelist.front().y, routelist.front().x) = cv::Vec3b(0, 0, 255);

    //Goal
    mapCopy.at<cv::Vec3b>(routelist.back().y, routelist.back().x) = cv::Vec3b(255, 0, 0);
}

void pathPlanner::brushfirePoint(pair start)
{

    for (int i = 0; i < map.rows; i++)
    {
        for (int j = 0; j < map.cols; j++)
        {
            cameFromMap[i][j] = intMap[i][j];
            cameFrom.at(i).at(j) = { -1,-1 };
        }
    }

    cameFrom.at(start.y).at(start.x) = start;
    cameFromMap[start.y][start.x] = 2;

    std::queue<pair> frontier;
    frontier.push(start);

    while (!frontier.empty())
    {

        pair current = frontier.front();
        frontier.pop();

        int counter = (cameFromMap[current.y][current.x]) + 1;
        //std::cout << "x: " << current.x << " y: " << current.y << " Val: "<<counter<<std::endl;

        pair neighbour;
        int connectivity = 8;
        for (size_t i = 0; i < connectivity; i++)
        {
            if (i == 0)
                neighbour = { current.x, current.y - 1 };//N
            else if (i == 1)
                neighbour = { current.x + 1, current.y };//E
            else if (i == 2)
                neighbour = { current.x, current.y + 1 };//S
            else if (i == 3)
                neighbour = { current.x - 1, current.y };//W
            else if (i == 4)
                neighbour = { current.x + 1, current.y - 1 };//NE
            else if (i == 5)
                neighbour = { current.x + 1, current.y + 1 };//SE
            else if (i == 6)
                neighbour = { current.x - 1, current.y + 1 };//SW
            else if (i == 7)
                neighbour = { current.x - 1, current.y - 1 };//NW

            if (neighbour.x < 0 || neighbour.y < 0 || neighbour.x >= map.cols || neighbour.y >= map.rows)
                continue;

            if (cameFromMap[neighbour.y][neighbour.x] == 0)
            {
                frontier.push(neighbour);
                cameFromMap[neighbour.y][neighbour.x] = counter;
                cameFrom.at(neighbour.y).at(neighbour.x) = current;
            }
        }
    }
}


//Planning Algorithms
void pathPlanner::pairToNode(pair var1, node &var2)
{
    var2.x = var1.x;
    var2.y = var1.y;
}

void pathPlanner::BFS(pair start, pair goal)
{

    for (int i = 0; i < map.rows; i++)
    {
        for (int j = 0; j < map.cols; j++)
        {
            cameFromMap[i][j] = intMap[i][j];
            cameFrom.at(i).at(j) = { -1,-1 };
        }
    }

    cameFrom.at(start.y).at(start.x) = start;
    cameFromMap[start.y][start.x] = 2;

    std::queue<pair> frontier;
    frontier.push(start);

    while (!frontier.empty())
    {

        pair current = frontier.front();
        frontier.pop();
        if (current.x == goal.x && current.y == goal.y)
            break;

        pair neighbour;
        int connectivity = 8;
        for (size_t i = 0; i < connectivity; i++)
        {
            if (i == 0)
                neighbour = { current.x, current.y - 1 };//N
            else if (i == 1)
                neighbour = { current.x + 1, current.y };//E
            else if (i == 2)
                neighbour = { current.x, current.y + 1 };//S
            else if (i == 3)
                neighbour = { current.x - 1, current.y };//W
            else if (i == 4)
                neighbour = { current.x + 1, current.y - 1 };//NE
            else if (i == 5)
                neighbour = { current.x + 1, current.y + 1 };//SE
            else if (i == 6)
                neighbour = { current.x - 1, current.y + 1 };//SW
            else if (i == 7)
                neighbour = { current.x - 1, current.y - 1 };//NW

            if (neighbour.x < 0 || neighbour.y < 0 || neighbour.x >= map.cols || neighbour.y >= map.rows)
                continue;

            int counter = (cameFromMap[current.y][current.x]) + 1;
            if(i>3)
                counter+=1.41;

            if (cameFromMap[neighbour.y][neighbour.x] == 0)
            {
                frontier.push(neighbour);
                cameFromMap[neighbour.y][neighbour.x] = counter;
                cameFrom.at(neighbour.y).at(neighbour.x) = current;
            }
        }
    }

    //Debug code
    //	for (size_t i = 0; i < map.rows; i++)
    //	{
    //		for (size_t j = 0; j < map.cols; j++)
    //		{
    //			if (i == start.y && j == start.x)
    //				std::cout << 'S';
    //			else if (i == goal.y && j == goal.x)
    //				std::cout << 'G';
    //			else if (cameFromMap[i][j] == 0)
    //				std::cout << ' ';
    //			else if (cameFromMap[i][j] == 1)
    //				std::cout << '#';
    //			else
    //			{
    //				pair temp = cameFrom.at(i).at(j);
    //				if (i < temp.y)
    //					std::cout << 'V';
    //				else if (i > temp.y)
    //					std::cout << '^';
    //				else if (j < temp.x)
    //					std::cout << '>';
    //				else if (j > temp.x)
    //					std::cout << '<';
    //			}
    //			//std::cout << char(cameFromMap[i][j] % 8 + '2');
    //		}
    //		std::cout << std::endl;
    //	}

}

void pathPlanner::GBFS(pair startPair, pair goalPair)
{

    for (int i = 0; i < map.rows; i++)
    {
        for (int j = 0; j < map.cols; j++)
        {
            cameFromMap[i][j] = intMap[i][j];
            cameFrom.at(i).at(j) = { -1,-1 };
        }
    }
    cameFrom.at(startPair.y).at(startPair.x) = startPair;

    cameFromMap[startPair.y][startPair.x] = 2;

    node goal;
    pairToNode(goalPair, goal);


    node start;
    pairToNode(startPair, start);
    start.h = 0;
    std::priority_queue< node, std::vector<node>, compareHeuristic> frontier;
    frontier.push(start);


    while (!frontier.empty())
    {

        node current = frontier.top();
        frontier.pop();

        if (current.x == goal.x && current.y == goal.y)
            break;

        node neighbour;
        int connectivity = 8;
        for (size_t i = 0; i < connectivity; i++)
        {
            if (i == 0)
                neighbour = { current.x, current.y - 1 };//N
            else if (i == 1)
                neighbour = { current.x + 1, current.y };//E
            else if (i == 2)
                neighbour = { current.x, current.y + 1 };//S
            else if (i == 3)
                neighbour = { current.x - 1, current.y };//W
            else if (i == 4)
                neighbour = { current.x + 1, current.y - 1 };//NE
            else if (i == 5)
                neighbour = { current.x + 1, current.y + 1 };//SE
            else if (i == 6)
                neighbour = { current.x - 1, current.y + 1 };//SW
            else if (i == 7)
                neighbour = { current.x - 1, current.y - 1 };//NW

            if (neighbour.x < 0 || neighbour.y < 0 || neighbour.x >= map.cols || neighbour.y >= map.rows)
                continue;

            int counter = (cameFromMap[current.y][current.x]) + 1;
            if(i>3)
                counter+=1.41;

            if (cameFromMap[neighbour.y][neighbour.x] == 0)
            {
                neighbour.h = (double)abs(neighbour.x - goal.x) + (double)abs(neighbour.y - goal.y);
                frontier.push(neighbour);
                cameFromMap[neighbour.y][neighbour.x] = counter;
                cameFrom.at(neighbour.y).at(neighbour.x) = { current.x,current.y };
            }
        }


    }

//    //debug code
//    for (size_t i = 0; i < map.rows; i++)
//    {
//        for (size_t j = 0; j < map.cols; j++)
//        {
//            if (i == start.y && j == start.x)
//                std::cout << 'S';
//            else if (i == goal.y && j == goal.x)
//                std::cout << 'G';
//            else if (cameFromMap[i][j] == 0)
//                std::cout << ' ';
//            else if (cameFromMap[i][j] == 1)
//                std::cout << '#';
//            else
//            {
//                pair temp = cameFrom.at(i).at(j);
//                if (i < temp.y)
//                    std::cout << 'V';
//                else if (i > temp.y)
//                    std::cout << '^';
//                else if (j < temp.x)
//                    std::cout << '>';
//                else if (j > temp.x)
//                    std::cout << '<';
//            }
//        }
//        std::cout << std::endl;
//    }
}

void pathPlanner::AStar(pair startPair, pair goalPair)
{
    for (int i = 0; i < map.rows; i++)
    {
        for (int j = 0; j < map.cols; j++)
        {
            cameFromMap[i][j] = intMap[i][j];
//            costSoFar[i][j]=intMap[i][j];
            cameFrom.at(i).at(j) = { -1,-1 };
        }
    }

    node goal;
    pairToNode(goalPair, goal);

    std::priority_queue< node, std::vector<node>, compareCost> frontier;
    node start;
    pairToNode(startPair, start);
    frontier.push(start);

    cameFromMap[startPair.y][startPair.x] = 2;
    cameFrom.at(startPair.y).at(startPair.x) = startPair;

    while (!frontier.empty())
    {

        node current = frontier.top();
        frontier.pop();

        if (current.x == goal.x && current.y == goal.y)
            break;

        int connectivity = 8;

        for (size_t i = 0; i < connectivity; i++)
        {

            node neighbour;
            if (i == 0)
                neighbour = { current.x, current.y - 1 };//N
            else if (i == 1)
                neighbour = { current.x + 1, current.y };//E
            else if (i == 2)
                neighbour = { current.x, current.y + 1 };//S
            else if (i == 3)
                neighbour = { current.x - 1, current.y };//W
            else if (i == 4)
                neighbour = { current.x + 1, current.y - 1 };//NE
            else if (i == 5)
                neighbour = { current.x + 1, current.y + 1 };//SE
            else if (i == 6)
                neighbour = { current.x - 1, current.y + 1 };//SW
            else if (i == 7)
                neighbour = { current.x - 1, current.y - 1 };//NW

            if (neighbour.x < 0 || neighbour.y < 0 || neighbour.x >= map.cols || neighbour.y >= map.rows)
                continue;


            double newCost = cameFromMap[current.y][current.x] + 1;

            //Diagonal Moves Have higher cost
            if (i > 3)
                newCost++;


            if (cameFromMap[neighbour.y][neighbour.x] == 0 || newCost < cameFromMap[neighbour.y][neighbour.x])
            {
                cameFromMap[neighbour.y][neighbour.x] = newCost;
                neighbour.f = newCost + sqrt(pow(abs(neighbour.x - goal.x), 2) + pow(abs(neighbour.y - goal.y), 2));
                frontier.push(neighbour);
//                cameFromMap[neighbour.y][neighbour.x] = counter;
                cameFrom.at(neighbour.y).at(neighbour.x) = { current.x,current.y };
                //std::cout << "(" << neighbour.x << "," << neighbour.y << ")" << " f: " << neighbour.f << std::endl;
            }
        }
    }


    //    brushfire.at<cv::Vec3b>(start.y,start.x)=cv::Vec3b(0,255,0);
    //    brushfire.at<cv::Vec3b>(goal.y,goal.x)=cv::Vec3b(0,255,0);
    //Debug which direction is preferred
    //	for (size_t i = 0; i < map.rows; i++)
    //	{
    //		for (size_t j = 0; j < map.cols; j++)
    //		{
    //			if (i == start.y && j == start.x)
    //				std::cout << 'S';
    //			else if (i == goal.y && j == goal.x)
    //				std::cout << 'G';
    //			else if (cameFromMap[i][j] == 0)
    //				std::cout << ' ';
    //			else if (cameFromMap[i][j] == 1)
    //				std::cout << '#';
    //			else
    //			{

    //				pair temp = cameFrom.at(i).at(j);
    //				if (i < temp.y)
    //					std::cout << 'V';
    //				else if (i > temp.y)
    //					std::cout << '^';
    //				else if (j < temp.x)
    //					std::cout << '>';
    //				else if (j > temp.x)
    //					std::cout << '<';
    //			}
    //		}
    //		std::cout << std::endl;
    //	}
}

std::deque<pair> pathPlanner::getPath(pair start, pair goal)
{
    pair current = goal;
    std::deque<pair> path;

    while (!(current.x == start.x && current.y == start.y))
    {
        path.push_front(current);
        current = cameFrom.at(current.y).at(current.x);
    }
    //std::cout << "Path Recievied" << std::endl;
    path.push_front(start);

    drawedBrushfire=false;
    routelist = path;
    return path;
}
