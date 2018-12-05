/*

cv::Mat mapWave = map.clone();
cv::Mat getMapWave()
{
    return mapWave;
}


std::deque<pair> pathPlanner::getWavefrontRoute()
{
    return routelist;
}

void pathPlanner::wavefrontPlanner(pair start, pair goal)
{

    for (int i = 0; i < map.rows; i++)
    {
        for (int j = 0; j < map.cols; j++)
        {
            cameFromMap[i][j] = intMap[i][j];
        }
    }

    std::deque<pair> queueAdj;
    queueAdj.push_back(pair{ goal.x,goal.y });
    cameFromMap[goal.y][goal.x] = 2;


    while (!queueAdj.empty())
    {

        pair current = queueAdj.front();
        if (current.x == start.x && current.y == start.y)
            break;

        int counter = (cameFromMap[current.y][current.x]) + 1;
        //std::cout << "x: " << current.x << " y: " << current.y << " Val: "<<counter<<std::endl;

        pair neighbour;
        for (size_t i = 0; i < 8; i++)
        {
            if (i == 0)
                neighbour = pair{ current.x, current.y - 1 };//N
            else if (i == 1)
                neighbour = pair{ current.x + 1, current.y };//E
            else if (i == 2)
                neighbour = pair{ current.x, current.y + 1 };//S
            else if (i == 3)
                neighbour = pair{ current.x - 1, current.y };//W
            else if (i == 4)
                neighbour = pair{ current.x + 1, current.y - 1 };//NE
            else if (i == 5)
                neighbour = pair{ current.x + 1, current.y + 1 };//SE
            else if (i == 6)
                neighbour = pair{ current.x - 1, current.y + 1 };//SW
            else if (i == 7)
                neighbour = pair{ current.x - 1, current.y - 1 };//NW


            if (cameFromMap[neighbour.y][neighbour.x] == 0)
            {
                queueAdj.push_back(neighbour);
                cameFromMap[neighbour.y][neighbour.x] = counter;
            }
        }

        queueAdj.pop_front();

    }


    //for (int i = 0; i < grayMap.rows; i++)
    //{
    //	for (int j = 0; j < grayMap.cols; j++)
    //	{
    //		if ((j==start.x && i==start.y))
    //			std::cout<<'!';
    //		else
    //			std::cout << cameFromMap[i][j];
    //	}
    //	std::cout << std::endl;
    //}

}

void pathPlanner::wavefrontRoute(pair start, pair goal)
{
    routelist.clear();
    routelist.push_back(pair{ start.x,start.y });
    pair current = pair{ start.x,start.y };

    //    std::cout<<"x: "<<temp.x<<" y: "<<temp.y<<" counter: "<< cameFromMap[temp.y][temp.x]<<std::endl;
    while (cameFromMap[current.y][current.x] != 2)
    {
        pair neighbour;
        for (size_t i = 0; i < 8; i++)
        {
            if (i == 0)
                neighbour = pair{ current.x, current.y - 1 };//N
            else if (i == 1)
                neighbour = pair{ current.x + 1, current.y };//E
            else if (i == 2)
                neighbour = pair{ current.x, current.y + 1 };//S
            else if (i == 3)
                neighbour = pair{ current.x - 1, current.y };//W
            else if (i == 4)
                neighbour = pair{ current.x + 1, current.y - 1 };//NE
            else if (i == 5)
                neighbour = pair{ current.x + 1, current.y + 1 };//SE
            else if (i == 6)
                neighbour = pair{ current.x - 1, current.y + 1 };//SW
            else if (i == 7)
                neighbour = pair{ current.x - 1, current.y - 1 };//NW


            if (cameFromMap[neighbour.y][neighbour.x] == cameFromMap[current.y][current.x] - 1)
            {
                routelist.push_back(neighbour);
                current = neighbour;
                break;
            }
        }
    }
}

void pathPlanner::drawWavefrontBrushfire(pair start, pair goal)
{
    mapCopy = map.clone();
    for (int i = 0; i < map.rows; i++)
    {
        for (int j = 0; j < map.cols; j++)
        {
            if (cameFromMap[i][j] == 1)
            {
                mapCopy.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
            }
            else if (cameFromMap[i][j] == 0)
            {
                mapCopy.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
            }
            else
            {
                mapCopy.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255 - cameFromMap[i][j], 0);
            }


        }
    }

    //Start
    mapCopy.at<cv::Vec3b>(start.y, start.x) = cv::Vec3b(0, 0, 255);

    //Goal
    mapCopy.at<cv::Vec3b>(goal.y, goal.x) = cv::Vec3b(255, 0, 0);
}

void pathPlanner::drawWavefrontRoute(pair start, pair goal)
{
    mapCopy = map.clone();

    if (routelist.empty())
        std::cout << "empty path" << std::endl;

    for (int i = 0; i < routelist.size(); i++)
        mapCopy.at<cv::Vec3b>(routelist.at(i).y, routelist.at(i).x) = cv::Vec3b(0, 255, 0);

    //Start
    mapCopy.at<cv::Vec3b>(start.y, start.x) = cv::Vec3b(0, 0, 255);

    //Goal
    mapCopy.at<cv::Vec3b>(goal.y, goal.x) = cv::Vec3b(255, 0, 0);
}
*/
