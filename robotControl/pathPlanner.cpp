#include "pathPlanner.h"

pathPlanner::pathPlanner(std::string path)
{
    smallMap=cv::imread(path,CV_LOAD_IMAGE_GRAYSCALE);
    mapWave = cv::imread(path, CV_LOAD_IMAGE_ANYCOLOR);

	for (int i = 0; i < smallMap.rows; i++)
	{
		for (int j = 0; j < smallMap.cols; j++)
		{
			if (smallMap.at<uchar>(i, j) == 0)
				charMap[i][j] = 1;
			else
				charMap[i][j] = 0;
		}
	}
}

cv::Mat pathPlanner::getMap()
{
	return map;
}

void pathPlanner::addAdj(std::deque<pair> &queueAdj)
{
	pair pos = queueAdj.front();
	int counter = (newMap[pos.y][pos.x]) + 1;
	//if (counter == 'z')
	//	counter = '3';
	//std::cout << "x: " << pos.x << " y: " << pos.y << " Val: "<<counter<<std::endl;
	//std::cout << counter << std::endl;

	//N
	pair temp = pair{ pos.x, pos.y - 1 };
	if (newMap[temp.y][temp.x] == 0)
	{
		queueAdj.push_back(pair{ temp.x,temp.y });
		newMap[temp.y][temp.x] = counter;
	}
//    //NE
//    temp = pair{ pos.x + 1, pos.y - 1 };
//    if (newMap[temp.y][temp.x] == 0)
//    {
//        queueAdj.push_back(pair{ temp.x,temp.y });
//        newMap[temp.y][temp.x] = counter;
//    }
	//E
	temp = pair{ pos.x + 1, pos.y };
	if (newMap[temp.y][temp.x] == 0)
	{
		queueAdj.push_back(pair{ temp.x,temp.y });
		newMap[temp.y][temp.x] = counter;
	}
//    //SE
//    temp = pair{ pos.x + 1, pos.y + 1 };
//    if (newMap[temp.y][temp.x] == 0)
//    {
//        queueAdj.push_back(pair{ temp.x,temp.y });
//        newMap[temp.y][temp.x] = counter;
//    }
	//S
	temp = pair{ pos.x, pos.y + 1 };
	if (newMap[temp.y][temp.x] == 0)
	{
		queueAdj.push_back(pair{ temp.x,temp.y });
		newMap[temp.y][temp.x] = counter;
	}
//    //SW
//    temp = pair{ pos.x - 1, pos.y + 1 };
//    if (newMap[temp.y][temp.x] == 0)
//    {
//        queueAdj.push_back(pair{ temp.x,temp.y });
//        newMap[temp.y][temp.x] = counter;
//    }
	//W
	temp = pair{ pos.x - 1, pos.y };
	if (newMap[temp.y][temp.x] == 0)
	{
		queueAdj.push_back(pair{ temp.x,temp.y });
		newMap[temp.y][temp.x] = counter;
	}
//    //NW
//    temp = pair{ pos.x - 1, pos.y - 1 };
//    if (newMap[temp.y][temp.x] == 0)
//    {
//        queueAdj.push_back(pair{ temp.x,temp.y });
//        newMap[temp.y][temp.x] = counter;
//    }
}

void pathPlanner::wavefrontRoute(pair start, pair goal)
{
    routelist.clear();
    routelist.push_back(pair{ start.x,start.y });
    pair temp=pair{start.x,start.y};

//    std::cout<<"x: "<<temp.x<<" y: "<<temp.y<<" counter: "<< newMap[temp.y][temp.x]<<std::endl;
    while(newMap[temp.y][temp.x]!=2)
    {
        //N
        if (newMap[temp.y-1][temp.x] == newMap[temp.y][temp.x]-1)
        {
            temp = pair{ temp.x, temp.y - 1 };
            routelist.push_back(pair{ temp.x,temp.y});
        }
//        //NE
//        else if (newMap[temp.y-1][temp.x+1]==newMap[temp.y][temp.x]-1)
//        {
//            temp = pair{ temp.x + 1, temp.y - 1 };
//            routelist.push_back(pair{ temp.x,temp.y });

//        }
        //E
        else if (newMap[temp.y][temp.x+1]==newMap[temp.y][temp.x]-1)
        {
            temp = pair{ temp.x + 1, temp.y };
            routelist.push_back(pair{ temp.x,temp.y });
        }
//        //SE
//        else if (newMap[temp.y+1][temp.x+1]==newMap[temp.y][temp.x]-1)
//        {
//            temp = pair{ temp.x + 1, temp.y + 1 };
//            routelist.push_back(pair{ temp.x,temp.y });
//        }
        //S
        else if (newMap[temp.y+1][temp.x] ==newMap[temp.y][temp.x]-1)
        {
            temp = pair{ temp.x, temp.y + 1 };
            routelist.push_back(pair{ temp.x,temp.y });
        }
//        //SW
//        else if (newMap[temp.y+1][temp.x-1]==newMap[temp.y][temp.x]-1)
//        {
//            temp = pair{ temp.x - 1, temp.y + 1 };
//            routelist.push_back(pair{ temp.x,temp.y });
//        }
        //W
        else if (newMap[temp.y][temp.x-1]==newMap[temp.y][temp.x]-1)
        {
            temp = pair{ temp.x - 1, temp.y };
            routelist.push_back(pair{ temp.x,temp.y });
        }
//        //NW
//        else if (newMap[temp.y-1][temp.x-1]==newMap[temp.y][temp.x]-1)
//        {
//            temp = pair{ temp.x - 1, temp.y - 1 };
//            routelist.push_back(pair{ temp.x,temp.y });
//        }
//        std::cout<<"x: "<<temp.x<<" y: "<<temp.y<<" counter: "<< newMap[temp.y][temp.x]<<std::endl;

    }
}


std::deque<pair> pathPlanner::getWavefrontRoute()
{
    return routelist;
}

void pathPlanner::drawWavefrontRoute(pair start,pair goal)
{
    mapWave = cv::imread("../robotControl/floor_plan.png", CV_LOAD_IMAGE_ANYCOLOR);

    for(int i=0;i<routelist.size();i++)
    {
        mapWave.at<cv::Vec3b>(routelist.at(i).y,routelist.at(i).x)[0] = 0;
        mapWave.at<cv::Vec3b>(routelist.at(i).y,routelist.at(i).x)[1] = 255;
        mapWave.at<cv::Vec3b>(routelist.at(i).y,routelist.at(i).x)[2] = 0;
    }
    //Start
    mapWave.at<cv::Vec3b>(start.y, start.x)[0] = 0;
    mapWave.at<cv::Vec3b>(start.y, start.x)[1] = 0;
    mapWave.at<cv::Vec3b>(start.y, start.x)[2] = 255;

    //Goal
    mapWave.at<cv::Vec3b>(goal.y,goal.x)[0] = 255;
    mapWave.at<cv::Vec3b>(goal.y,goal.x)[1] = 0;
    mapWave.at<cv::Vec3b>(goal.y,goal.x)[2] = 0;
    cv::resize(mapWave, mapWave, cv::Size(), 8, 8, cv::INTER_NEAREST);
}

void pathPlanner::drawWavefrontBrushfire(pair start,pair goal)
{
    mapWave = cv::imread("../robotControl/floor_plan.png", CV_LOAD_IMAGE_ANYCOLOR);
    for (int i = 0; i < mapWave.rows; i++)
    {
        for (int j = 0; j < mapWave.cols; j++)
        {
            if (newMap[i][j] == 1 )
            {
                for (int k = 0; k < mapWave.channels(); k++)
                    mapWave.at<cv::Vec3b>(i, j)[k] = 0;
            }
            else if (newMap[i][j] == 0)
            {
                for (int k = 0; k < mapWave.channels(); k++)
                    mapWave.at<cv::Vec3b>(i, j)[k] = 255;
            }
            else
            {
                    mapWave.at<cv::Vec3b>(i, j)[0] = 0;
                    mapWave.at<cv::Vec3b>(i, j)[1] = 255 - newMap[i][j];
                    mapWave.at<cv::Vec3b>(i, j)[2] = 0;
            }


        }
    }

    //Start
    mapWave.at<cv::Vec3b>(start.x, start.y)[0] = 0;
    mapWave.at<cv::Vec3b>(start.x, start.y)[1] = 0;
    mapWave.at<cv::Vec3b>(start.x, start.y)[2] = 255;


    //Goal
    mapWave.at<cv::Vec3b>(goal.x,goal.y)[0] = 255;
    mapWave.at<cv::Vec3b>(goal.x,goal.y)[1] = 0;
    mapWave.at<cv::Vec3b>(goal.x,goal.y)[2] = 0;


    cv::resize(mapWave, mapWave, cv::Size(), 8, 8, cv::INTER_NEAREST);
}




void pathPlanner::wavefrontPlanner(pair start, pair goal)
{

	for (int i = 0; i < smallMap.rows; i++)
	{
		for (int j = 0; j < smallMap.cols; j++)
		{
			newMap[i][j] = charMap[i][j];
		}
	}


	//    for (int i = 0;i<smallMap.rows;i++)
	//    {
	//        for (int j = 0;j<smallMap.cols;j++)
	//        {
	//            std::cout << newMap[i][j];
	//        }
	//        std::cout << std::endl;
	//    }

	std::deque<pair> queueAdj;
	queueAdj.push_back(pair{ goal.x,goal.y });
	newMap[goal.y][goal.x] = 2;


	while (!( queueAdj.front().x==start.x && queueAdj.front().y==start.y) )
	{
		addAdj(queueAdj);
		queueAdj.pop_front();
	}
	

	//for (int i = 0; i < smallMap.rows; i++)
	//{
	//	for (int j = 0; j < smallMap.cols; j++)
	//	{
	//		if ((j==start.x && i==start.y))
	//			std::cout<<'!';
	//		else
	//			std::cout << newMap[i][j];
	//	}
	//	std::cout << std::endl;
	//}
	
}

//void pathPlanner::doBrushfire()
//{
//    // map
//
//
//
//
//        char charMap[smallMap.rows][smallMap.cols];
//
////        std::cout << smallMap.rows << ":" << smallMap.cols << std::endl;
//
//        for (int i = 0;i<smallMap.rows;i++)
//        {
//            for (int j = 0;j<smallMap.cols;j++)
//            {
//                if (smallMap.at<uchar>(i,j) == 0)
//                    charMap[i][j] = '1';
//                else
//                    charMap[i][j] = '0';
//            }
//        }
//
//
//
//        //Check before Brushfire
//        for (int i = 0;i<smallMap.rows;i++)
//        {
//            for (int j = 0;j<smallMap.cols;j++)
//            {
//                std::cout << charMap[i][j];
//            }
//            std::cout << std::endl;
//        }
//
////        std::cout << "Linje 42" << std::endl;
//
//        //Brushfire
//        int valueChanged=true;
//        int counter=0;
//        while(valueChanged)
//        {
//            valueChanged=false;
//            counter++;
//
//            for(int i=0;i<smallMap.rows;i++) //Rows
//            {
//                for(int j=0; j<smallMap.cols;j++) //Columns
//                {
//                    if(charMap[i][j] - '0' == counter)
//                    {
//                        if(i-1 >= 0) //Checks out-of-bounds
//                        {
//                            if(charMap[i-1][j] - '0'== 0) //Change value to the left
//                            {
//                                valueChanged=true;
//                                charMap[i-1][j] = '0'+counter+1;
//                            }
//                        }
//
//                        if(i+1 <= smallMap.rows) //Checks out-of-bounds
//                        {
//                            if(charMap[i+1][j] - '0' == 0) //Change value to the right
//                            {
//                                valueChanged=true;
//                                charMap[i+1][j] = '0'+counter+1;
//                            }
//                        }
//
//                        if(j-1 >= 0) //Checks out-of-bounds
//                        {
//                            if(charMap[i][j-1] - '0'== 0) //Change value above
//                            {
//                                valueChanged=true;
//                                charMap[i][j-1] = '0'+counter+1;
//                            }
//                        }
//
//                        if(j+1 <= smallMap.cols) //Checks out-of-bounds
//                        {
//                            if(charMap[i][j+1] - '0'== 0) //Change value below
//                            {
//                                valueChanged=true;
//                                charMap[i][j+1] = '0'+counter+1;
//                            }
//                        }
//
//                    }
//                }
//            }
//        }
//
//
////        for (int i = 0;i<smallMap.rows;i++)
////        {
////            for (int j = 0;j<smallMap.cols;j++)
////            {
////                std::cout << charMap[i][j];
////            }
////            std::cout << std::endl;
////        }
//
////        std::cout << "Counter " << counter << std::endl;
//
//
//
//
//
//
//
//        //Brushfire - With openCV
//        cv::Mat1b im_brushfire(80,120,255);
//         im_brushfire = cv::imread("../robotControl/floor_plan.png",CV_LOAD_IMAGE_GRAYSCALE);
//         for (int i = 0;i<im_brushfire.rows;i++)
//         {
//             for (int j = 0;j<im_brushfire.cols;j++)
//             {
//                 if (im_brushfire.at<uchar>(i,j) == 0)
//                     im_brushfire.at<uchar>(i,j) = 0;
//                 else
//                     im_brushfire.at<uchar>(i,j) = 255;
//             }
//         }
//                map=im_brushfire.clone();
//                cv::resize(map, map, cv::Size(), 8, 8, cv::INTER_NEAREST);
//
////            cv::imshow("Brushfire_before",im_brushfire);
//
//        valueChanged=true;
//        counter=0;
//        int offset=18;
//        while(valueChanged)
//        {
//            valueChanged=false;
//
//
//            for(int i=0;i<smallMap.rows;i++) //Rows
//            {
//                for(int j=0; j<smallMap.cols;j++) //Columns
//                {
//                    if(im_brushfire[i][j] == counter)
//                    {
//                        if(i-1 >= 0) //Checks out-of-bounds
//                        {
//                            if(im_brushfire[i-1][j] == 255) //Change value to the left
//                            {
//                                valueChanged=true;
//                                im_brushfire[i-1][j] = counter+offset;
//                            }
//                        }
//
//                        if(i+1 <= smallMap.rows) //Checks out-of-bounds
//                        {
//                            if(im_brushfire[i+1][j]  == 255) //Change value to the right
//                            {
//                                valueChanged=true;
//                                im_brushfire[i+1][j] = counter+offset;
//                            }
//                        }
//
//                        if(j-1 >= 0) //Checks out-of-bounds
//                        {
//                            if(im_brushfire[i][j-1] == 255) //Change value above
//                            {
//                                valueChanged=true;
//                                im_brushfire[i][j-1] = counter+offset;
//                            }
//                        }
//
//                        if(j+1 <= smallMap.cols) //Checks out-of-bounds
//                        {
//                            if(im_brushfire[i][j+1] == 255) //Change value below
//                            {
//                                valueChanged=true;
//                                im_brushfire[i][j+1] = counter+offset;
//                            }
//                        }
//
//                    }
//                }
//            }
//
//            counter+=offset;
//        }
//
//        cv::resize(im_brushfire, im_brushfire, cv::Size(), 8, 8, cv::INTER_NEAREST);
//
//
////        cv::imshow("Brushfire",im_brushfire);
//
//
//
//}


void pathPlanner::addVertex(int x, int y)
{
	vertex node;
	node.x = x;
	node.y = y;
	ballList.push_back(node);
}

void pathPlanner::AStar(int goalX, int goalY)
{
	std::vector<vertex> visited;
	std::vector<vertex> unvisited;

	for (int i = 0; i < ballList.size(); i++)
	{
		unvisited.push_back(ballList.at(i));
	}

	for (int i = 0; i < visited.size(); i++)
	{
		//Manhattan distance
		unvisited.at(i).h = abs(goalX - ballList.at(i).x) + abs(goalY - ballList.at(i).y);
	}

	unvisited.at(0).currentNode = 1;
	unvisited.at(0).previousNode - 1;
	unvisited.at(0).g = 0;
	unvisited.at(0).f = unvisited.at(0).g + unvisited.at(0).h;

	visited.push_back(unvisited.at(0));
	unvisited.erase(unvisited.begin());

	int parentVertex = unvisited.at(0).currentNode;

	while (!unvisited.empty())
	{
		for (int i = 0; i < unvisited.size(); i++)
		{
			if (parentVertex == unvisited.at(i).previousNode)
			{

			}
		}


	}

}
