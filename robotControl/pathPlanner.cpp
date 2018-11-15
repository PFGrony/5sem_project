#include "pathPlanner.h"

pathPlanner::pathPlanner()
{
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


//Wavefront

std::deque<pair> pathPlanner::getWavefrontRoute()
{
    return routelist;
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
        pair pos = queueAdj.front();
        int counter = (newMap[pos.y][pos.x]) + 1;
        //std::cout << "x: " << pos.x << " y: " << pos.y << " Val: "<<counter<<std::endl;

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


//A Star
void pathPlanner::voronoiDiagram()
{

    cv::Mat voronoiDiagram=cv::imread("../robotControl/floor_plan.png",CV_LOAD_IMAGE_ANYCOLOR);
    cv::Mat copyVoronoi=voronoiDiagram.clone();
    bool valueChanged=true;
    cv::Vec3b counter(0,0,0);
    cv::Vec3b offset(18,18,18); //For colouring

    while(valueChanged)
    {
        valueChanged=false;


        for(int i=0;i<voronoiDiagram.rows;i++) //Rows
        {
            for(int j=0; j<voronoiDiagram.cols;j++) //Columns
            {
                if(voronoiDiagram.at<cv::Vec3b>(i,j) == counter)
                {
                    cv::Vec3b temp;
                    //N
                    if(i-1 >= 0) //Checks out-of-bounds
                    {
                        temp=voronoiDiagram.at<cv::Vec3b>(i-1,j);
                        if(temp == cv::Vec3b(255,255,255)) //Change value to the left
                        {
                            valueChanged=true;
                            voronoiDiagram.at<cv::Vec3b>(i-1,j) = counter+offset;
                        }
//                        else if(temp == counter+offset && temp[1]>0)
//                        {
////                            valueChanged=true;
//                            copyVoronoi.at<cv::Vec3b>(i-1,j) = cv::Vec3b(255,0,255);
//                        }
                    }
                    //S
                    if(i+1 < voronoiDiagram.rows) //Checks out-of-bounds
                    {
                        temp=voronoiDiagram.at<cv::Vec3b>(i+1,j);
                        if( temp == cv::Vec3b(255,255,255)) //Change value to the right
                        {
                            valueChanged=true;
                            voronoiDiagram.at<cv::Vec3b>(i+1,j) = counter+offset;
                        }
//                        else if(temp == counter+offset&& temp[1]>0)
//                        {
////                            valueChanged=true;
//                            copyVoronoi.at<cv::Vec3b>(i+1,j) = cv::Vec3b(255,0,255);
//                        }
                    }
                    //W
                    if(j-1 >= 0) //Checks out-of-bounds
                    {
                        temp=voronoiDiagram.at<cv::Vec3b>(i,j-1);
                        if( temp == cv::Vec3b(255,255,255)) //Change value above
                        {
                            valueChanged=true;
                            voronoiDiagram.at<cv::Vec3b>(i,j-1) = counter+offset;
                        }
//                        else if(temp == counter+offset&& temp[1]>0)
//                        {
//    //                            valueChanged=true;
//                            copyVoronoi.at<cv::Vec3b>(i,j-1) = cv::Vec3b(255,0,255);
//                        }
                    }
                    //E
                    if(j+1 < voronoiDiagram.cols) //Checks out-of-bounds
                    {
                        temp=voronoiDiagram.at<cv::Vec3b>(i,j+1);
                        if( temp == cv::Vec3b(255,255,255)) //Change value below
                        {
                            valueChanged=true;
                            voronoiDiagram.at<cv::Vec3b>(i,j+1) = counter+offset;
                        }
//                        else if(temp == counter+offset&& temp[1]>0)
//                        {
////                            valueChanged=true;
//                            copyVoronoi.at<cv::Vec3b>(i,j+1) = cv::Vec3b(255,0,255);
//                        }

                    }

                }
            }
        }

        counter+=offset;
    }

//    voronoiDiagram=copyVoronoi.clone();

    cv::resize(voronoiDiagram, voronoiDiagram, cv::Size(), 8, 8, cv::INTER_NEAREST);
    map=voronoiDiagram.clone();
}


void pathPlanner::doBrushfire()
{
    // map




//        char charMap[smallMap.rows][smallMap.cols];

//        std::cout << smallMap.rows << ":" << smallMap.cols << std::endl;

        for (int i = 0;i<smallMap.rows;i++)
        {
            for (int j = 0;j<smallMap.cols;j++)
            {
                if (smallMap.at<uchar>(i,j) == 0)
                    charMap[i][j] = '1';
                else
                    charMap[i][j] = '0';
            }
        }



        //Check before Brushfire
        for (int i = 0;i<smallMap.rows;i++)
        {
            for (int j = 0;j<smallMap.cols;j++)
            {
                std::cout << charMap[i][j];
            }
            std::cout << std::endl;
        }

//        std::cout << "Linje 42" << std::endl;

        //Brushfire
        int valueChanged=true;
        int counter=0;
        while(valueChanged)
        {
            valueChanged=false;
            counter++;

            for(int i=0;i<smallMap.rows;i++) //Rows
            {
                for(int j=0; j<smallMap.cols;j++) //Columns
                {
                    if(charMap[i][j] - '0' == counter)
                    {
                        if(i-1 >= 0) //Checks out-of-bounds
                        {
                            if(charMap[i-1][j] - '0'== 0) //Change value to the left
                            {
                                valueChanged=true;
                                charMap[i-1][j] = '0'+counter+1;
                            }
                        }

                        if(i+1 <= smallMap.rows) //Checks out-of-bounds
                        {
                            if(charMap[i+1][j] - '0' == 0) //Change value to the right
                            {
                                valueChanged=true;
                                charMap[i+1][j] = '0'+counter+1;
                            }
                        }

                        if(j-1 >= 0) //Checks out-of-bounds
                        {
                            if(charMap[i][j-1] - '0'== 0) //Change value above
                            {
                                valueChanged=true;
                                charMap[i][j-1] = '0'+counter+1;
                            }
                        }

                        if(j+1 <= smallMap.cols) //Checks out-of-bounds
                        {
                            if(charMap[i][j+1] - '0'== 0) //Change value below
                            {
                                valueChanged=true;
                                charMap[i][j+1] = '0'+counter+1;
                            }
                        }

                    }
                }
            }
        }


//        for (int i = 0;i<smallMap.rows;i++)
//        {
//            for (int j = 0;j<smallMap.cols;j++)
//            {
//                std::cout << charMap[i][j];
//            }
//            std::cout << std::endl;
//        }

//        std::cout << "Counter " << counter << std::endl;



        //Brushfire - With openCV
        cv::Mat1b im_brushfire=cv::imread("../robotControl/floor_plan.png",CV_LOAD_IMAGE_GRAYSCALE);

        valueChanged=true;
        counter=0;
        int offset=18;
        while(valueChanged)
        {
            valueChanged=false;


            for(int i=0;i<smallMap.rows;i++) //Rows
            {
                for(int j=0; j<smallMap.cols;j++) //Columns
                {
                    if(im_brushfire[i][j] == counter)
                    {
                        if(i-1 >= 0) //Checks out-of-bounds
                        {
                            if(im_brushfire[i-1][j] == 255) //Change value to the left
                            {
                                valueChanged=true;
                                im_brushfire[i-1][j] = counter+offset;
                            }
                        }

                        if(i+1 <= smallMap.rows) //Checks out-of-bounds
                        {
                            if(im_brushfire[i+1][j]  == 255) //Change value to the right
                            {
                                valueChanged=true;
                                im_brushfire[i+1][j] = counter+offset;
                            }
                        }

                        if(j-1 >= 0) //Checks out-of-bounds
                        {
                            if(im_brushfire[i][j-1] == 255) //Change value above
                            {
                                valueChanged=true;
                                im_brushfire[i][j-1] = counter+offset;
                            }
                        }

                        if(j+1 <= smallMap.cols) //Checks out-of-bounds
                        {
                            if(im_brushfire[i][j+1] == 255) //Change value below
                            {
                                valueChanged=true;
                                im_brushfire[i][j+1] = counter+offset;
                            }
                        }

                    }
                }
            }

            counter+=offset;
        }

        cv::resize(im_brushfire, im_brushfire, cv::Size(), 8, 8, cv::INTER_NEAREST);
        map=im_brushfire.clone();

}


void pathPlanner::AStar(pair start, pair goal)
{
    std::vector<vertex> closedSet;
    std::vector<vertex> openSet;
//    cv::Mat cameFrom = cv::imread("../robotControl/floor_plan.png", CV_LOAD_IMAGE_GRAYSCALE);
//    cv::Mat gScore=cameFrom.clone();

    double gScore[smallMap.rows][smallMap.cols];

    for (int i = 0; i < smallMap.rows; i++)
    {
        for (int j = 0; j < smallMap.cols; j++)
        {
            if (smallMap.at<uchar>(i, j) == 0)
                gScore[i][j] = 1;
            else
                gScore[i][j] =INFINITY;
        }
    }


}
