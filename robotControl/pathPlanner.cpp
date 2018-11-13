#include "pathPlanner.h"

pathPlanner::pathPlanner()
{
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
}

cv::Mat pathPlanner::getMap()
{
    return map;
}


bool pathPlanner::freePlace(int x,int y)
{
    if(newMap[x][y]=='0' && newMap[x][y]!='1')
    {
        return true;
    }
    return false;
}

void pathPlanner::addAdj(std::deque<pair> queueAdj)
{
    pair pos=queueAdj.front();
    char counter=(newMap[pos.x][pos.y])+1;

    //N
    if(freePlace(pos.x,pos.y+1))
    {
        queueAdj.push_back(pair{pos.x,pos.y+1});
        newMap[pos.x][pos.y+1]=counter;
        std::cout<<pos.x << ":"<< pos.y+1 << std::endl;
    }
    //NE
    if(freePlace(pos.x+1,pos.y+1))
    {
        queueAdj.push_back(pair{pos.x+1,pos.y+1});
        newMap[pos.x+1][pos.y+1]=counter;
        std::cout<<pos.x+1 << ":"<< pos.y+1 << std::endl;
    }
    //E
    if(freePlace(pos.x+1,pos.y))
    {
        queueAdj.push_back(pair{pos.x+1,pos.y});
        newMap[pos.x+1][pos.y]=counter;
    }
    //SE
    if(freePlace(pos.x+1,pos.y-1))
    {
        queueAdj.push_back(pair{pos.x+1,pos.y-1});
        newMap[pos.x+1][pos.y-1]=counter;
    }
    //S
    if(freePlace(pos.x,pos.y-1))
    {
        queueAdj.push_back(pair{pos.x,pos.y-1});
        newMap[pos.x][pos.y-1]=counter;
    }
    //SW
    if(freePlace(pos.x-1,pos.y-1))
    {
        queueAdj.push_back(pair{pos.x-1,pos.y-1});
        newMap[pos.x-1][pos.y-1]=counter;
    }
    //W
    if(freePlace(pos.x-1,pos.y))
    {
        queueAdj.push_back(pair{pos.x-1,pos.y});
        newMap[pos.x-1][pos.y]=counter;
    }
    //NW
    if(freePlace(pos.x-1,pos.y+1))
    {
        queueAdj.push_back(pair{pos.x-1,pos.y+1});
        newMap[pos.x-1][pos.y+1]=counter;
    }
    std::cout<<"break1"<<std::endl;
}




void pathPlanner::wavefrontPlanner(int startX,int startY,int goalX,int goalY)
{

    for(int i=0;i<smallMap.rows;i++)
    {
        for(int j=0;j<smallMap.cols;j++)
        {
            newMap[i][j] = charMap[i][j];
        }
    }

    for (int i = 0;i<smallMap.rows;i++)
    {
        for (int j = 0;j<smallMap.cols;j++)
        {
            std::cout << newMap[i][j];
        }
        std::cout << std::endl;
    }



//    char charMap[smallMap.rows][smallMap.cols];

//    for (int i = 0;i<smallMap.rows;i++)
//    {
//        for (int j = 0;j<smallMap.cols;j++)
//        {
//            if (smallMap.at<uchar>(i,j) == 0)
//                charMap[i][j] = '1';
//            else
//                charMap[i][j] = '0';
//        }
//    }

    //goal
    char counter='2';
    newMap[goalX][goalY]=counter;

    std::deque<pair> queueAdj;
    queueAdj.push_back(pair{goalX,goalY});
//    addAdj(queueAdj,{goalX,goalY});


//    while(charMap[startX][startY]!='0')
//    {
//        counter++;

//        int queueSize=queueAdj.size();
//        for(int i=0;i<queueSize;i++)
//        {
//            if(charMap[queueAdj.front().x][queueAdj.front().y]=='0' && charMap[queueAdj.front().x][queueAdj.front().y]!='1')
//            {
//               charMap[queueAdj.front().x][queueAdj.front().y]=counter;
//            }
//           queueAdj.push_back(queueAdj2.at(i));
//        }
//    }

    while(queueAdj.front().x!=startX && queueAdj.front().y!=startY)
    {
        addAdj(queueAdj);
//        std::cout<<"break0"<<std::endl;
        for (int i = 0;i<smallMap.rows;i++)
        {
            for (int j = 0;j<smallMap.cols;j++)
            {
                std::cout << newMap[i][j];
            }
            std::cout << std::endl;
        }

        queueAdj.pop_front();
//        std::cout<<"break2"<<std::endl;

    }




    for (int i = 0;i<smallMap.rows;i++)
    {
        for (int j = 0;j<smallMap.cols;j++)
        {
            std::cout << newMap[i][j];
        }
        std::cout << std::endl;
    }
}

void pathPlanner::doBrushfire()
{
    // map




        char charMap[smallMap.rows][smallMap.cols];

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
        cv::Mat1b im_brushfire(80,120,255);
         im_brushfire = cv::imread("../robotControl/floor_plan.png",CV_LOAD_IMAGE_GRAYSCALE);
         for (int i = 0;i<im_brushfire.rows;i++)
         {
             for (int j = 0;j<im_brushfire.cols;j++)
             {
                 if (im_brushfire.at<uchar>(i,j) == 0)
                     im_brushfire.at<uchar>(i,j) = 0;
                 else
                     im_brushfire.at<uchar>(i,j) = 255;
             }
         }
                map=im_brushfire.clone();
                cv::resize(map, map, cv::Size(), 8, 8, cv::INTER_NEAREST);

//            cv::imshow("Brushfire_before",im_brushfire);

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


//        cv::imshow("Brushfire",im_brushfire);



}


void pathPlanner::addVertex(int x, int y)
{
    vertex node;
    node.x=x;
    node.y=y;
    ballList.push_back(node);
}

void pathPlanner::AStar(int goalX,int goalY)
{
    std::vector<vertex> visited;
    std::vector<vertex> unvisited;

    for(int i=0;i<ballList.size();i++)
    {
        unvisited.push_back(ballList.at(i));
    }

    for(int i=0;i<visited.size();i++)
    {
        //Manhattan distance
        unvisited.at(i).h=abs(goalX-ballList.at(i).x)+abs(goalY-ballList.at(i).y);
    }

    unvisited.at(0).currentNode=1;
    unvisited.at(0).previousNode-1;
    unvisited.at(0).g=0;
    unvisited.at(0).f=unvisited.at(0).g+unvisited.at(0).h;

    visited.push_back(unvisited.at(0));
    unvisited.erase(unvisited.begin());

    int parentVertex=unvisited.at(0).currentNode;

    while(!unvisited.empty())
    {
       for(int i=0;i<unvisited.size();i++)
       {
           if(parentVertex==unvisited.at(i).previousNode)
           {

           }
       }


    }

}
