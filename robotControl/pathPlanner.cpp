#include "pathPlanner.h"

pathPlanner::pathPlanner()
{

}

void pathPlanner::doBrushfire()
{
    // map
        cv::Mat smallMap;

        smallMap = cv::imread("../robotControl/floor_plan.png",CV_LOAD_IMAGE_GRAYSCALE);

        char charMap[smallMap.rows][smallMap.cols];

        std::cout << smallMap.rows << ":" << smallMap.cols << std::endl;

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

        std::cout << "Linje 42" << std::endl;

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


        for (int i = 0;i<smallMap.rows;i++)
        {
            for (int j = 0;j<smallMap.cols;j++)
            {
                std::cout << charMap[i][j];
            }
            std::cout << std::endl;
        }

        std::cout << "Counter " << counter << std::endl;







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

            cv::imshow("Brushfire_before",im_brushfire);

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


        cv::imshow("Brushfire",im_brushfire);

}


