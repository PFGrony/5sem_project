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
                    charMap[i][j] = ' ';
            }
        }

        //Brushfire
        int valueChanged=true;
        while(valueChanged)
        {
            valueChanged=false;
            for(int i=0;i<smallMap.rows;i++) //Rows
            {
                for(int j=0; j<smallMap.rows;j++) //Columns
                {
                    if(charMap[i][j] == ' ') //Checks if "pixel" is empty and should be altered
                    {
                        valueChanged=true;


                    }
                }
            }
        }

}
