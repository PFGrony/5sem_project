#include "mapPlanning.h"


mapPlanning::mapPlanning(std::string path)
{
	map = cv::imread(path, CV_LOAD_IMAGE_ANYCOLOR);
	cv::cvtColor(map, grayMap, CV_BGR2GRAY);

	intMap = new int*[map.rows];

	for (int i = 0; i < map.rows; i++)
		intMap[i] = new int[map.cols];

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

mapPlanning::~mapPlanning()
{
}

void mapPlanning::setImgPath(std::string path)
{
	map = cv::imread(path, CV_LOAD_IMAGE_ANYCOLOR);
	cv::cvtColor(map, grayMap, CV_BGR2GRAY);

	intMap = new int*[map.rows];

	for (int i = 0; i < map.rows; i++)
		intMap[i] = new int[map.cols];

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

void mapPlanning::showMap()
{
	cv::namedWindow("Map", CV_WINDOW_FREERATIO);
    cv::imshow("Map", mapWithPaths);
    cv::waitKey(0);
	cv::destroyWindow("Map");
}

void mapPlanning::calculateMap()
{
	findCriticalPoints();
	planMap();
}

int mapPlanning::getCrits()
{
	return critPoints;
}

int mapPlanning::getPathsCount()
{
	return pathVec.size();
}

std::vector<paths> mapPlanning::getPathVec()
{
	return pathVec;
}

void mapPlanning::findCriticalPoints()
{
	///Brushfire
	cv::Mat im_brushfire;
	cv::cvtColor(map, im_brushfire, CV_BGR2GRAY);

	int valueChanged = true;
	int counter = 0;
	int offset = 18;
	while (valueChanged)
	{
		valueChanged = false;


		for (int i = 0; i < grayMap.rows; i++) //Rows
		{
			for (int j = 0; j < grayMap.cols; j++) //Columns
			{
				if (im_brushfire.at<uchar>(i, j) == counter)
				{
					if (i - 1 >= 0) //Checks out-of-bounds
					{
						if (im_brushfire.at<uchar>(i - 1, j) == 255) //Change value to the left
						{
							valueChanged = true;
							im_brushfire.at<uchar>(i - 1, j) = counter + offset;
						}
					}

					if (i + 1 < grayMap.rows) //Checks out-of-bounds
					{
						if (im_brushfire.at<uchar>(i + 1, j) == 255) //Change value to the right
						{
							valueChanged = true;
							im_brushfire.at<uchar>(i + 1, j) = counter + offset;
						}
					}

					if (j - 1 >= 0) //Checks out-of-bounds
					{
						if (im_brushfire.at<uchar>(i, j - 1) == 255) //Change value above
						{
							valueChanged = true;
							im_brushfire.at<uchar>(i, j - 1) = counter + offset;
						}
					}

					if (j + 1 < grayMap.cols) //Checks out-of-bounds
					{
						if (im_brushfire.at<uchar>(i, j + 1) == 255) //Change value below
						{
							valueChanged = true;
							im_brushfire.at<uchar>(i, j + 1) = counter + offset;
						}
					}

				}
			}
		}

		counter += offset;
	}

	///Finds local maximas
	cv::Mat maxims(im_brushfire.size(), im_brushfire.type()); // container for all local maximums
	cv::dilate(im_brushfire, maxims, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(double(map.cols / THRESHOLD), double(map.rows / THRESHOLD))));
	cv::compare(im_brushfire, maxims, points, CV_CMP_GE);



    //
    points.convertTo(points, CV_8U);
    cv::bitwise_and(points, points, points);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(points, contours, hierarchy,
        CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    cv::cvtColor(points, points, CV_GRAY2BGR);
    for (int contour = 0; (contour < contours.size()); contour++)
    {
        cv::Scalar colour(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF);
        cv::drawContours(points, contours, contour, colour,
            CV_FILLED, 8, hierarchy);
    }

    std::vector<cv::Point> liste;

    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::Point temp = cv::Point(0, 0);
        for (size_t j = 0; j < contours.at(i).size(); j++)
        {
            temp += contours.at(i).at(j);
                std::cout << contours.at(i).at(j) << std::endl;

            //points.at<cv::Vec3b>(contours.at(i).at(j).y, contours.at(i).at(j).x) = cv::Vec3b(255, 255, 255);
        }

        temp.x = int(temp.x/contours.at(i).size());
        temp.y = int(temp.y/contours.at(i).size());
            std::cout <<"result: "<< temp << std::endl;
        liste.push_back(temp);
    }

    criticalPoints.clear();
    for(int i=0;i<liste.size();i++)
    {
        criticalPoints.push_back({liste.at(i).x,liste.at(i).y});
    }

	im_brushfire = map.clone();

	for (size_t i = 0; i < criticalPoints.size(); i++)
	{
		im_brushfire.at<cv::Vec3b>(criticalPoints.at(i).y, criticalPoints.at(i).x) = cv::Vec3b(0, 255, 0);
	}


	mapWithCrits = im_brushfire.clone();
	mapWithPaths = im_brushfire.clone();
	critPoints = criticalPoints.size();
}

void mapPlanning::planMap()
{
	if (!connectedPoints.empty())
		return;

	int **copyIntMap;

	connectedPoints.push_back(criticalPoints.front());

	criticalPoints.erase(criticalPoints.begin());

	// Main while loop
	while (!criticalPoints.empty())
	{
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

		copyIntMap = intMap;
        std::queue<coordinate> frontier;
		frontier.push(connectedPoints.back());
		bool pointUnConnected = true;

		// first brushfire
		while (!frontier.empty() && pointUnConnected)
		{
			coordinate current = frontier.front();
			frontier.pop();

			for (size_t i = 0; i < criticalPoints.size(); i++)
			{
				if (current.x == criticalPoints[i].x && current.y == criticalPoints[i].y)
				{
					connectedPoints.push_back(criticalPoints[i]);
					criticalPoints.erase(criticalPoints.begin() + i);
					pointUnConnected = false;
				}
			}

			int counter = (copyIntMap[current.y][current.x]) + 1;
			coordinate neighbour;
			int connectivity = 4;
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

				if (copyIntMap[neighbour.y][neighbour.x] == 0)
				{
					frontier.push(neighbour);
					copyIntMap[neighbour.y][neighbour.x] = counter;
					//cameFrom.at(neighbour.y).at(neighbour.x) = current;
				}
			}
		}

		pointUnConnected = true;
		copyIntMap = intMap;

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

		int ToDistance = 1000;
		double distance = 0;
		bool multipleCon = false;

		frontier = std::queue<coordinate>();
		frontier.push(connectedPoints.back());
		coordinate current = frontier.front();
        std::vector<coordinate> connectionsVec;
		int counter = (copyIntMap[current.y][current.x]) + 1;

		// second brushfire
		while (!frontier.empty() && ToDistance > counter /*&& connectionsVec.size() < 2*/)
		{
			current = frontier.front();
			counter = (copyIntMap[current.y][current.x]) + 1;
			frontier.pop();

			if (ToDistance == 1000)
			{
				for (size_t i = 0; i < connectedPoints.size() - 1; i++)
				{
					if (current.x == connectedPoints[i].x && current.y == connectedPoints[i].y)
					{
						connectionsVec.push_back(current);
						distance = sqrt(pow(current.x - connectedPoints[i].x, 2) + pow(current.y - connectedPoints[i].y, 2));
						ToDistance = counter * 1.5;
					}
				}
			}
			else
			{
				for (size_t i = 0; i < connectedPoints.size() - 1; i++)
				{
					if (current.x == connectedPoints[i].x && current.y == connectedPoints[i].y)
					{
						double distanceBetweenConnection = sqrt(pow(connectionsVec[0].x - connectedPoints[i].x, 2) + pow(connectionsVec[0].y - connectedPoints[i].y, 2));
						if (counter*0.5 < distanceBetweenConnection)
							connectionsVec.push_back(current);
					}
				}
			}
			coordinate neighbour;
			int connectivity = 4;
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

				if (copyIntMap[neighbour.y][neighbour.x] == 0)
				{
					frontier.push(neighbour);
					copyIntMap[neighbour.y][neighbour.x] = counter;
				}
			}
		}
		planPath(connectionsVec);
	}
}

void mapPlanning::planPath(std::vector<coordinate> conVec)
{
	int **copyIntMap = intMap;

	for (int i = 0; i < conVec.size(); i++)
	{
		coordinate current = conVec[i];

		int length = 0;
		std::deque<coordinate> path;
		coordinate neighbour;
		bool breakNow = false;
		size_t k = 0;

		while (true)
		{
			int connectivity = 4;
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

				if (copyIntMap[current.y][current.x] == 1)
				{
					k = 0;
					for (; k < connectedPoints.size(); k++)
					{
						if (neighbour.x == connectedPoints[k].x && neighbour.y == connectedPoints[k].y)
						{
							breakNow = true;
						}
						if (breakNow)
							break;
					}
				}

				if (copyIntMap[neighbour.y][neighbour.x] == copyIntMap[current.y][current.x] - 1)
				{
					length++;
					current = neighbour;
					path.push_front(neighbour);
					break;
				}
				if (breakNow)
					break;

			}


			if (breakNow)
				break;
		}


		pathVec.push_back(paths{ connectedPoints[k],conVec[i],length,path });
		//cout << start.x << ":" << start.y << "; " << goal.x << ":" << goal.y << "; " << length << endl;
		for (int i = 0; i < path.size(); i++)
		{
			mapWithPaths.at<cv::Vec3b>(path.at(i).y, path.at(i).x) = cv::Vec3b(0, 20, 255);
		}
		//showMap();
	}

}
