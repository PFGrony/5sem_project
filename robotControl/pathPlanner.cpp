#include "pathPlanner.h"

pathPlanner::pathPlanner(std::string path)
{
	cv::cvtColor(map, grayMap, CV_BGR2GRAY);

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

cv::Mat pathPlanner::getMap()
{
	return map;
}

void pathPlanner::printCameFrom()
{
	for (int i = 0; i < map.rows; i++)
	{
		for (int j = 0; j < map.cols; j++)
		{
			std::cout << cameFrom.at(i).at(j).x;
		}
		std::cout << std::endl;
	}
}

//Wavefront

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

	for (size_t i = 0; i < map.rows; i++)
	{
		for (size_t j = 0; j < map.cols; j++)
		{
			if (i == start.y && j == start.x)
				std::cout << 'S';
			else if (i == goal.y && j == goal.x)
				std::cout << 'G';
			else if (cameFromMap[i][j] == 0)
				std::cout << ' ';
			else if (cameFromMap[i][j] == 1)
				std::cout << '#';
			else
			{
				pair temp = cameFrom.at(i).at(j);
				if (i < temp.y)
					std::cout << 'V';
				else if (i > temp.y)
					std::cout << '^';
				else if (j < temp.x)
					std::cout << '>';
				else if (j > temp.x)
					std::cout << '<';
			}
			//std::cout << char(cameFromMap[i][j] % 8 + '2');
		}
		std::cout << std::endl;
	}

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

		int counter = (cameFromMap[current.y][current.x]) + 1;
		//std::cout << "x: " << current.x << " y: " << current.y << " Val: "<<counter<<std::endl;

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

			if (cameFromMap[neighbour.y][neighbour.x] == 0)
			{
				neighbour.h = (double)abs(neighbour.x - goal.x) + (double)abs(neighbour.y - goal.y);
				frontier.push(neighbour);
				cameFromMap[neighbour.y][neighbour.x] = counter;
				cameFrom.at(neighbour.y).at(neighbour.x) = { current.x,current.y };
			}
		}


	}

	for (size_t i = 0; i < map.rows; i++)
	{
		for (size_t j = 0; j < map.cols; j++)
		{
			if (i == start.y && j == start.x)
				std::cout << 'S';
			else if (i == goal.y && j == goal.x)
				std::cout << 'G';
			else if (cameFromMap[i][j] == 0)
				std::cout << ' ';
			else if (cameFromMap[i][j] == 1)
				std::cout << '#';
			else
			{
				pair temp = cameFrom.at(i).at(j);
				if (i < temp.y)
					std::cout << 'V';
				else if (i > temp.y)
					std::cout << '^';
				else if (j < temp.x)
					std::cout << '>';
				else if (j > temp.x)
					std::cout << '<';
			}
		}
		std::cout << std::endl;
	}
}

void pathPlanner::AStar(pair startPair, pair goalPair)
{
	double costSoFar[ROW][COL];

	for (int i = 0; i < map.rows; i++)
	{
		for (int j = 0; j < map.cols; j++)
		{
			cameFromMap[i][j] = intMap[i][j];

			if (intMap[i][j] == 1)
				costSoFar[i][j] = 1;
			else
				costSoFar[i][j] = 0;

			cameFrom.at(i).at(j) = { -1,-1 };
		}
	}

	node goal;
	pairToNode(goalPair, goal);

	std::priority_queue< node, std::vector<node>, compareCost> frontier;
	node start;
	pairToNode(startPair, start);
	frontier.push(start);

	costSoFar[startPair.y][startPair.x] = 0;
	cameFrom.at(startPair.y).at(startPair.x) = startPair;
	cameFromMap[startPair.y][startPair.x] = 2;

	while (!frontier.empty())
	{

		node current = frontier.top();
		frontier.pop();

		if (current.x == goal.x && current.y == goal.y)
			break;

		int counter = (cameFromMap[current.y][current.x]) + 1;

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


			double newCost = costSoFar[current.y][current.x] + 1;

			//Diagonal Moves Have higher cost
			if (i > 3)
				newCost++;


			if (costSoFar[neighbour.y][neighbour.x] == 0 || newCost < costSoFar[neighbour.y][neighbour.x])
			{
				costSoFar[neighbour.y][neighbour.x] = newCost;
				neighbour.f = newCost + sqrt(pow(abs(neighbour.x - goal.x), 2) + pow(abs(neighbour.y - goal.y), 2));
				frontier.push(neighbour);
				cameFromMap[neighbour.y][neighbour.x] = counter;
				cameFrom.at(neighbour.y).at(neighbour.x) = { current.x,current.y };
				//std::cout << "(" << neighbour.x << "," << neighbour.y << ")" << " f: " << neighbour.f << std::endl;
			}
		}
	}

	for (size_t i = 0; i < map.rows; i++)
	{
		for (size_t j = 0; j < map.cols; j++)
		{
			if (i == start.y && j == start.x)
				std::cout << 'S';
			else if (i == goal.y && j == goal.x)
				std::cout << 'G';
			else if (cameFromMap[i][j] == 0)
				std::cout << ' ';
			else if (cameFromMap[i][j] == 1)
				std::cout << '#';
			else
			{

				pair temp = cameFrom.at(i).at(j);
				if (i < temp.y)
					std::cout << 'V';
				else if (i > temp.y)
					std::cout << '^';
				else if (j < temp.x)
					std::cout << '>';
				else if (j > temp.x)
					std::cout << '<';
			}
		}
		std::cout << std::endl;
	}
}



//Get Planned Path
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

	routelist = path;
	return path;
}

void pathPlanner::drawPath(pair start, pair goal)
{
	mapCopy = map.clone();

	if (routelist.empty())
		std::cout << "No path, try executing getPath()" << std::endl;

	for (int i = 0; i < routelist.size(); i++)
	{
		mapCopy.at<cv::Vec3b>(routelist.at(i).y, routelist.at(i).x) = cv::Vec3b(0, 255, 0);
	}
	//Start
	mapCopy.at<cv::Vec3b>(start.y, start.x) = cv::Vec3b(0, 0, 255);

	//Goal
	mapCopy.at<cv::Vec3b>(goal.y, goal.x) = cv::Vec3b(255, 0, 0);
}


//Critical Points
void pathPlanner::AGP()
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

	///Sweeps to find centers of local maximas
	std::vector<pair> Highest1;
	std::vector<pair> Highest2;


	//Not ends included
	std::vector<barVal> bar(map.cols);

	//Horizontal bar sweep
	for (int i = 0; i < map.rows; i++)
	{
		for (int j = 0; j < map.cols; j++)
		{
			if (points.at<uchar>(i, j) == 0 && bar.at(j).seen)
			{
				if (bar.at(j).amount > 1)
					bar.at(j).pairs.y -= (bar.at(j).amount / 2);
				Highest1.push_back(bar.at(j).pairs);
				bar.at(j).seen = false;
				bar.at(j).amount = 1;
			}
			else if (points.at<uchar>(i, j) > 0)
			{
				if (!bar.at(j).seen)
				{
					bar.at(j).val = points.at<uchar>(i, j);
					bar.at(j).pairs = { j,i };
					bar.at(j).amount = 1;
				}
				else if (points.at<uchar>(i, j) == bar.at(j).val)
				{
					bar.at(j).amount++;
					bar.at(j).pairs = { j,i };
				}
				bar.at(j).seen = true;
			}
		}
	}

	bar.clear();
	bar.resize(map.rows);

	//Vertical bar sweep
	for (int i = 0; i < map.rows; i++)
	{
		for (int j = 0; j < map.cols; j++)
		{
			if (points.at<uchar>(i, j) == 0 && bar.at(i).seen)
			{
				if (bar.at(i).amount > 1)
					bar.at(i).pairs.x -= (bar.at(i).amount / 2);
				Highest2.push_back(bar.at(i).pairs);
				bar.at(i).seen = false;
				bar.at(i).amount = 1;
			}
			else if (points.at<uchar>(i, j) > 0)
			{
				if (!bar.at(i).seen)
				{
					bar.at(i).val = points.at<uchar>(i, j);
					bar.at(i).pairs = { j,i };
					bar.at(i).amount = 1;
				}
				else if (points.at<uchar>(i, j) == bar.at(i).val)
				{
					bar.at(i).amount++;
					bar.at(i).pairs = { j,i };
				}
				bar.at(i).seen = true;
			}
		}
	}

	///Find intersections
	cv::cvtColor(points, points, CV_GRAY2BGR);
	for (size_t i = 0; i < Highest1.size(); i++)
	{
		//std::cout << Highest1.at(i).x << " " << Highest1.at(i).y << std::endl;
		points.at<cv::Vec3b>(Highest1.at(i).y, Highest1.at(i).x) = cv::Vec3b(0, 0, 255);
	}
	for (size_t i = 0; i < Highest2.size(); i++)
	{
		//std::cout << Highest.at(i).x << " " << Highest.at(i).y << std::endl;
		if (points.at<cv::Vec3b>(Highest2.at(i).y, Highest2.at(i).x) == cv::Vec3b(0, 0, 255))
		{
			points.at<cv::Vec3b>(Highest2.at(i).y, Highest2.at(i).x) = cv::Vec3b(0, 255, 0);
		}
		else
			points.at<cv::Vec3b>(Highest2.at(i).y, Highest2.at(i).x) = cv::Vec3b(255, 0, 0);
	}

	/// reduce clusters of intersections
	for (int i = 0; i < map.rows; i++)
	{
		for (int j = 0; j < map.cols; j++)
		{

			if (points.at<cv::Vec3b>(i, j) == cv::Vec3b(0, 255, 0))
			{
				for (int k = 0; k < 8; k++)
				{

					pair neighbour;
					if (k == 0)
						neighbour = { j, i - 1 };//N
					else if (k == 1)
						neighbour = { j + 1, i };//E
					else if (k == 2)
						neighbour = { j, i + 1 };//S
					else if (k == 3)
						neighbour = { j - 1, i };//W
					else if (k == 4)
						neighbour = { j + 1, i - 1 };//NE
					else if (k == 5)
						neighbour = { j + 1, i + 1 };//SE
					else if (k == 6)
						neighbour = { j - 1, i + 1 };//SW
					else if (k == 7)
						neighbour = { j - 1, i - 1 };//NW

					if (neighbour.x < 0 || neighbour.y < 0 || neighbour.x >= map.cols || neighbour.y >= map.rows)
						continue;

					if (points.at<cv::Vec3b>(neighbour.y, neighbour.x) == cv::Vec3b(0, 255, 0))
					{
						points.at<cv::Vec3b>(neighbour.y, neighbour.x) = cv::Vec3b(255, 255, 0);
					}

				}

			}
		}
	}

	/// add critical maximas
	for (int i = 0; i < map.rows; i++)
	{
		for (int j = 0; j < map.cols; j++)
		{
			if (points.at<cv::Vec3b>(i, j) == cv::Vec3b(0, 255, 0))
			{
				criticalPoints.push_back({ j,i });
			}
		}
	}

	im_brushfire = map.clone();

	for (size_t i = 0; i < criticalPoints.size(); i++)
	{
		im_brushfire.at<cv::Vec3b>(criticalPoints.at(i).y, criticalPoints.at(i).x) = cv::Vec3b(0, 255, 0);
	}
	mapCopy = im_brushfire.clone();
	mapWithPaths = im_brushfire.clone();

}

std::deque<pair> pathPlanner::getCriticalPoints()
{
	return criticalPoints;
}

void pathPlanner::drawGraph()
{

	if (routelist.empty())
		std::cout << "No path, try executing getPath()" << std::endl;

	for (int i = 0; i < routelist.size(); i++)
	{
		mapWithPaths.at<cv::Vec3b>(routelist.at(i).y, routelist.at(i).x) = cv::Vec3b(0, 0, 255);
	}
	for (int i = 0; i < map.rows; i++)
	{
		for (int j = 0; j < map.cols; j++)
		{
			if (points.at<cv::Vec3b>(i, j) == cv::Vec3b(0, 255, 0))
				mapWithPaths.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 0);
		}
	}
}



void pathPlanner::AGPgraph()
{
		pairPair lengths;

	///Checks every Critical node
	for (size_t k = 0; k < criticalPoints.size(); k++)
	{
		//int k = 10;
		int amountFound = 0;
		///resets map for brushfire
		for (int i = 0; i < map.rows; i++)
		{
			for (int j = 0; j < map.cols; j++)
			{
				cameFromMap[i][j] = intMap[i][j];
				cameFrom.at(i).at(j) = { -1,-1 };
			}
		}
		///Brushfire init
		pair start = criticalPoints.at(k);
		lengths.start = start;


		cameFrom.at(start.y).at(start.x) = start;
		cameFromMap[start.y][start.x] = 2;

		std::queue<pair> frontier;
		frontier.push(start);


		bool existInTree = false;

		///Begin brushfire
		while (!frontier.empty())// && amountFound < 4)
		{

			pair current = frontier.front();
			frontier.pop();


			/// Check if goal
			for (size_t i = 0; i < criticalPoints.size(); i++)
			{
				//Cant connect to itself
				if (i == k)
					continue;

				if (current.x == criticalPoints.at(i).x && current.y == criticalPoints.at(i).y)
				{
					AGPnode temp;
					temp.current = start;
					lengths.goal = criticalPoints.at(i);
					temp.children.push_back({ lengths.goal,true });

					//for (size_t i = 0; i < pointsTree.size(); i++)
					//{
					//	if (pointsTree.at(i).current.x == current.x && pointsTree.at(i).current.y == current.y)
					//	{
					//		pointsTree.at(i).children.push_back(temp.children.front());
					//		amountFound++;
					//		bool existInTree = true;
					//		break;
					//	}
					//}


					if (!existInTree)
					{
						amountFound++;


						pointsTree.push_back(temp);
						getPath(lengths.start, lengths.goal);
						drawGraph();
					}
					existInTree = false;
				}
			}

			///Brushfire map
			int counter = (cameFromMap[current.y][current.x]) + 1;
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
}





//void pathPlanner::doBrushfire()
//{
//	// map
//
//
//
//
//	char charMap[ROW][COL];
//
//	//        std::cout << grayMap.rows << ":" << grayMap.cols << std::endl;
//
//	for (int i = 0; i < grayMap.rows; i++)
//	{
//		for (int j = 0; j < grayMap.cols; j++)
//		{
//			if (grayMap.at<uchar>(i, j) == 0)
//				charMap[i][j] = '1';
//			else
//				charMap[i][j] = '0';
//		}
//	}
//
//
//
//	////Check before Brushfire
//	//for (int i = 0; i < grayMap.rows; i++)
//	//{
//	//	for (int j = 0; j < grayMap.cols; j++)
//	//	{
//	//		std::cout << charMap[i][j];
//	//	}
//	//	std::cout << std::endl;
//	//}
//
//	//        std::cout << "Linje 42" << std::endl;
//
//			//Brushfire
//	int valueChanged = true;
//	int counter = 0;
//	while (valueChanged)
//	{
//		valueChanged = false;
//		counter++;
//
//		for (int i = 0; i < grayMap.rows; i++) //Rows
//		{
//			for (int j = 0; j < grayMap.cols; j++) //Columns
//			{
//				if (charMap[i][j] - '0' == counter)
//				{
//					if (i - 1 >= 0) //Checks out-of-bounds
//					{
//						if (charMap[i - 1][j] - '0' == 0) //Change value to the left
//						{
//							valueChanged = true;
//							charMap[i - 1][j] = '0' + counter + 1;
//						}
//					}
//
//					if (i + 1 <= grayMap.rows) //Checks out-of-bounds
//					{
//						if (charMap[i + 1][j] - '0' == 0) //Change value to the right
//						{
//							valueChanged = true;
//							charMap[i + 1][j] = '0' + counter + 1;
//						}
//					}
//
//					if (j - 1 >= 0) //Checks out-of-bounds
//					{
//						if (charMap[i][j - 1] - '0' == 0) //Change value above
//						{
//							valueChanged = true;
//							charMap[i][j - 1] = '0' + counter + 1;
//						}
//					}
//
//					if (j + 1 <= grayMap.cols) //Checks out-of-bounds
//					{
//						if (charMap[i][j + 1] - '0' == 0) //Change value below
//						{
//							valueChanged = true;
//							charMap[i][j + 1] = '0' + counter + 1;
//						}
//					}
//
//				}
//			}
//		}
//	}
//
//
//	for (int i = 0; i < grayMap.rows; i++)
//	{
//		for (int j = 0; j < grayMap.cols; j++)
//		{
//			std::cout << charMap[i][j];
//		}
//		std::cout << std::endl;
//	}
//
//	std::cout << "Counter " << counter << std::endl;
//
//
//
//	//Brushfire - With openCV
//	cv::Mat im_brushfire = grayMap.clone();
//
//	valueChanged = true;
//	counter = 0;
//	int offset = 18;
//	while (valueChanged)
//	{
//		valueChanged = false;
//
//
//		for (int i = 0; i < grayMap.rows; i++) //Rows
//		{
//			for (int j = 0; j < grayMap.cols; j++) //Columns
//			{
//				if (im_brushfire.at<uchar>(i, j) == counter)
//				{
//					if (i - 1 >= 0) //Checks out-of-bounds
//					{
//						if (im_brushfire.at<uchar>(i - 1, j) == 255) //Change value to the left
//						{
//							valueChanged = true;
//							im_brushfire.at<uchar>(i - 1, j) = counter + offset;
//						}
//					}
//
//					if (i + 1 < grayMap.rows) //Checks out-of-bounds
//					{
//						if (im_brushfire.at<uchar>(i + 1, j) == 255) //Change value to the right
//						{
//							valueChanged = true;
//							im_brushfire.at<uchar>(i + 1, j) = counter + offset;
//						}
//					}
//
//					if (j - 1 >= 0) //Checks out-of-bounds
//					{
//						if (im_brushfire.at<uchar>(i, j - 1) == 255) //Change value above
//						{
//							valueChanged = true;
//							im_brushfire.at<uchar>(i, j - 1) = counter + offset;
//						}
//					}
//
//					if (j + 1 < grayMap.cols) //Checks out-of-bounds
//					{
//						if (im_brushfire.at<uchar>(i, j + 1) == 255) //Change value below
//						{
//							valueChanged = true;
//							im_brushfire.at<uchar>(i, j + 1) = counter + offset;
//						}
//					}
//
//				}
//			}
//		}
//
//		counter += offset;
//	}
//
//	//cv::cvtColor(im_brushfire, im_brushfire, CV_GRAY2RGB);
//
//	//for (int i = 0; i < grayMap.rows; i++)
//	//{
//	//	for (int j = 0; j < grayMap.cols; j++)
//	//	{
//	//		if (im_brushfire.at<cv::Vec3b>(i, j)[1] > 120)
//	//			im_brushfire.at<cv::Vec3b>(i, j)= cv::Vec3b(0, 0, 255);
//	//	}
//	//}
//	cv::resize(im_brushfire, im_brushfire, cv::Size(), 8, 8, cv::INTER_NEAREST);
//	map = im_brushfire.clone();
//
//}

