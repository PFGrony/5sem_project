#include "pathPlanner.h"

pathPlanner::pathPlanner(std::string path)
{
    map = cv::imread(path, CV_LOAD_IMAGE_ANYCOLOR);
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
