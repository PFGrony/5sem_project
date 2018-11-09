#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <math.h>
#include <fstream>

#pragma once

#define numberOfRooms 19

using namespace std;

struct QlPoints
{
	double x;
	double y;
};

struct state
{
	vector<state *> possibleStates;
	QlPoints posStates;
	int roomNumber;
	bool unexplored;
};

struct moves
{
	state * s;
	int a;
};


class QLearning
{
public:
	QLearning();
	~QLearning();

	void runQLearning();

	void printBestActions();
	void saveaiTable();

	void calculateaiTable();

    QlPoints getPoint(int x);
private:
	void loadRooms();
	void loadMaples();

	state * getNextState(state* s, int a);
	int getReward(state* s, int a);
	int getNextAction(state* s);

	int getMaxAction(state* s);

	int getNextTableAction(state * s);

	double learningRate;
	double discountRate;

	double theta;

	int e;

	int tMax;

	int iteration;

	vector<state> allStates;
	vector<moves> bestActions;
	double aiTable[numberOfRooms * 2][numberOfRooms * 2] = { };
	bool hasRun;
};

