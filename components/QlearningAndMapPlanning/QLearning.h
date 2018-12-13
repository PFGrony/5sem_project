#include "mapPlanning.h"
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <list>
#include <queue>

#pragma once

using namespace std;

struct QlPoints
{
	double x;
	double y;
};

struct state
{
	vector<state *> possibleStates;
	vector<int> cost;
	QlPoints posStates;
	int roomNumber;
};

struct moves
{
	state * s;
	int a;
	int battery;
};

struct valueState
{
	int state;
	int action;
	double value;
};

class QLearning
{
public:
	QLearning();
	~QLearning();

	void runQLearning();

	QlPoints getPoint(int x);

	void printBestActions();

	void loadTestWorld();
	void importMap(vector<paths> pathVec);

	void setE(int x);
	void setDiscountRate(double x);
	void setLearningRate(double x);
	void setRun(int x);

	int getMarblesFound();

	void printAiList();

	int getBestSize();
private:
	void calculateaiTable();

	void setAiList();
	void insertValueAiList(int room, int state, int action, double value);
	double getValueAiList(int room, int state, int action);

	void loadBigWorld();

	void setExploration();

	state * getNextState(state* s, int a);
	int getReward(state* s, int a);
	int getNextAction(state* s);

	int getNextTableAction(state * s);

	double learningRate;
	double discountRate;

	double theta;

	int runs;

	int e;

	int numberOfRooms;
	int startingRoom;
	int maxCost;

	int statesUsed;

	int iteration;

	vector<state> allStates;
	vector<moves> bestActions;
	vector<bool> worldUnExp;

	int numberOfMarbles;
	int finalMarblesFound;
	vector<int> marbles;

	int batteryStart;

	list<list<valueState>*> aiList;

	//double aiTable[320][10] = { };
	bool hasRun;
	bool test;
};