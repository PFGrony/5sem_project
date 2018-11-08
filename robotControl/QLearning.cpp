#include "QLearning.h"

QLearning::QLearning()
{
	// Learning rate and discount rate determins the values of the Q(s,a)
	learningRate = 0.8;
	discountRate = 0.5;

	// the threshhold
	theta = 0.000000001;

	// how often will we get a random move (1/e)
	e = 13;

	// maximum time steps before it terminates the run
	tMax = 100;

	// how many iterations did it run
	iteration = 0;

	// if it is false it will load the rooms
	hasRun = false;
}

QLearning::~QLearning()
{
}

void QLearning::runQLearning()
{
	// load the rooms
	loadRooms();
	// setup all the need data types
	double delta			= 0;
	double tableBuffer		= 0;

	vector<moves> actions;
	vector<moves> temp(tMax);
	bestActions = temp;

	state * s;
	state * nS;


	int a					= 0;
	int roomsExplored		= 0;
	int i					= 0;
	int reward				= 0;
	int t					= 0;

	int aiTableS			= 0;
	int aiTableSA			= 0;
	int aiTablenSA			= 0;

	// Run the AI
	do {
		//reset delta
		delta = 0;
		// increase iterator
		i++;
		// reload maples / unexplored rooms
		loadMaples();
		// set starting state/room
		s = &allStates[10];
		// set to starting state aswell or else it crashes (??)
		nS = &allStates[10];
		// reset time
		t = 0;
		// reset the amout of rooms explored
		roomsExplored = 0;

		while (true)
		{
			// time of specific run increased
			t++;
			// get action
			a = getNextAction(s);
			// reward for state with the action
			reward = getReward(s, a);
			// add to list of moves for bestActions vector
			actions.push_back(moves{ s,a });
			// get next state from current state and action
			nS = getNextState(s, a);

			// caculate the needed placements on the table
			if (s->unexplored)
				aiTableS = (s->roomNumber - 1) * 2;
			else
				aiTableS = (s->roomNumber - 1) * 2 + 1;

			if (nS->unexplored)
				aiTableSA = (nS->roomNumber - 1) * 2;
			else
				aiTableSA = (nS->roomNumber - 1) * 2 + 1;

			if (getNextState(nS, getMaxAction(nS))->unexplored)
				aiTablenSA = (getNextState(nS, getMaxAction(nS))->roomNumber - 1) * 2;
			else
				aiTablenSA = (getNextState(nS, getMaxAction(nS))->roomNumber - 1) * 2 + 1;

			// Save current table value
			tableBuffer = aiTable[aiTableS][aiTableSA];

			// Caculate Q(s,a)
			aiTable[aiTableS][aiTableSA] = aiTable[aiTableS][aiTableSA] + learningRate * (getReward(s, a) + (discountRate * aiTable[aiTablenSA][aiTableSA]) - aiTable[aiTableS][aiTableSA]);

			// Save biggest diffrence
			delta = std::max(delta, (double)fabs(tableBuffer - aiTable[aiTableS][aiTableSA]));

			// Check if the state gets explored
			if (s->unexplored && reward == 1)
			{
				roomsExplored++;
				s->unexplored = false;
			}

			// Set current state to next state
			s = nS;

			// Terminate run if all rooms are explored or the time exceeds tMax
			if (roomsExplored == numberOfRooms || t > tMax)
				break;
		}
		// check if current run is better than best run
		if (actions.size() < bestActions.size())
		{
			bestActions = actions;
		}
		cout << i << ":" << delta << endl;
		// clear action vector before rerunning
		actions.clear();
		// Run again is delta is above theta
	} while (delta > theta);
	iteration = i;
}

void QLearning::printBestActions()
{
	cout << "Fastest path was done in " << bestActions.size() << " moves" << endl;
	for (int i = 0; i < bestActions.size(); i++)
	{
		if (bestActions[i].a == 0)
			cout << "Seach in room " << bestActions[i].s->roomNumber << endl;
		else
			cout << "Move from room " << bestActions[i].s->roomNumber << " to room " << bestActions[i + 1].s->roomNumber << endl;
	}
}

void QLearning::saveaiTable()
{
	ofstream myfile;
	myfile.open("aiTable.txt");
	myfile << "s/a" << "\t";
	for (int i = 0; i < 38; i++)
		myfile << "Room" << ((i % 2 == 0) ? " " : "E") << ((i / 2) + 1) << "\t";
	myfile << endl;

	for (int i = 0; i < 38; i++)
	{
		myfile << "Room" << ((i % 2 == 0) ? " " : "E") << ((i / 2) + 1) << "\t";
		for (int j = 0; j < 38; j++)
			myfile << std::setprecision(4) << aiTable[j][i] << "\t";
		myfile << endl;
	}
	myfile.close();
}

void QLearning::caculateaiTable()
{
	int t				= 0;
	int a				= 0;
	int roomsExplored	= 0;
	float reward		= 0;
	vector<moves> actions;

	state * s = &allStates[10];
	state * nS = &allStates[10];

	loadMaples();

	while (true)
	{
		// time of specific run increased
		t++;
		// get action
		a = getNextTableAction(s);
		// reward for state with the action
		reward = getReward(s, a);
		// add to list of moves for bestActions vector
		actions.push_back(moves{ s,a });
		// get next state from current state and action
		nS = getNextState(s, a);


		// Check if the state gets explored
		if (s->unexplored && reward == 1)
		{
			roomsExplored++;
			s->unexplored = false;
		}

		// Set current state to next state
		s = nS;

		// Terminate run if all rooms are explored or the time exceeds tMax
		if (roomsExplored == numberOfRooms || t > tMax)
			break;
	}
	bestActions = actions;
}

void QLearning::loadRooms()
{
	if (hasRun)
		return;
	// Create all rooms
	for (int i = 0; i < numberOfRooms; i++)
	{
		allStates.push_back(state{});
		allStates[i].roomNumber = i + 1;
		allStates[i].unexplored = true;
	}

	// add all room connections + the ability to stay in the room
	for (int i = 0; i<allStates.size(); i++)
		allStates[i].possibleStates.push_back(&allStates[i]);

	allStates[0].possibleStates.push_back(&allStates[1]);

	allStates[1].possibleStates.push_back(&allStates[2]);
	allStates[1].possibleStates.push_back(&allStates[0]);

	allStates[2].possibleStates.push_back(&allStates[1]);
	allStates[2].possibleStates.push_back(&allStates[3]);
	allStates[2].possibleStates.push_back(&allStates[17]);

	allStates[3].possibleStates.push_back(&allStates[2]);
	allStates[3].possibleStates.push_back(&allStates[4]);
	allStates[3].possibleStates.push_back(&allStates[5]);
	allStates[3].possibleStates.push_back(&allStates[10]);

	allStates[4].possibleStates.push_back(&allStates[3]);

	allStates[5].possibleStates.push_back(&allStates[3]);

	allStates[6].possibleStates.push_back(&allStates[7]);
	allStates[6].possibleStates.push_back(&allStates[12]);

	allStates[7].possibleStates.push_back(&allStates[6]);
	allStates[7].possibleStates.push_back(&allStates[13]);

	allStates[8].possibleStates.push_back(&allStates[17]);
	allStates[8].possibleStates.push_back(&allStates[9]);

	allStates[9].possibleStates.push_back(&allStates[8]);

	allStates[10].possibleStates.push_back(&allStates[17]);
	allStates[10].possibleStates.push_back(&allStates[3]);
	allStates[10].possibleStates.push_back(&allStates[12]);

	allStates[11].possibleStates.push_back(&allStates[17]);

	allStates[12].possibleStates.push_back(&allStates[10]);
	allStates[12].possibleStates.push_back(&allStates[6]);
	allStates[12].possibleStates.push_back(&allStates[13]);

	allStates[13].possibleStates.push_back(&allStates[12]);
	allStates[13].possibleStates.push_back(&allStates[14]);
	allStates[13].possibleStates.push_back(&allStates[7]);
	allStates[13].possibleStates.push_back(&allStates[16]);

	allStates[14].possibleStates.push_back(&allStates[13]);

	allStates[15].possibleStates.push_back(&allStates[16]);
	allStates[15].possibleStates.push_back(&allStates[18]);

	allStates[16].possibleStates.push_back(&allStates[13]);
	allStates[16].possibleStates.push_back(&allStates[15]);

	allStates[17].possibleStates.push_back(&allStates[8]);
	allStates[17].possibleStates.push_back(&allStates[10]);
	allStates[17].possibleStates.push_back(&allStates[11]);
	allStates[17].possibleStates.push_back(&allStates[2]);

	allStates[18].possibleStates.push_back(&allStates[15]);
}

void QLearning::loadMaples()
{
	// Set all rooms to unexplored
	for (int i = 0; i < allStates.size(); i++)
	{
		allStates[i].unexplored = true;
	}
}

state * QLearning::getNextState(state * s, int a)
{
	// return the next state given the action
	return s->possibleStates[a];
}

int QLearning::getReward(state * s, int a)
{
	// Check the state and action to give the right reward: -2, -1, 0 or 1
	if (a == 0)
	{
		if (s->unexplored != 0)
		{
			return 1;
		}
		else
			return -2;
	}
	else
	{
		if (getNextState(s, a)->unexplored != 0)
			return 0;
		else
			return -1;
	}
}

int QLearning::getNextAction(state * s)
{
	int maxReward = -3;
	int reward;
	int bestAction;

	// Find the best action (greedy) or do something random
	if (rand() % e + 1 == e)
	{
		bestAction = rand() % s->possibleStates.size();
	}
	else
	{
		for (int i = 0; i < s->possibleStates.size(); i++)
		{
			reward = getReward(s, i);
			if (reward > maxReward)
			{
				bestAction = i;
				maxReward = reward;
			}
		}
	}

	return bestAction;
}

int QLearning::getMaxAction(state * s)
{
	int maxReward = -3;
	int bestAction = 0;

	// find the best action (greedy)
	for (int i = 0; i < s->possibleStates.size(); i++)
	{
		if (getReward(s, i) > maxReward)
		{
			bestAction = i;
			maxReward = getReward(s, i);
		}
	}

	return bestAction;
}

int QLearning::getNextTableAction(state * s)
{
	float maxReward = -100000;
	float reward = 0;
	int bestAction = 0;

	int aiTableS, aiTableSA;

	if (s->unexplored)
		aiTableS = (s->roomNumber - 1) * 2;
	else
		aiTableS = (s->roomNumber - 1) * 2 + 1;

	state * nS;

	// find the best action (greedy)
	for (int i = 0; i < s->possibleStates.size(); i++)
	{
		nS = getNextState(s, i);

		if (nS->unexplored)
			aiTableSA = (nS->roomNumber - 1) * 2;
		else
			aiTableSA = (nS->roomNumber - 1) * 2 + 1;

		reward = (aiTable[aiTableS][aiTableSA]);
		if (reward > maxReward)
		{
			bestAction = i;
			maxReward = reward;
		}
	}

	return bestAction;
}
