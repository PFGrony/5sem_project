#include "QLearning.h"

QLearning::QLearning()
{
    // Learning rate and discount rate determins the values of the Q(s,a)
    learningRate = 0.92;
    discountRate = 0.24;

    // the threshhold
    theta = 0.01;
    runs = 2000;

    // how often will we get a random move (1/e)
    e = 11;

    // the number of marbles
    numberOfMarbles = 16;
    batteryStart = 500;

    // how many iterations did it run
    iteration = 0;
    statesUsed = 0;

    // if it is false it will load the rooms
    hasRun = false;
    randomNumber = 2;

    // init random seed
    /*srand(time(NULL));
    srand(rand() % 100 + 1);*/
}

QLearning::~QLearning()
{
}

void QLearning::runQLearning()
{
    // load aiList
    setAiList();
    // setup all the need data types
    double delta			= 0;

    state * s;
    state * nS;

    int a					= 0;
    int roomsExplored		= 0;
    int marblesFound		= 0;
    int reward				= 0;
    int t					= 0;
    int skalar				= 0;
    int battery				= 0;

    double Qsa = 0;
    double argmaxQsa = 0;
    double newQsa = 0;

    // Run the AI
    do {
        //reset delta
        delta = 0;
        // reload maples / unexplored rooms
        setExploration();
        // set starting state/room

        s = &allStates[startingRoom];

        // set to starting state aswell or else it crashes (??)
        nS = s;

        // reset time
        t++;
        // reset the amout of rooms explored
        roomsExplored = 0;
        marblesFound = 0;
        battery = batteryStart;

        while (true)
        {
            // Check if the state gets explored
            if (worldUnExp[s->roomNumber - 1] && marbles[s->roomNumber - 1] == 0)
            {
                roomsExplored++;
                worldUnExp[s->roomNumber - 1] = false;
            }
            // get action
            a = getNextAction(s);
            // reward for state with the action
            reward = getReward(s, a);
            // get next state from current state and action
            nS = getNextState(s, a);

            // battery update
            if (reward == 10)
                battery += 25;

            battery += (s->cost[a] * -1);
            // caculate the needed placements on the table

            skalar = 0;
            for (int i = 0; i < allStates.size(); i++)
            {
                if (worldUnExp[i] == true)
                    skalar = skalar + 1 * pow(2, i);
            }

            Qsa = getValueAiList(s->roomNumber, skalar, a);

            argmaxQsa = getValueAiList(nS->roomNumber, skalar, getNextTableAction(nS));

            newQsa = Qsa + learningRate * (getReward(s, a) + (discountRate * argmaxQsa) - Qsa);

            insertValueAiList(s->roomNumber, skalar, a, newQsa);

            delta = std::max(delta, (double)fabs(Qsa - newQsa));

            if (a == 0 && marbles[s->roomNumber - 1] != 0)
            {
                marblesFound++;
                marbles[s->roomNumber - 1]--;
            }
            // Set current state to next state
            s = nS;

            // Terminate run if all rooms are explored or the time exceeds tMax
            if (marblesFound == numberOfMarbles || battery <= 0 )
                break;
        }
        //cout << delta << endl;

        // Run again is delta is above theta // delta > theta
    } while (t < runs);

    //cout << "Q-learning was done in " << i << " iterations." << endl;

    calculateaiTable();
}

QlPoints QLearning::getPoint(int x)
{
    return bestActions[x].s->posStates;
}

void QLearning::printBestActions()
{
    cout << "Marbles found: " << finalMarblesFound << endl;
    cout << "The path was done in " << bestActions.size() << " moves" << endl;
    int length;
    if (finalMarblesFound == numberOfMarbles)
        length = bestActions.size();
    else
        length = bestActions.size() - 1;

    for (int i = 0; i < length; i++)
    {
        if (bestActions[i].a == 0)
            cout << "Seach in room " << bestActions[i].s->roomNumber << endl;
        else
            cout << "Move from room " << bestActions[i].s->roomNumber << " to room " << bestActions[i + 1].s->roomNumber << endl;
        cout << "Battery: " << bestActions[i].battery << endl;
    }
    cout << "States used: " << statesUsed << endl;
}

void QLearning::calculateaiTable()
{
    int t				= 0;
    int a				= 0;
    int roomsExplored	= 0;
    int marblesFound	= 0;
    int battery			= batteryStart;
    float reward		= 0;
    vector<moves> actions;
    int TotalReward = 0;
    state * s;
    state * nS;


    s = &allStates[startingRoom];

    nS = s;

    setExploration();

    while (true)
    {
        // time of specific run increased
        t++;
        // Check if the state gets explored
        if (worldUnExp[s->roomNumber - 1] && marbles[s->roomNumber - 1] == 0)
        {
            roomsExplored++;
            worldUnExp[s->roomNumber - 1] = false;
        }
        // get action
        a = getNextTableAction(s);
        // reward for state with the action
        reward = getReward(s, a);
        TotalReward += reward;

        if (reward == 10)
            battery += 25;
        battery += (s->cost[a] * -1);

        if (reward == -20)
            battery += -50;
        //
        if (a == 0 && marbles[s->roomNumber - 1] != 0)
        {
            marblesFound++;
            marbles[s->roomNumber - 1]--;
        }
        // add to list of moves for bestActions vector
        actions.push_back(moves{ s,a,battery });
        // get next state from current state and action
        nS = getNextState(s, a);

        // Set current state to next state
        s = nS;

        // Terminate run if all rooms are explored or the time exceeds tMax
        if (marblesFound == numberOfMarbles || battery <= 0)
            break;
    }
    finalMarblesFound = marblesFound;
    bestActions = actions;
    cout << "Total reward: " <<TotalReward << endl;
}

void QLearning::setAiList()
{
    if (aiList.empty())
    {
        for (int i = 0; i < allStates.size(); i++)
            aiList.push_back(new list<valueState>);
    }
    else
    {
        for (list<list<valueState>*>::iterator it = aiList.begin(); it != aiList.end(); ++it)
            (*it)->clear();
    }
}

void QLearning::insertValueAiList(int room, int state, int action, double value)
{

    list<list<valueState>*>::iterator it = aiList.begin();

    for (int i = 0; i < room - 1; i++)
        ++it;

    list<valueState>::iterator it2 = (*it)->begin();

    while (it2 != (*it)->end() && (*it2).state > state )
        ++it2;

    while (it2 != (*it)->end() && (*it2).state == state)
    {
        if ((*it2).action == action)
        {
            (*it2).value = value;
            return;
        }
        ++it2;
    }

    statesUsed++;

    if (it2 == (*it)->end())
        (*it)->push_back(valueState{ state,action,value });
    else if ((*it2).state != state)
        (*it)->insert(it2, valueState{ state,action,value });

}

double QLearning::getValueAiList(int room, int state, int action)
{
    list<list<valueState>*>::iterator it = aiList.begin();

    for (int i = 0; i < room - 1; i++)
        ++it;

    list<valueState>::iterator it2 = (*it)->begin();

    while (it2 != (*it)->end() && (*it2).state > state )
        ++it2;

    while (it2 != (*it)->end() && (*it2).state == state)
    {
        if ((*it2).action == action)
            return (*it2).value;
        ++it2;
    }

    return -25.0;
}

void QLearning::loadBigWorld()
{
    if (hasRun)
        return;
    else
        hasRun = true;

    numberOfRooms = 19;

    test = false;
    // Create all rooms
    for (int i = 0; i < numberOfRooms; i++)
    {
        allStates.push_back(state{});
        allStates[i].roomNumber = i + 1;
    }

    // add all room connections + the ability to stay in the room
    for (int i = 0; i< numberOfRooms; i++)
        allStates[i].possibleStates.push_back(&allStates[i]);

    for (int i = 0; i < numberOfRooms; i++)
    {
        worldUnExp.push_back(bool(true));
        marbles.push_back(0);
    }



    allStates[0].posStates.x = ((double) 67 / 8);
    allStates[0].posStates.y = ((double)61 / 8);

    allStates[0].possibleStates.push_back(&allStates[1]);

    allStates[1].posStates.x = ((double)194 / 8);
    allStates[1].posStates.y = ((double)74 / 8);

    allStates[1].possibleStates.push_back(&allStates[2]);
    allStates[1].possibleStates.push_back(&allStates[0]);

    allStates[2].posStates.x = ((double)119 / 8);
    allStates[2].posStates.y = ((double)191 / 8);

    allStates[2].possibleStates.push_back(&allStates[1]);
    allStates[2].possibleStates.push_back(&allStates[3]);
    allStates[2].possibleStates.push_back(&allStates[17]);

    allStates[3].posStates.x = ((double)337 / 8);
    allStates[3].posStates.y = ((double)126 / 8);

    allStates[3].possibleStates.push_back(&allStates[2]);
    allStates[3].possibleStates.push_back(&allStates[4]);
    allStates[3].possibleStates.push_back(&allStates[5]);
    allStates[3].possibleStates.push_back(&allStates[10]);

    allStates[4].posStates.x = ((double)540 / 8);
    allStates[4].posStates.y = ((double)81 / 8);

    allStates[4].possibleStates.push_back(&allStates[3]);

    allStates[5].posStates.x = ((double)544 / 8);
    allStates[5].posStates.y = ((double)208 / 8);

    allStates[5].possibleStates.push_back(&allStates[3]);

    allStates[6].posStates.x = ((double)743 / 8);
    allStates[6].posStates.y = ((double)143 / 8);

    allStates[6].possibleStates.push_back(&allStates[7]);
    allStates[6].possibleStates.push_back(&allStates[12]);

    allStates[7].posStates.x = ((double)884 / 8);
    allStates[7].posStates.y = ((double)114 / 8);

    allStates[7].possibleStates.push_back(&allStates[6]);
    allStates[7].possibleStates.push_back(&allStates[13]);

    allStates[8].posStates.x = ((double)70 / 8);
    allStates[8].posStates.y = ((double)327 / 8);

    allStates[8].possibleStates.push_back(&allStates[17]);
    allStates[8].possibleStates.push_back(&allStates[9]);

    allStates[9].posStates.x = ((double)70 / 8);
    allStates[9].posStates.y = ((double)506 / 8);

    allStates[9].possibleStates.push_back(&allStates[8]);

    allStates[10].posStates.x = ((double)480 / 8);
    allStates[10].posStates.y = ((double)313 / 8);

    allStates[10].possibleStates.push_back(&allStates[17]);
    allStates[10].possibleStates.push_back(&allStates[3]);
    allStates[10].possibleStates.push_back(&allStates[12]);

    allStates[11].posStates.x = ((double)250 / 8);
    allStates[11].posStates.y = ((double)484 / 8);

    allStates[11].possibleStates.push_back(&allStates[17]);

    allStates[12].posStates.x = ((double)720 / 8);
    allStates[12].posStates.y = ((double)300 / 8);

    allStates[12].possibleStates.push_back(&allStates[10]);
    allStates[12].possibleStates.push_back(&allStates[6]);
    allStates[12].possibleStates.push_back(&allStates[13]);

    allStates[13].posStates.x = ((double)856 / 8);
    allStates[13].posStates.y = ((double)377 / 8);

    allStates[13].possibleStates.push_back(&allStates[12]);
    allStates[13].possibleStates.push_back(&allStates[14]);
    allStates[13].possibleStates.push_back(&allStates[7]);
    allStates[13].possibleStates.push_back(&allStates[16]);

    allStates[14].posStates.x = ((double)550 / 8);
    allStates[14].posStates.y = ((double)430 / 8);

    allStates[14].possibleStates.push_back(&allStates[13]);

    allStates[15].posStates.x = ((double)619 / 8);
    allStates[15].posStates.y = ((double)557 / 8);

    allStates[15].possibleStates.push_back(&allStates[16]);
    allStates[15].possibleStates.push_back(&allStates[18]);

    allStates[16].posStates.x = ((double)835 / 8);
    allStates[16].posStates.y = ((double)567 / 8);

    allStates[16].possibleStates.push_back(&allStates[13]);
    allStates[16].possibleStates.push_back(&allStates[15]);

    allStates[17].posStates.x = ((double)219 / 8);
    allStates[17].posStates.y = ((double)317 / 8);

    allStates[17].possibleStates.push_back(&allStates[8]);
    allStates[17].possibleStates.push_back(&allStates[10]);
    allStates[17].possibleStates.push_back(&allStates[11]);
    allStates[17].possibleStates.push_back(&allStates[2]);

    allStates[18].posStates.x = ((double)440 / 8);
    allStates[18].posStates.y = ((double)565 / 8);

    allStates[18].possibleStates.push_back(&allStates[15]);
}

void QLearning::loadTestWorld()
{
    if (hasRun)
        return;
    else
        hasRun = true;

    numberOfRooms = 5;

    test = true;

    for (int i = 0; i < 5; i++)
    {
        worldUnExp.push_back(bool(true));
    }

    for (int i = 0; i < 5; i++)
    {
        allStates.push_back(state{});
        allStates[i].roomNumber = i + 1;
    }

    for (int i = 0; i < allStates.size(); i++)
    {
        allStates[i].possibleStates.push_back(&allStates[i]);
        allStates[i].posStates.x = 0;
        allStates[i].posStates.y = 0;
    }


    allStates[0].possibleStates.push_back(&allStates[1]);
    allStates[0].possibleStates.push_back(&allStates[3]);

    allStates[1].possibleStates.push_back(&allStates[2]);
    allStates[1].possibleStates.push_back(&allStates[3]);
    allStates[1].possibleStates.push_back(&allStates[0]);

    allStates[2].possibleStates.push_back(&allStates[1]);

    allStates[3].possibleStates.push_back(&allStates[4]);
    allStates[3].possibleStates.push_back(&allStates[0]);
    allStates[3].possibleStates.push_back(&allStates[1]);

    allStates[4].possibleStates.push_back(&allStates[3]);
}

void QLearning::importMap(vector<paths> pathVec)
{
    if (hasRun)
        return;
    else
        hasRun = true;



    vector<coordinate> roomCoord;

    for (int j = 0; j < pathVec.size(); j++)
    {
        bool notThere = true;
        for (int i = 0; i < roomCoord.size(); i++)
        {
            if (roomCoord[i].x == pathVec[j].start.x && roomCoord[i].y == pathVec[j].start.y)
                notThere = false;
        }

        if (notThere)
            roomCoord.push_back(pathVec[j].start);

        notThere = true;
        for (int i = 0; i < roomCoord.size(); i++)
        {
            if (roomCoord[i].x == pathVec[j].end.x && roomCoord[i].y == pathVec[j].end.y)
                notThere = false;
        }

        if (notThere)
            roomCoord.push_back(pathVec[j].end);
    }

    numberOfRooms = roomCoord.size();

    test = false;

    // Create all rooms
    for (int i = 0; i < numberOfRooms; i++)
    {
        allStates.push_back(state{});
        allStates[i].roomNumber = i + 1;

    }

    // add all room connections + the ability to stay in the room
    for (int i = 0; i < numberOfRooms; i++)
    {
        allStates[i].cost.push_back(0);
        allStates[i].possibleStates.push_back(&allStates[i]);
    }

    for (int i = 0; i < numberOfRooms; i++)
    {
        worldUnExp.push_back(bool(true));
        marbles.push_back(0);
    }

    maxCost = 0;

    for (int i = 0; i < roomCoord.size(); i++)
    {
        if (roomCoord[i].x == 64 && roomCoord[i].y == 39)
            startingRoom = i;

        /*cout << "Node " << i+1 << "; " << roomCoord[i].x << ":" << roomCoord[i].y << endl;*/
        allStates[i].posStates.x = roomCoord[i].x;
        allStates[i].posStates.y = roomCoord[i].y;
        for (int j = 0; j < pathVec.size(); j++)
        {
            if (roomCoord[i].x == pathVec[j].start.x && roomCoord[i].y == pathVec[j].start.y)
            {
                for (int k = 0; k < roomCoord.size(); k++)
                {
                    if (roomCoord[k].x == pathVec[j].end.x && roomCoord[k].y == pathVec[j].end.y)
                    {
                        if (pathVec[j].cost > maxCost)
                            maxCost = pathVec[j].cost;

                        allStates[i].cost.push_back(pathVec[j].cost);
                        allStates[i].possibleStates.push_back(&allStates[k]);
                        allStates[k].cost.push_back(pathVec[j].cost);
                        allStates[k].possibleStates.push_back(&allStates[i]);
                        break;
                    }
                }
            }
        }
    }
    //cout << "MaxCost: " << maxCost << endl;

}

void QLearning::setE(int x)
{
    e = x;
}

void QLearning::setDiscountRate(double x)
{
    discountRate = x;
}

void QLearning::setLearningRate(double x)
{
    learningRate = x;
}

void QLearning::setRun(int x)
{
    runs = x;
}

int QLearning::getMarblesFound()
{
    return finalMarblesFound;
}

void QLearning::printAiList()
{
    int counter = 0;
    list<list<valueState>*>::iterator it = aiList.begin();
    while (it != aiList.end())
    {
        list<valueState>::iterator it2 = (*it)->begin();
        while (it2 != (*it)->end())
        {
            cout << "s: " << (*it2).state << "; a: " << (*it2).action << "; v: " << (*it2).value << endl;
            ++it2;
            counter++;
        }
        ++it;
        cout << endl;
    }
    cout << "States used: " << counter << endl;
}

int QLearning::getBestSize()
{
    return bestActions.size();
}

void QLearning::setExploration()
{
    // Set all rooms to unexplored
    for (int i = 0; i < numberOfRooms; i++)
    {
        worldUnExp[i] = true;
        marbles[i] = 0;
    }


    srand(randomNumber++);

    for (int i = 0; i < numberOfMarbles; i++)
        marbles[rand() % numberOfRooms] += 1;

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
        if (worldUnExp[s->roomNumber - 1])
        {
            return 10;
        }
        else
            return -20;
    }
    else
    {
        if (worldUnExp[getNextState(s, a)->roomNumber - 1])
            return 5;
        else
            return -5;
    }
}

int QLearning::getNextAction(state * s)
{
    int maxReward = -10000;
    int reward;

    vector<int> vec;

    // Find the best action (greedy) or do something random
    if (rand() % e + 1 == e)
    {
        vec.push_back(rand() % s->possibleStates.size());
    }
    else
    {
        for (int i = 0; i < s->possibleStates.size(); i++)
        {
            reward = getReward(s, i);
            if (reward > maxReward)
            {
                vec.clear();
                vec.push_back(i);
                maxReward = reward;
            }
            else if (reward == maxReward)
            {
                vec.push_back(i);
            }
        }
    }

    if (vec.size() == 1)
        return vec[0];
    else
        return vec[rand() % vec.size()];
}

int QLearning::getNextTableAction(state * s)
{
    double maxReward = -100000;
    double reward = 0;
    int skalar = 0;

    for (int i = 0; i < allStates.size(); i++)
    {
        if (worldUnExp[i] == true)
            skalar = skalar + 1 * pow(2, i);
    }

    vector<int> vec;

    // find the best action (greedy)
    for (int i = 0; i < s->possibleStates.size(); i++)
    {
        reward = getValueAiList(s->roomNumber, skalar, i);

        if (reward > maxReward)
        {
            vec.clear();
            vec.push_back(i);
            maxReward = reward;
        }
        else if (reward == maxReward)
            vec.push_back(i);
    }

    if (vec.size() == 1)
        return vec[0];
    else
        return vec[rand() % vec.size()];
}
