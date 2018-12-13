#include <iostream>
#include "mapPlanning.h"
#include "QLearning.h"

using namespace std;

int main()
{
	// Opret objekt med path til kortet
	mapPlanning planner("floor_plan.png");
	// beregn det
	planner.calculateMap();

	QLearning QL;
	QL.importMap(planner.getPathVec());

	//double stepsize = 0.02;
	//for (int e = 6; e < 21; e++)
	//{
	//	for (double l = stepsize*10; l < 1; l += stepsize)
	//	{
	//		cout << l << endl;
	//		for (double d = stepsize*10; d < 1; d += stepsize)
	//		{
	//			int counter = 0;
	//			for (int i = 0; i < 5; i++)
	//			{
	//				QL.setE(e);
	//				QL.setLearningRate(l);
	//				QL.setDiscountRate(d);

	//				QL.runQLearning();
	//				if (QL.getMarblesFound() == 16)
	//				{
	//					counter++;
	//				}
	//			}

	//			if (counter == 5)
	//				cout << "e: " << e << "; l: " << l << "; d: " << d << "; "<< counter <<endl;
	//			
	//		}
	//	}
	//}

	for (int i = 1; i < 101; i++)
	{
		QL.setRun(100 * i);
		QL.runQLearning();
		cout /*<< "Runs: " << i*100 << ",  "*/ << QL.getMarblesFound() << " ";
	}

	//QL.runQLearning();

	//QL.printBestActions();
	
	planner.showMap();

	return 0;
}