#include "Strategy.h"
#include "./Scaratest/Scaratest.h"
#include "./Carrot/Carrot.h"
#include "./May/BoxMoving.h"

// Constructor
Strategy::Strategy()
{
    isStrategyRunning = false;
}

// Destructor
Strategy::~Strategy() {}

/**
 * Run choosen strategy
 * 
 * @param strategy_name - the strategy name choosen by user
 */
void Strategy::select_strategy(std::string strategy_name)
{
    isStrategyRunning = true;
    while (isStrategyRunning)
    {
        // add strategy below !!!!!!!!
        // If the strategy name match the cases, execute the strategy
        if (strategy_name == "Scaratest")
        {
        }
        else if (strategy_name == "ForwardData")
        {
            Carrot *CCarrot;
            CCarrot = new Carrot(0);
            delete CCarrot;
            isStrategyRunning = false;
        }
        else if (strategy_name == "ShiftData")
        {
            Carrot *CCarrot;
            CCarrot = new Carrot(1);
            delete CCarrot;
            isStrategyRunning = false;
        }
        else if (strategy_name == "SelfTurnData")
        {
            Carrot *CCarrot;
            CCarrot = new Carrot(2);
            delete CCarrot;
            isStrategyRunning = false;
        }
        else if (strategy_name == "ImpedenceControl")
        {
            Carrot *CCarrot;
            CCarrot = new Carrot(3);
            delete CCarrot;
            isStrategyRunning = false;
        }
        else if (strategy_name == "SAMove")
        {
            Carrot *CCarrot;
            CCarrot = new Carrot(4);
            delete CCarrot;
            isStrategyRunning = false;
        }
        else if (strategy_name == "DualArmAdapt")
        {
            Carrot *CCarrot;
            CCarrot = new Carrot(5);
            delete CCarrot;
            isStrategyRunning = false;
        }
        else if (strategy_name == "Debug")
        {
            Carrot *CCarrot;
            CCarrot = new Carrot(-1);
            delete CCarrot;
            isStrategyRunning = false;
        }
        else if (strategy_name == "BoxMoving")
        {
            BoxMoving *CBoxMoving;
            CBoxMoving = new BoxMoving();
            delete CBoxMoving;
            isStrategyRunning = false;
        }
        else
            ;
    }
}