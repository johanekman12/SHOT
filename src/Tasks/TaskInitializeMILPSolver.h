/*
 * TaskInitializeMILPSolver.h
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#pragma once

#include <TaskBase.h>
#include "../ProcessInfo.h"
#include "UtilityFunctions.h"
#include "../MILPSolver/IMILPSolver.h"
#include "../MILPSolver/MILPSolverCplex.h"
#include "../MILPSolver/MILPSolverGurobi.h"
#include "../MILPSolver/MILPSolverOsiCbc.h"
#include "../MILPSolver/MILPSolverCplexExperimental.h"

class TaskInitializeMILPSolver: public TaskBase
{
	public:
		TaskInitializeMILPSolver(OSInstance *originalInstance);
		virtual ~TaskInitializeMILPSolver();

		void run();
		virtual std::string getType();

	private:

		SHOTSettings::Settings *settings;
		ProcessInfo *processInfo;
};

