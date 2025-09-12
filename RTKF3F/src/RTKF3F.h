//RTKF3F.h

#pragma once

#include "_macros.h"
#include "Const.h"  
#include "Structs.h"
#include <GNSS.h>
#include "RadioModule.h"
//#include "ConfigMgr.h"
//#include "Arena.h"
//#include "Glider.h"


enum class F3FTaskState {
	TASK_UNKNOWN,
	TASK_OUTSIDE_A,
	TASK_STARTED,
	TASK_TURN1,
	TASK_TURN2,
	TASK_TURN3,
	TASK_TURN4,
	TASK_TURN5,
	TASK_TURN6,
	TASK_TURN7,
	TASK_TURN8,
	TASK_TURN9,
	TASK_FINISHED
};