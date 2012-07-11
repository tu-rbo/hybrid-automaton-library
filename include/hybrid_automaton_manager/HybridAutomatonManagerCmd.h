/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#ifndef __HYBRIDAUTOMATONMANAGER_CMD_H__
#define __HYBRIDAUTOMATONMANAGER_CMD_H__

#include "rCommand/rCmdDefine.h"

#define DEFAULT_CMD		(RCMD_USER + 1)
#define EXECUTE_PLAN	(RCMD_USER + 2)
#define PAUSE			(RCMD_USER + 3)
#define RESUME			(RCMD_USER + 4)
#define SERVO_ON		(RCMD_USER + 5)
#define BLACKBOARD_ON	(RCMD_USER + 6)

// use this scheme as follows:
// control->command(BLACKBOARD_ON, (1999 << 16) | (URI_BOTTOM_2 << 8) | (URI_HASMA << 0));
// where 1999 means which port to use (on both ends)
#define URI_LOCAL			0
#define URI_BOTTOM_1		1
#define URI_BOTTOM_2		2
#define URI_BOTTOM_3		3
#define URI_LOHENGRIN		4
#define URI_HASMA			10
#define URI_LEIBNIZ			11

#endif