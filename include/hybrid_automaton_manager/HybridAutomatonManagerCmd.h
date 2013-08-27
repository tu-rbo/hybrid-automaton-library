/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#ifndef __HYBRIDAUTOMATONMANAGER_CMD_H__
#define __HYBRIDAUTOMATONMANAGER_CMD_H__

#include "rCommand/rCmdDefine.h"

#define DEFAULT_CMD		(RCMD_USER + 1)
#define SERVO_ON		(RCMD_USER + 5)
#define BLACKBOARD_ON	(RCMD_USER + 6)
#define BLACKBOARD_OFF	(RCMD_USER + 7)
#define ACTIVATE	(RCMD_USER + 7)

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
#define URI_POSEIDON		12
#define URI_FIRSTMM			13
#define URI_SHOEFER			14
#define URI_RBO_EXTRA		15

// this is for plotting from the rxApplication
// (so you don't have to remember which channel to plot)
#define PLOT_TORQUE					1
#define PLOT_Q						2
#define PLOT_VELOCITY				3
#define PLOT_EEFRAME				4

#endif