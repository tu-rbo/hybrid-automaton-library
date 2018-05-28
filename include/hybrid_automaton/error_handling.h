#ifndef HYBRID_AUTOMATON_ERROR_HANDLING_H_
#define HYBRID_AUTOMATON_ERROR_HANDLING_H_

#include <iostream>
#include <sstream>
#include "NonblockingPrinting.h"

#define HA_STREAM(logger_name, args, level) "[" << logger_name << "] " << level << ": " << args << " (" << __FILE__ << ":" << __LINE__ << ")"

#define HA_DEBUG(logger_name, args)do{\
	std::stringstream ss_safeprint;\
    ss_safeprint << HA_STREAM(logger_name, args, "DEBUG");\
	safePrint(ss_safeprint.str(), PRINTLVL_DEBUG);\
	}while(0)

#define HA_INFO(logger_name, args)do{\
	std::stringstream ss_safeprint;\
    ss_safeprint << HA_STREAM(logger_name, args, "INFO");\
	safePrint(ss_safeprint.str(), PRINTLVL_INFO);\
	}while(0)

#define HA_WARN(logger_name, args)do{\
	std::stringstream ss_safeprint;\
    ss_safeprint << HA_STREAM(logger_name, args, "WARN");\
	safePrint(ss_safeprint.str(), PRINTLVL_WARN);\
	}while(0)

#define HA_ERROR(logger_name, args)do{ \
	std::stringstream ss_safeprint;\
    ss_safeprint << HA_STREAM(logger_name, args, "ERROR");\
	safePrint(ss_safeprint.str(), PRINTLVL_ERROR);\
    }while(0)

#define HA_THROW_ERROR(logger_name, args)\
{ std::ostringstream __os__;\
__os__ << HA_STREAM(logger_name, args, "ERROR") << std::endl;\
std::cerr << __os__.str() << std::endl;\
throw __os__.str(); }



#endif