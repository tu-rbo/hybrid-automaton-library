#ifndef HYBRID_AUTOMATON_ERROR_HANDLING_H_
#define HYBRID_AUTOMATON_ERROR_HANDLING_H_

#include <iostream>
#include <sstream>

#define HA_STREAM(logger_name, args, level) "[" << logger_name << "] " << level << ": " << args << " (" << __FILE__ << ":" << __LINE__ << ")"
#define HA_DEBUG(logger_name, args) std::cout << HA_STREAM(logger_name, args, "DEBUG") << std::endl
#define HA_INFO(logger_name, args) std::cout << HA_STREAM(logger_name, args, "INFO") << std::endl
#define HA_WARN(logger_name, args) std::cerr << HA_STREAM(logger_name, args, "WARN") << std::endl
#define HA_ERROR(logger_name, args) std::cerr << HA_STREAM(logger_name, args, "ERROR") << std::endl
#define HA_THROW_ERROR(logger_name, args)\
{ std::ostringstream __os__;\
__os__ << HA_STREAM(logger_name, args, "ERROR") << std::endl;\
std::cerr << __os__.str() << std::endl;\
throw __os__.str(); }

#endif