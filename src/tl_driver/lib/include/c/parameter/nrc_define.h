#ifndef INCLUDE_C_PARAMETER_NRC_DEFINE_H_
#define INCLUDE_C_PARAMETER_NRC_DEFINE_H_

#include <string>
#include <vector>

#if	defined(_WIN32) || defined(WIN32)   //windows
#define EXPORT_API __declspec(dllexport)
#else
#define EXPORT_API
#endif

using SOCKETFD = int;

#endif /* INCLUDE_API_NRC_INTERFACE_DEFINE_H_ */



