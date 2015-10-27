#ifndef utils_h
#define utils_h

extern "C" {
#include <TH/TH.h>
}

#define PCLIMP(return_type, class_name, name) extern "C" return_type TH_CONCAT_4(pcl_, class_name, TYPE_KEY, name)

#endif