#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>


namespace ORB_SLAM2
{


long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;


}