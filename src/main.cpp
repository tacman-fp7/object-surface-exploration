/*
 * Author: Nawid Jamali
 * Project: TACMAN
 */
#include <stdio.h>
#include <yarp/os/Time.h>
#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

using namespace std;
using namespace yarp::os;


int main(int argc, char *argv[])
{	
  Network yarp;
  if (!yarp.checkNetwork())
  {
    printf("Could not contact yarp server, quitting.\n");
    return false;
  }
  
  return 0;
}
