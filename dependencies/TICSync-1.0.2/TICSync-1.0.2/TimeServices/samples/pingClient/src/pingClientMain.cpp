/* 
 * copyright {
 * TICSync - A Clock Synchronization Library
 * Copyright 2012 Isis Innovation Limited
 * University of Oxford
 * 
 * Authors: Alastair Harrison
 *          arh@robots.ox.ac.uk
 *
 * See included LICENSE file for license details
 * }
 */

#include <iostream>
#include <string>
#include <sstream>
#include "Poco/Thread.h"
#include "Poco/Format.h"
#include "Poco/Timestamp.h"
#include "TICSync/BasicTimeClient.h"
#include <GetPot>


int main(int argc, char** argv)
{
  GetPot cl(argc, argv);

  if( cl.search("--help") )
  {
    std::cerr << cl[0] << " -h <hostaddress> -p <port> -f <frequency Hz>\n";
    return 0;
  }

  std::string address("localhost");
  if( cl.search(2, "-h", "--host") )
    address = cl.next("localhost");

  int port(5555);
  if ( cl.search(2, "-p", "--port") )
    port = cl.next(5555);

  double freq(10.0);
  if (cl.search(2, "-f", "--frequency") )
    freq = cl.next(10.0);

  int samplePeriod = static_cast<int> (1.0e3 / freq); // ms

  std::stringstream s;
  s << "tcp://" << address << ":" << port;
  std::string zmq_address(s.str());

  TICSync::BasicTimeClient timeClient(zmq_address.c_str());
  Poco::Timestamp serverStart(timeClient.requestServerTime());


  Poco::Timestamp lastPingTime;
  while (1)
  {
    Poco::Timestamp requestTime(0);
    Poco::Timestamp responseTime(0);
    Poco::Timestamp serverTime(timeClient.requestServerTime(requestTime, responseTime));

    std::cout << requestTime.epochMicroseconds() << " ";
    std::cout << serverTime.epochMicroseconds() << " ";
    std::cout << responseTime.epochMicroseconds() << "\n";

    Poco::Timestamp timeNow;
    Poco::Timestamp::TimeDiff elapsed = (timeNow - lastPingTime);
    int delay = samplePeriod - elapsed/1000;
    if (delay < 0) delay = 0;

    Poco::Thread::sleep(delay);
    lastPingTime.update();
  }

  return 0;
}
