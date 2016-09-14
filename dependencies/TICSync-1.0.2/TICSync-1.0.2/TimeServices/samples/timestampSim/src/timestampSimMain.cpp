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
#include "TICSync/TwoWayTimeSim.h"
#include <GetPot>



int main(int argc, char** argv)
{
  GetPot cl(argc, argv);

  if( cl.search("--help") )
  {
    std::cerr << cl[0] << " -f <frequency Hz> -n <num meas>\n";
    return 0;
  }

  double freq(10.0);
  if (cl.search(2, "-f", "--frequency") )
    freq = cl.next(10.0);

  int nmeas(10);
  if (cl.search(1, "-n"))
    nmeas = cl.next(10);

  TICSync::TwoWayTimeSim sim;
  sim.setFreq(freq);
  sim.setMinDelay(0.030);

  while (nmeas)
  {
    TICSync::TSTriple ts;
    sim.nextMeas(ts);

    Poco::Timestamp requestTime  = static_cast<Poco::Timestamp::TimeVal> (ts.requestTime * 1e6);
    Poco::Timestamp serverTime   = static_cast<Poco::Timestamp::TimeVal> (ts.serverTime * 1e6);
    Poco::Timestamp responseTime = static_cast<Poco::Timestamp::TimeVal> (ts.receiptTime * 1e6);

    std::cout << requestTime.epochMicroseconds() << " ";
    std::cout << serverTime.epochMicroseconds() << " ";
    std::cout << responseTime.epochMicroseconds() << "\n";

    --nmeas;
  }

  return 0;
}
