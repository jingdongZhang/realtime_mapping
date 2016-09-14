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

#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <iomanip>


#include "Poco/Format.h"
#include "Poco/Timestamp.h"
#include "Poco/DateTimeFormatter.h"
#include "Poco/DateTimeFormat.h"
#include "TICSync/TwoWayTimeSim.h"
#include "TICSync/LinearTwoWayClockSync.h"


using namespace TICSync;
using Poco::Timestamp;


int main(int argc, char **argv)
{
  Timestamp timeNowTS;
  Timestamp::TimeVal timeNow = timeNowTS.epochMicroseconds();

  TICSync::TwoWayTimeSim timeSim;
  timeSim.setM1(1.0);
  timeSim.setM2(1.0);
  timeSim.setFreq(1.0);

  // Now for a shiny new MaxSepFilter
  TICSync::LinearTwoWayClockSync<Timestamp::TimeVal> clockMapper;

  for (unsigned int i=0; i < 1000; ++i)
  {
    TICSync::TSTriple ts;
    timeSim.nextMeas(ts);

    Timestamp::TimeVal RQ = static_cast<Timestamp::TimeVal> (ts.requestTime * 1.0e6);
    Timestamp::TimeVal TX = static_cast<Timestamp::TimeVal> (ts.serverTime  * 1.0e6);
    Timestamp::TimeVal RX = static_cast<Timestamp::TimeVal> (ts.receiptTime * 1.0e6);

    clockMapper.update(RQ+timeNow, TX+timeNow, RX+timeNow);
  }

  // Output what we learned
  double skew = clockMapper.skew();

  Timestamp::TimeVal Tc1 = timeNow;
  Timestamp::TimeVal Ts1 = clockMapper.clientTimeToServerTime(Tc1);
  Timestamp::TimeVal Tc2 = clockMapper.serverTimeToClientTime(Ts1);

  double dfTrueAlpha = (timeSim.getM2() - timeSim.getM1()) / timeSim.getM1();
  std::cout << "True relative skew: alpha = " << dfTrueAlpha << std::endl;

  double dfTrueBeta = timeSim.getC2() - (timeSim.getM2()*timeSim.getC1())/timeSim.getM1();
  std::cout << "True relative offset: beta = " << static_cast<Timestamp::TimeVal> (dfTrueBeta * 1e6) << std::endl;

  std::cout << "Learned skew: ";
  std::cout << skew << "\n";

  std::cout << "Client Time: " << Tc1 << " ";
  std::cout << "Maps to Server Time: " << Ts1 << "\n";

  std::cout << "Server Time: " << Ts1 << " ";
  std::cout << "Maps to Client Time: " << Tc2 << "\n";

  return 0;
}
