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

#include <TICSync/LinearTwoWayClockSync.h>

double randDelay(int max_ms)
{
  int delay_ms = rand() % (max_ms * 1000);
  return delay_ms / 1e6;
}


int main(int argc, char **argv)
{

  /* initialize random seed: */
  srand ( time(NULL) );

  double m1 = 1.0;
  double m2 = 1.1;

  double c1 = 1.0;
  double c2 = 1000.0;

  double freq = 1; // Hz
  double minDelay = 0.003; // Minimum packet delay


  // Instantiate an object to perform the clock synchronization
  TICSync::LinearTwoWayClockSync<double> clockMapper;

  double t3=0;

  for (unsigned int i=0; i < 10000; ++i)
  {
    // Earliest start of transaction
    double t = static_cast<double> (i) / freq;

    // Can't have time going backwards!
    if (t < t3) t = t3;

    // Generate set of three timestamps
    double t1 =  t + randDelay(10);
    double t2 = t1 + randDelay(40) + minDelay;
    t3 = t2 + randDelay(40) + minDelay;

    // Push through clock models
    double RQ = m2*t1 + c2; // Client requests
    double TX = m1*t2 + c1; // Server responds
    double RX = m2*t3 + c2; // Client receives

    // Feed into TICSync
    //filt1.update(RQ, TX, RX, &skewinfo1);

    clockMapper.update(RQ, TX, RX);
  }

  // Output what we learned
  double skew = clockMapper.skew();


  double clientTime1 = 50.0;
  double serverTime1 = clockMapper.clientTimeToServerTime(clientTime1);

  double serverTime2 = 50.0;
  double clientTime2 = clockMapper.serverTimeToClientTime(serverTime2);

  double clientTime3 = clockMapper.serverTimeToClientTime(serverTime1);


  double dfTrueAlpha = (m2-m1)/m1;
  std::cout << "True relative skew: alpha = " << dfTrueAlpha << std::endl;

  double dfTrueBeta = c2 - (m2*c1)/m1;
  std::cout << "True relative offset: beta = " << dfTrueBeta << std::endl;

  std::cout << "Learned skew: ";
  std::cout << skew << "\n";

  std::cout << "Client Time: " << clientTime1 << " ";
  std::cout << "Maps to Server Time: " << serverTime1 << "\n";

  std::cout << "Server Time: " << serverTime2 << " ";
  std::cout << "Maps to Client Time: " << clientTime2 << "\n";

  std::cout << "Server Time: " << serverTime1 << " ";
  std::cout << "Maps to Client Time: " << clientTime3 << "\n";

  return 0;
}
