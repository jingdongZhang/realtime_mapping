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
#include <ctime>

#include <TICSync/MaxSepFilter.h>

double randDelay(int max_ms)
{
  int delay_ms = rand() % (max_ms * 1000);
  return delay_ms / 1e6;
}


int main(int argc, char **argv)
{

  /* initialize random seed: */
  srand ( static_cast<unsigned int> (time(NULL)) );

  double m1 = 1.1;
  double m2 = 0.9;

  double c1 = 0;
  double c2 = 0;

  double freq = 5; // Hz
  double minDelay = 0.003; // Minimum packet delay


  // Now for a shiny new MaxSepFilter
  TICSync::MaxSepFilter<double> filtMS;

  double t3=0;

  for (unsigned int i=0; i < 1e6; ++i)
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


    double dfOffsetLB = RQ - TX;
    double dfOffsetUB = RX - TX;
    filtMS.addPointPair(TX, dfOffsetUB, dfOffsetLB);

  }

  // Output what we learned
  TICSync::Line<double> seg;
  filtMS.getMidLine(seg);
  double slope = seg.dy() / seg.dx();

  double dfTrueAlpha = (m2-m1)/m1;
  std::cout << "True relative skew: alpha = " << dfTrueAlpha << std::endl;

  double dfTrueBeta = c2 - (m2*c1)/m1;
  std::cout << "True relative offset: beta = " << dfTrueBeta << std::endl;

  std::cout << "Learned slope: ";
  std::cout << slope << "\n";

  //std::cout << "beta error: " << fabs(skewinfo1.beta - dfTrueBeta) << std::endl;

  //getchar();

  return 0;
}
