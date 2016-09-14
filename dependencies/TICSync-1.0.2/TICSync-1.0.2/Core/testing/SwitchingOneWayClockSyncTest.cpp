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
#include <string>
#include <sstream>
#include <TICSync/LinearOneWayClockSync.h>


int main(int argc, char **argv)
{

  double x[] = { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0,
      12.0, 13.0, 14.0, 15.0, 16.0, 17.0 };
  double y[] = { 2.0, 2.0, 2.0, 4.0, 4.0, 7.0, 7.0, 10.0, 8.0, 10.0, 12.0,
      14.0, 16.0, 18.0, 20.0, 22.0, 24.0, 26.0 };

  TICSync::SwitchingOneWayClockSync<double> filt;

  std::stringstream ss;

  unsigned int n = sizeof(x) / sizeof(x[0]);
  for (unsigned int i = 0; i < n; ++i)
  {
    double dfCorrected = filt.filterTimestamp(x[i], y[i]);

    ss << x[i] << " " << y[i] << " " << dfCorrected << "\n";

    std::cout << x[i] << " " << y[i] << " -> ";

    std::cout << dfCorrected;
    std::cout << std::endl;
    std::cout << "-----\n";
    filt.dump();
    std::cout << "-----";
    std::cout << std::endl;

  }

  ss << "e\n";

  //getchar();
  return 0;
}
