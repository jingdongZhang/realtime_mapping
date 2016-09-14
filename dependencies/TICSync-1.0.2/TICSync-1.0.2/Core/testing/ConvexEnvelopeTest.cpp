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

#include "TICSync/ConvexEnvelope.h"

int main(int argc, char **argv)
{

  double x[] = {1.0, 1.5, 2.0, 2.3, 3.0, 3.7, 4.0, 5.0};
  double y[] = {6.0, 5.0, 3.0, 4.0, 1.0, 4.0, 2.0, 6.0};

  TICSync::ConvexEnvelope<double> env(TICSync::ConvexEnvelopeTypes::below);

  unsigned int n = sizeof(x)/sizeof(x[0]);
  for (unsigned int i=0; i < n; ++i)
    env.addPoint(x[i], y[i]);

  env.dump();

  std::cout << "Span: " << env.span() << std::endl;

  return 0;
}
