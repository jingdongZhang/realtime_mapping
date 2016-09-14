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

#ifndef __timeserviceExceptions_h
#define __timeserviceExceptions_h

#include "TICSync/TimeServices.h"
#include <exception>
#include <string>

namespace TICSync
{

class TimeServices_API IOException: public std::exception
{
  virtual const char* what() const throw ()
  {
    return "Too few data points to perform requested operation.";
  }
};

} // namespace TICSync

#endif // __timeserviceExceptions_h
