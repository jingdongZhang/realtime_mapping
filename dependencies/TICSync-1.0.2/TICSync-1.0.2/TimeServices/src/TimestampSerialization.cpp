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
 
#include "TICSync/TimestampSerialization.h"
#include "Poco/BinaryWriter.h"
#include "Poco/BinaryReader.h"
#include "Poco/Timestamp.h"

namespace TICSync
{

} // namespace Poco


Poco::BinaryReader& operator>>(Poco::BinaryReader& s, Poco::Timestamp& val)
{
  Poco::Timestamp::UtcTimeVal utc;
  s >> utc;
  val = Poco::Timestamp::fromUtcTime(utc);
  return s;
}


Poco::BinaryWriter& operator<<(Poco::BinaryWriter& s, const Poco::Timestamp& val)
{
  return (s << val.utcTime());
}
