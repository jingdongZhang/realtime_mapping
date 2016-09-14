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

#ifndef TIMESTAMPSERIALIZATION_H_
#define TIMESTAMPSERIALIZATION_H_

#include "TICSync/TimeServices.h"
#include "Poco/Timestamp.h"
#include "Poco/BinaryWriter.h"
#include "Poco/BinaryReader.h"

namespace TICSync
{
} // TICSync


Poco::BinaryReader& operator>>(Poco::BinaryReader& s, Poco::Timestamp& val);
Poco::BinaryWriter& operator<<(Poco::BinaryWriter& s, const Poco::Timestamp& val);



#endif /* TIMESTAMPSERIALIZATION_H_ */
