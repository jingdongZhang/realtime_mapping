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
 
#ifndef TIMESERVICES_H_
#define TIMESERVICES_H_

#if defined(_WIN32) && defined(TICSync_BUILD_DLLS)
#if defined(TimeServices_EXPORTS)
#define TimeServices_API __declspec(dllexport)
#else
#define TimeServices_API __declspec(dllimport)
#endif
#endif

#if !defined(TimeServices_API)
#define TimeServices_API
#endif

#endif /* TIMESERVICES_H_ */
