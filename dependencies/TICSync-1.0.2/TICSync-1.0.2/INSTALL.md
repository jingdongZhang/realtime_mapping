Installing TICSync
==================

Supported platforms
-------------------

TICSync has been tested with Linux and OSX.  There should be nothing to stop it
from working under Windows, but it has not been tested.

Note however that the Poco timestamp classes used in the example programs do
not take advantage of the windows High Performance timers.  The conventional
Windows clock has a granularity of around 10ms, so the
performance of the TICSync example programs may be relatively poor under
Windows.

There is of course nothing to stop you from feeding the output of the Windows
HP timers into TICSync yourself.


Dependencies
------------

###Required

- [CMake](http://www.cmake.org/cmake/resources/software.html)  
  This is a build system from Kitware.  
  Version 2.8 or above is required.


###Optional
Whilst the Core of TICSync has no dependencies (as it is header only),
the TimeServices library and sample programs require the Poco and ZMQ
libraries.  These are both cross-platform and easy to compile.

- [Poco](http://pocoproject.org/download/index.html)  
  A cross-platform library of useful C++ classes.  
  Versions 1.3.6 and above are suitable

- [ZeroMQ](http://www.zeromq.org/)  
  A cross-platform socket library.  
  Versions 2.0 and above are suitable
  
- [cppzmq](https://github.com/zeromq/cppzmq)
  C++ bindings for ZeroMQ.  Required if you use ZeroMQ 3.0 or above.
  Just copy the file `zmq.hpp` into your `/usr/local/include` directory.

- [Doxygen](http://www.doxygen.org/)  
  Required if you wish to build the (currently sparse) doxygen documentation
  found in the source code.

Building TICSync
----------------

The build instructions below are specific to Unix-like systems, but CMake can
be easily configured to produce project files for Microsoft Visual Studio on
Windows platforms. 

These steps describe how to perform an 'out of source' build with CMake.
This keeps the build files separate from the source tree, to avoid
contamination.

Type:  
`mkdir TICSync-build && cd TICSync-build`  
`ccmake /path/to/TICSync/src`

This will bring up the cmake curses interface.
Press 'c' to configure the build.  At this point you may adjust build options.
If cmake is unable to locate ZMQ or Poco then press 't' to bring up the
advanced variables, so that you can specifiy the correct paths manually.

Keep pressing 'c' until there are no new variables (marked with an asterisk).
Then press 'g' to generate the build scripts.

Then type:  
`make`

You can build the Doxygen documentation with:  
`make doc`

You can perform a system install of TICSync using:  
`make install`  
but you must have root privileges

