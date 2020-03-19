//-*-c++-*-
#ifndef DEBUG_MACROS_H
#define DEBUG_MACROS_H

#define DOUT(n, msg) if (DEBUG_LEVEL >= (n)) { std::cout << std::endl << msg << std::flush; }
#define DOUTS(n, msg) if (DEBUG_LEVEL >= (n)) { std::cout << msg << std::flush; }
#define DDO(n, action) if (DEBUG_LEVEL >= (n)) { action; }

#endif
