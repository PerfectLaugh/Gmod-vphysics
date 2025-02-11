#include <ctype.h>

#include <tier1/tier1.h>
#include <tier0/platform.h>

#include "vphysics_interface.h"
#include <vphysics/collision_set.h> // THIS FILE HAS NO INCLUDE GUARDS!

// NEW INTERFACE HEADERS
#include "vphysics_interfaceV32.h"

#include <cmodel.h>

#include "Bullet.h"

#include <algorithm>
#include <string>
#include <vector>
#include <unordered_map>

//#define DEBUG_DRAW 1

// Probably shouldn't be using defines for these.
#define SLEEP_LINEAR_THRESHOLD 0.15 // m/s
#define SLEEP_ANGULAR_THRESHOLD 0.1 // rad/s

#define DevWarning Warning

#define NOT_IMPLEMENTED				DevWarning("VPhysics UNIMPLEMENTED: %s (%s:%u)\n", __FUNCTION__, __FILE__, __LINE__);
#define NOT_IMPLEMENTED_CRITICAL	Error("VPhysics UNIMPLEMENTED: %s (%s:%u)\n", __FUNCTION__, __FILE__, __LINE__);

// NOTE: Not available on non-MSVC builds due to use of __asm.
//#define NOT_IMPLEMENTED_BREAK		{DevWarning("VPhysics UNIMPLEMENTED: %s (%s:%u)\n", __FUNCTION__, __FILE__, __LINE__); __asm int 3;}
