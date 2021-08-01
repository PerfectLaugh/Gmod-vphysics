# My very first makefile!
MODULE_NAME = vphysics.so
PROJECT_NAME = vphysics

# Configuration (can only be "debug" or "release")
CONFIGURATION = release

# Path Configuration
# Doesn't matter whether it's mp or sp.
SOURCE_SDK = ../thirdparty/hl2sdk-csgo
BULLET_SDK = ../bullet3

PROJECT_DIR = .
OUT_DIR = ../build/bin/linux/$(CONFIGURATION)
OBJ_DIR = ../build/obj/linux/$(PROJECT_NAME)/$(CONFIGURATION)

# Compilation Configuration
INCLUDES = \
	-I../include 			\
	-I$(SOURCE_SDK)/public 		\
	-I$(SOURCE_SDK)/public/tier0 	\
	-I$(SOURCE_SDK)/public/tier1 	\
	-I$(BULLET_SDK)/src
	
LIBDIRS = \
	-L../build/lib/linux/$(CONFIGURATION) \
	-L$(SOURCE_SDK)/lib/linux
	
# Only works in this order for whatever reason!
LIBS = \
	-lBulletSoftBody 	\
	-lBulletDynamics 	\
	-lBulletCollision	\
	-lLinearMath		\
	$(SOURCE_SDK)/lib/linux/tier1_i486.a 	\
	$(SOURCE_SDK)/lib/linux/mathlib_i486.a 	\
	$(SOURCE_SDK)/lib/linux/interfaces_i486.a 	\
	-ltier0 								\
	-lvstdlib

CC = /usr/bin/g++
LINK = /usr/bin/g++
ARCH = i386
DEFINES = -DCOMPILER_GCC -DLINUX -D__LINUX__ -D_LINUX -D__linux__ -DPOSIX -DGNUC -DARCH=$(ARCH) -Dstricmp=strcasecmp -D_stricmp=strcasecmp -D_snprintf=snprintf -D_vsnprintf=vsnprintf -DHAVE_STDINT_H -D_FILE_OFFSET_BITS=64 -DNETWORK_VARS_ENABLED -D_GLIBCXX_USE_CXX11_ABI=0
CFLAGS = $(INCLUDES) $(DEFINES) -fpermissive -fPIC -w -msse2 -mfpmath=sse -m32 -march=$(ARCH) -g -std=c++14
LFLAGS = -m32 $(LIBDIRS) $(LIBS) -shared -lm -latomic -static-libgcc

ifeq ($(CONFIGURATION), debug)
	# Optimize but don't affect debugging experience. (-Og, only on G++ 4.8 and above)
	#CFLAGS += -Og
	CFLAGS += -D_DEBUG -O0
endif

ifeq ($(CONFIGURATION), release)
	CFLAGS += -O2 -DNDEBUG
endif

SOURCES = $(wildcard *.cpp)
OBJECTS = $(addprefix $(OBJ_DIR)/, $(SOURCES:.cpp=.o))

# Commands
RM = rm -f
CP = cp -f
MKDIR = mkdir -p
MAKE = make

all: dirs $(MODULE_NAME)
build: dirs $(MODULE_NAME)

dirs:
	@-$(MKDIR) $(OUT_DIR)
	@-$(MKDIR) $(OBJ_DIR)
	
$(MODULE_NAME): $(OBJECTS)
	@echo " + Linking..."
	@$(LINK) -o $(OUT_DIR)/$@ $(OBJECTS) $(LFLAGS)
	@echo " + Module created at $(OUT_DIR)/$@"

$(OBJ_DIR)/%.o: %.cpp
	@echo " + Compiling '$<'"
	@$(CC) $(CFLAGS) -o $@ -c $<
	
clean:
	@$(RM) $(OBJECTS)
	@$(RM) $(OUT_DIR)/$(MODULE_NAME)
	@echo " + Clean!"
  
