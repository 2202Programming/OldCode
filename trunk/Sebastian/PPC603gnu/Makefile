# Wind River Workbench generated Makefile.
# Do not edit!!!
#
# The file ".wrmakefile" is the template used by the Wind River Workbench to
# generate the makefiles of this project. Add user-specific build targets and
# make rules only(!) in this project's ".wrmakefile" file. These will then be
# automatically dumped into the makefiles.

WIND_HOME := $(subst \,/,$(WIND_HOME))
WIND_BASE := $(subst \,/,$(WIND_BASE))
WIND_USR := $(subst \,/,$(WIND_USR))

all : pre_build main_all post_build

_clean ::
	@echo "make: removing targets and objects of `pwd`"

TRACE=0
TRACEON=$(TRACE:0=@)
TRACE_FLAG=$(TRACEON:1=)

MAKEFILE := Makefile

FLEXIBLE_BUILD := 1

BUILD_SPEC = PPC603gnu
DEBUG_MODE = 1
ifeq ($(DEBUG_MODE),1)
MODE_DIR := Debug
else
MODE_DIR := NonDebug
endif
OBJ_DIR := .
WS_ROOT_DIR := C:/beast
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/Sebastian



#Global Build Macros
PROJECT_TYPE = DKM
DEFINES = 
EXPAND_DBG = 0


#BuildSpec specific Build Macros
VX_CPU_FAMILY = ppc
CPU = PPC603
TOOL_FAMILY = gnu
TOOL = gnu
TOOL_PATH = 
CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
LIBPATH = 
LIBS = 

IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 

IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 

IDE_DEFINES = 



#BuildTool flags
ifeq ($(DEBUG_MODE),1)
DEBUGFLAGS_C-Compiler = -g
DEBUGFLAGS_C++-Compiler = -g
DEBUGFLAGS_Linker = -g
DEBUGFLAGS_Partial-Image-Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = -g
else
DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_Partial-Image-Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif


#Project Targets
PROJECT_TARGETS = Sebastian/$(MODE_DIR)/Sebastian.out \
	 Sebastian_partialImage/$(MODE_DIR)/Sebastian_partialImage.o


#Rules

# Sebastian
ifeq ($(DEBUG_MODE),1)
Sebastian/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
Sebastian/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
Sebastian/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
Sebastian/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Sebastian/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Sebastian/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
Sebastian/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
Sebastian/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
Sebastian/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
Sebastian/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Sebastian/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Sebastian/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
Sebastian/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 
Sebastian/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
Sebastian/$(MODE_DIR)/% : IDE_DEFINES = 
Sebastian/$(MODE_DIR)/% : PROJECT_TYPE = DKM
Sebastian/$(MODE_DIR)/% : DEFINES = 
Sebastian/$(MODE_DIR)/% : EXPAND_DBG = 0
Sebastian/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
Sebastian/$(MODE_DIR)/% : CPU = PPC603
Sebastian/$(MODE_DIR)/% : TOOL_FAMILY = gnu
Sebastian/$(MODE_DIR)/% : TOOL = gnu
Sebastian/$(MODE_DIR)/% : TOOL_PATH = 
Sebastian/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
Sebastian/$(MODE_DIR)/% : LIBPATH = 
Sebastian/$(MODE_DIR)/% : LIBS = 
Sebastian/$(MODE_DIR)/% : OBJ_DIR := Sebastian/$(MODE_DIR)

OBJECTS_Sebastian = Sebastian_partialImage/$(MODE_DIR)/Sebastian_partialImage.o

Sebastian/$(MODE_DIR)/Sebastian.out : $(OBJECTS_Sebastian)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";rm -f "$@";nmppc $(OBJECTS_Sebastian) $(IDE_LIBRARIES) | tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -fdollars-in-identifiers -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o $(OBJ_DIR)/ctdt.o -c $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X -T $(WIND_BASE)/target/h/tool/gnu/ldscripts/link.OUT -o "$@" $(OBJ_DIR)/ctdt.o $(OBJECTS_Sebastian) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

Sebastian/$(MODE_DIR)/Sebastian_compile_file : $(FILE) ;

_clean :: Sebastian/$(MODE_DIR)/Sebastian_clean

Sebastian/$(MODE_DIR)/Sebastian_clean : 
	$(TRACE_FLAG)if [ -d "Sebastian" ]; then cd "Sebastian"; rm -rf $(MODE_DIR); fi


# Sebastian_partialImage
ifeq ($(DEBUG_MODE),1)
Sebastian_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
Sebastian_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
Sebastian_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
Sebastian_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Sebastian_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Sebastian_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
Sebastian_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
Sebastian_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
Sebastian_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
Sebastian_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Sebastian_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Sebastian_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
Sebastian_partialImage/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 
Sebastian_partialImage/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
Sebastian_partialImage/$(MODE_DIR)/% : IDE_DEFINES = 
Sebastian_partialImage/$(MODE_DIR)/% : PROJECT_TYPE = DKM
Sebastian_partialImage/$(MODE_DIR)/% : DEFINES = 
Sebastian_partialImage/$(MODE_DIR)/% : EXPAND_DBG = 0
Sebastian_partialImage/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
Sebastian_partialImage/$(MODE_DIR)/% : CPU = PPC603
Sebastian_partialImage/$(MODE_DIR)/% : TOOL_FAMILY = gnu
Sebastian_partialImage/$(MODE_DIR)/% : TOOL = gnu
Sebastian_partialImage/$(MODE_DIR)/% : TOOL_PATH = 
Sebastian_partialImage/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
Sebastian_partialImage/$(MODE_DIR)/% : LIBPATH = 
Sebastian_partialImage/$(MODE_DIR)/% : LIBS = 
Sebastian_partialImage/$(MODE_DIR)/% : OBJ_DIR := Sebastian_partialImage/$(MODE_DIR)

Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/DriveControl.o : $(PRJ_ROOT_DIR)/DriveControl.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/LCDShooterMotors.o : $(PRJ_ROOT_DIR)/LCDShooterMotors.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/LunacyShooterMotors.o : $(PRJ_ROOT_DIR)/LunacyShooterMotors.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/PoleVaultControl.o : $(PRJ_ROOT_DIR)/PoleVaultControl.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/SebastianRobot.o : $(PRJ_ROOT_DIR)/SebastianRobot.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/SebastianShooterMotors.o : $(PRJ_ROOT_DIR)/SebastianShooterMotors.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/ShooterControl.o : $(PRJ_ROOT_DIR)/ShooterControl.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/SonarSensor.o : $(PRJ_ROOT_DIR)/SonarSensor.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


OBJECTS_Sebastian_partialImage = Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/DriveControl.o \
	 Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/LCDShooterMotors.o \
	 Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/LunacyShooterMotors.o \
	 Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/PoleVaultControl.o \
	 Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/SebastianRobot.o \
	 Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/SebastianShooterMotors.o \
	 Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/ShooterControl.o \
	 Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/SonarSensor.o

Sebastian_partialImage/$(MODE_DIR)/Sebastian_partialImage.o : $(OBJECTS_Sebastian_partialImage)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X  -o "$@" $(OBJECTS_Sebastian_partialImage) $(LIBPATH) $(LIBS) $(ADDED_LIBPATH) $(ADDED_LIBS)  && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

Sebastian_partialImage/$(MODE_DIR)/Sebastian_partialImage_compile_file : $(FILE) ;

_clean :: Sebastian_partialImage/$(MODE_DIR)/Sebastian_partialImage_clean

Sebastian_partialImage/$(MODE_DIR)/Sebastian_partialImage_clean : 
	$(TRACE_FLAG)if [ -d "Sebastian_partialImage" ]; then cd "Sebastian_partialImage"; rm -rf $(MODE_DIR); fi

force : 

DEP_FILES := Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/DriveControl.d Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/LCDShooterMotors.d Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/LunacyShooterMotors.d \
	 Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/PoleVaultControl.d Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/SebastianRobot.d Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/SebastianShooterMotors.d \
	 Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/ShooterControl.d Sebastian_partialImage/$(MODE_DIR)/Objects/Sebastian/SonarSensor.d
-include $(DEP_FILES)



-include $(PRJ_ROOT_DIR)/*.makefile

-include *.makefile

main_all : external_build $(PROJECT_TARGETS)
	@echo "make: built targets of `pwd`"

# entry point for extending the build
external_build ::
	@echo ""

# main entry point for pre processing prior to the build
pre_build :: $(PRE_BUILD_STEP) generate_sources
	@echo ""

# entry point for generating sources prior to the build
generate_sources ::
	@echo ""

# main entry point for post processing after the build
post_build :: $(POST_BUILD_STEP) deploy_output
	@echo ""

# entry point for deploying output after the build
deploy_output ::
	@echo ""

clean :: external_clean $(CLEAN_STEP) _clean

# entry point for extending the build clean
external_clean ::
	@echo ""
