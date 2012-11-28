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
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/Deadzone



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
PROJECT_TARGETS = Deadzone/$(MODE_DIR)/Deadzone.out \
	 Deadzone_partialImage/$(MODE_DIR)/Deadzone_partialImage.o


#Rules

# Deadzone
ifeq ($(DEBUG_MODE),1)
Deadzone/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
Deadzone/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
Deadzone/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
Deadzone/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Deadzone/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Deadzone/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
Deadzone/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
Deadzone/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
Deadzone/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
Deadzone/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Deadzone/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Deadzone/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
Deadzone/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 
Deadzone/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
Deadzone/$(MODE_DIR)/% : IDE_DEFINES = 
Deadzone/$(MODE_DIR)/% : PROJECT_TYPE = DKM
Deadzone/$(MODE_DIR)/% : DEFINES = 
Deadzone/$(MODE_DIR)/% : EXPAND_DBG = 0
Deadzone/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
Deadzone/$(MODE_DIR)/% : CPU = PPC603
Deadzone/$(MODE_DIR)/% : TOOL_FAMILY = gnu
Deadzone/$(MODE_DIR)/% : TOOL = gnu
Deadzone/$(MODE_DIR)/% : TOOL_PATH = 
Deadzone/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
Deadzone/$(MODE_DIR)/% : LIBPATH = 
Deadzone/$(MODE_DIR)/% : LIBS = 
Deadzone/$(MODE_DIR)/% : OBJ_DIR := Deadzone/$(MODE_DIR)

OBJECTS_Deadzone = Deadzone_partialImage/$(MODE_DIR)/Deadzone_partialImage.o

Deadzone/$(MODE_DIR)/Deadzone.out : $(OBJECTS_Deadzone)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";rm -f "$@";nmppc $(OBJECTS_Deadzone) $(IDE_LIBRARIES) | tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -fdollars-in-identifiers -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o $(OBJ_DIR)/ctdt.o -c $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X -T $(WIND_BASE)/target/h/tool/gnu/ldscripts/link.OUT -o "$@" $(OBJ_DIR)/ctdt.o $(OBJECTS_Deadzone) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

Deadzone/$(MODE_DIR)/Deadzone_compile_file : $(FILE) ;

_clean :: Deadzone/$(MODE_DIR)/Deadzone_clean

Deadzone/$(MODE_DIR)/Deadzone_clean : 
	$(TRACE_FLAG)if [ -d "Deadzone" ]; then cd "Deadzone"; rm -rf $(MODE_DIR); fi


# Deadzone_partialImage
ifeq ($(DEBUG_MODE),1)
Deadzone_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
Deadzone_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
Deadzone_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
Deadzone_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Deadzone_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Deadzone_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
Deadzone_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
Deadzone_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
Deadzone_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
Deadzone_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Deadzone_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Deadzone_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
Deadzone_partialImage/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 
Deadzone_partialImage/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
Deadzone_partialImage/$(MODE_DIR)/% : IDE_DEFINES = 
Deadzone_partialImage/$(MODE_DIR)/% : PROJECT_TYPE = DKM
Deadzone_partialImage/$(MODE_DIR)/% : DEFINES = 
Deadzone_partialImage/$(MODE_DIR)/% : EXPAND_DBG = 0
Deadzone_partialImage/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
Deadzone_partialImage/$(MODE_DIR)/% : CPU = PPC603
Deadzone_partialImage/$(MODE_DIR)/% : TOOL_FAMILY = gnu
Deadzone_partialImage/$(MODE_DIR)/% : TOOL = gnu
Deadzone_partialImage/$(MODE_DIR)/% : TOOL_PATH = 
Deadzone_partialImage/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
Deadzone_partialImage/$(MODE_DIR)/% : LIBPATH = 
Deadzone_partialImage/$(MODE_DIR)/% : LIBS = 
Deadzone_partialImage/$(MODE_DIR)/% : OBJ_DIR := Deadzone_partialImage/$(MODE_DIR)

Deadzone_partialImage/$(MODE_DIR)/Objects/Deadzone/DriveControl.o : $(PRJ_ROOT_DIR)/DriveControl.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Deadzone_partialImage/$(MODE_DIR)/Objects/Deadzone/SonarSensor.o : $(PRJ_ROOT_DIR)/SonarSensor.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Deadzone_partialImage/$(MODE_DIR)/Objects/Deadzone/XboxController.o : $(PRJ_ROOT_DIR)/XboxController.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Deadzone_partialImage/$(MODE_DIR)/Objects/Deadzone/deadzone.o : $(PRJ_ROOT_DIR)/deadzone.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


OBJECTS_Deadzone_partialImage = Deadzone_partialImage/$(MODE_DIR)/Objects/Deadzone/DriveControl.o \
	 Deadzone_partialImage/$(MODE_DIR)/Objects/Deadzone/SonarSensor.o \
	 Deadzone_partialImage/$(MODE_DIR)/Objects/Deadzone/XboxController.o \
	 Deadzone_partialImage/$(MODE_DIR)/Objects/Deadzone/deadzone.o

Deadzone_partialImage/$(MODE_DIR)/Deadzone_partialImage.o : $(OBJECTS_Deadzone_partialImage)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X  -o "$@" $(OBJECTS_Deadzone_partialImage) $(LIBPATH) $(LIBS) $(ADDED_LIBPATH) $(ADDED_LIBS)  && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

Deadzone_partialImage/$(MODE_DIR)/Deadzone_partialImage_compile_file : $(FILE) ;

_clean :: Deadzone_partialImage/$(MODE_DIR)/Deadzone_partialImage_clean

Deadzone_partialImage/$(MODE_DIR)/Deadzone_partialImage_clean : 
	$(TRACE_FLAG)if [ -d "Deadzone_partialImage" ]; then cd "Deadzone_partialImage"; rm -rf $(MODE_DIR); fi

force : 

DEP_FILES := Deadzone_partialImage/$(MODE_DIR)/Objects/Deadzone/DriveControl.d Deadzone_partialImage/$(MODE_DIR)/Objects/Deadzone/SonarSensor.d Deadzone_partialImage/$(MODE_DIR)/Objects/Deadzone/XboxController.d \
	 Deadzone_partialImage/$(MODE_DIR)/Objects/Deadzone/deadzone.d
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
