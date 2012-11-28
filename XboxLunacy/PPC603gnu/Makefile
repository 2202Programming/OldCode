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
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/XboxLunacy



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
PROJECT_TARGETS = XboxLunacy/$(MODE_DIR)/XboxLunacy.out \
	 XboxLunacy_partialImage/$(MODE_DIR)/XboxLunacy_partialImage.o


#Rules

# XboxLunacy
ifeq ($(DEBUG_MODE),1)
XboxLunacy/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
XboxLunacy/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
XboxLunacy/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
XboxLunacy/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
XboxLunacy/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
XboxLunacy/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
XboxLunacy/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
XboxLunacy/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
XboxLunacy/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
XboxLunacy/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
XboxLunacy/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
XboxLunacy/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
XboxLunacy/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 
XboxLunacy/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
XboxLunacy/$(MODE_DIR)/% : IDE_DEFINES = 
XboxLunacy/$(MODE_DIR)/% : PROJECT_TYPE = DKM
XboxLunacy/$(MODE_DIR)/% : DEFINES = 
XboxLunacy/$(MODE_DIR)/% : EXPAND_DBG = 0
XboxLunacy/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
XboxLunacy/$(MODE_DIR)/% : CPU = PPC603
XboxLunacy/$(MODE_DIR)/% : TOOL_FAMILY = gnu
XboxLunacy/$(MODE_DIR)/% : TOOL = gnu
XboxLunacy/$(MODE_DIR)/% : TOOL_PATH = 
XboxLunacy/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
XboxLunacy/$(MODE_DIR)/% : LIBPATH = 
XboxLunacy/$(MODE_DIR)/% : LIBS = 
XboxLunacy/$(MODE_DIR)/% : OBJ_DIR := XboxLunacy/$(MODE_DIR)

XboxLunacy/$(MODE_DIR)/Objects/XboxLunacy/ArcadeDriver.o : $(PRJ_ROOT_DIR)/ArcadeDriver.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


XboxLunacy/$(MODE_DIR)/Objects/XboxLunacy/Robotlunacy.o : $(PRJ_ROOT_DIR)/Robotlunacy.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


XboxLunacy/$(MODE_DIR)/Objects/XboxLunacy/Toggler.o : $(PRJ_ROOT_DIR)/Toggler.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


OBJECTS_XboxLunacy = XboxLunacy/$(MODE_DIR)/Objects/XboxLunacy/ArcadeDriver.o \
	 XboxLunacy/$(MODE_DIR)/Objects/XboxLunacy/Robotlunacy.o \
	 XboxLunacy/$(MODE_DIR)/Objects/XboxLunacy/Toggler.o

XboxLunacy/$(MODE_DIR)/XboxLunacy.out : $(OBJECTS_XboxLunacy)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";rm -f "$@";nmppc $(OBJECTS_XboxLunacy) $(IDE_LIBRARIES) | tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -fdollars-in-identifiers -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o $(OBJ_DIR)/ctdt.o -c $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X -T $(WIND_BASE)/target/h/tool/gnu/ldscripts/link.OUT -o "$@" $(OBJ_DIR)/ctdt.o $(OBJECTS_XboxLunacy) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

XboxLunacy/$(MODE_DIR)/XboxLunacy_compile_file : $(FILE) ;

_clean :: XboxLunacy/$(MODE_DIR)/XboxLunacy_clean

XboxLunacy/$(MODE_DIR)/XboxLunacy_clean : 
	$(TRACE_FLAG)if [ -d "XboxLunacy" ]; then cd "XboxLunacy"; rm -rf $(MODE_DIR); fi


# XboxLunacy_partialImage
ifeq ($(DEBUG_MODE),1)
XboxLunacy_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
XboxLunacy_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
XboxLunacy_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
XboxLunacy_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
XboxLunacy_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
XboxLunacy_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
XboxLunacy_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
XboxLunacy_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
XboxLunacy_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
XboxLunacy_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
XboxLunacy_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
XboxLunacy_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
XboxLunacy_partialImage/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 
XboxLunacy_partialImage/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
XboxLunacy_partialImage/$(MODE_DIR)/% : IDE_DEFINES = 
XboxLunacy_partialImage/$(MODE_DIR)/% : PROJECT_TYPE = DKM
XboxLunacy_partialImage/$(MODE_DIR)/% : DEFINES = 
XboxLunacy_partialImage/$(MODE_DIR)/% : EXPAND_DBG = 0
XboxLunacy_partialImage/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
XboxLunacy_partialImage/$(MODE_DIR)/% : CPU = PPC603
XboxLunacy_partialImage/$(MODE_DIR)/% : TOOL_FAMILY = gnu
XboxLunacy_partialImage/$(MODE_DIR)/% : TOOL = gnu
XboxLunacy_partialImage/$(MODE_DIR)/% : TOOL_PATH = 
XboxLunacy_partialImage/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
XboxLunacy_partialImage/$(MODE_DIR)/% : LIBPATH = 
XboxLunacy_partialImage/$(MODE_DIR)/% : LIBS = 
XboxLunacy_partialImage/$(MODE_DIR)/% : OBJ_DIR := XboxLunacy_partialImage/$(MODE_DIR)

XboxLunacy_partialImage/$(MODE_DIR)/Objects/XboxLunacy/ArcadeDriver.o : $(PRJ_ROOT_DIR)/ArcadeDriver.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


XboxLunacy_partialImage/$(MODE_DIR)/Objects/XboxLunacy/Robotlunacy.o : $(PRJ_ROOT_DIR)/Robotlunacy.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


XboxLunacy_partialImage/$(MODE_DIR)/Objects/XboxLunacy/Toggler.o : $(PRJ_ROOT_DIR)/Toggler.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


OBJECTS_XboxLunacy_partialImage = XboxLunacy_partialImage/$(MODE_DIR)/Objects/XboxLunacy/ArcadeDriver.o \
	 XboxLunacy_partialImage/$(MODE_DIR)/Objects/XboxLunacy/Robotlunacy.o \
	 XboxLunacy_partialImage/$(MODE_DIR)/Objects/XboxLunacy/Toggler.o

XboxLunacy_partialImage/$(MODE_DIR)/XboxLunacy_partialImage.o : $(OBJECTS_XboxLunacy_partialImage)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X  -o "$@" $(OBJECTS_XboxLunacy_partialImage) $(LIBPATH) $(LIBS) $(ADDED_LIBPATH) $(ADDED_LIBS)  && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

XboxLunacy_partialImage/$(MODE_DIR)/XboxLunacy_partialImage_compile_file : $(FILE) ;

_clean :: XboxLunacy_partialImage/$(MODE_DIR)/XboxLunacy_partialImage_clean

XboxLunacy_partialImage/$(MODE_DIR)/XboxLunacy_partialImage_clean : 
	$(TRACE_FLAG)if [ -d "XboxLunacy_partialImage" ]; then cd "XboxLunacy_partialImage"; rm -rf $(MODE_DIR); fi

force : 

DEP_FILES := XboxLunacy/$(MODE_DIR)/Objects/XboxLunacy/ArcadeDriver.d XboxLunacy/$(MODE_DIR)/Objects/XboxLunacy/Robotlunacy.d XboxLunacy/$(MODE_DIR)/Objects/XboxLunacy/Toggler.d \
	 XboxLunacy_partialImage/$(MODE_DIR)/Objects/XboxLunacy/ArcadeDriver.d XboxLunacy_partialImage/$(MODE_DIR)/Objects/XboxLunacy/Robotlunacy.d XboxLunacy_partialImage/$(MODE_DIR)/Objects/XboxLunacy/Toggler.d
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
