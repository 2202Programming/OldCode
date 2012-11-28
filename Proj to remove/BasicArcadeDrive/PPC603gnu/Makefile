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
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/BasicArcadeDrive



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
PROJECT_TARGETS = BasicArcadeDrive/$(MODE_DIR)/BasicArcadeDrive.out \
	 BasicArcadeDrive_partialImage/$(MODE_DIR)/BasicArcadeDrive_partialImage.o


#Rules

# BasicArcadeDrive
ifeq ($(DEBUG_MODE),1)
BasicArcadeDrive/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
BasicArcadeDrive/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
BasicArcadeDrive/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
BasicArcadeDrive/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
BasicArcadeDrive/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
BasicArcadeDrive/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
BasicArcadeDrive/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
BasicArcadeDrive/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
BasicArcadeDrive/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
BasicArcadeDrive/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
BasicArcadeDrive/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
BasicArcadeDrive/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
BasicArcadeDrive/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 
BasicArcadeDrive/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
BasicArcadeDrive/$(MODE_DIR)/% : IDE_DEFINES = 
BasicArcadeDrive/$(MODE_DIR)/% : PROJECT_TYPE = DKM
BasicArcadeDrive/$(MODE_DIR)/% : DEFINES = 
BasicArcadeDrive/$(MODE_DIR)/% : EXPAND_DBG = 0
BasicArcadeDrive/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
BasicArcadeDrive/$(MODE_DIR)/% : CPU = PPC603
BasicArcadeDrive/$(MODE_DIR)/% : TOOL_FAMILY = gnu
BasicArcadeDrive/$(MODE_DIR)/% : TOOL = gnu
BasicArcadeDrive/$(MODE_DIR)/% : TOOL_PATH = 
BasicArcadeDrive/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
BasicArcadeDrive/$(MODE_DIR)/% : LIBPATH = 
BasicArcadeDrive/$(MODE_DIR)/% : LIBS = 
BasicArcadeDrive/$(MODE_DIR)/% : OBJ_DIR := BasicArcadeDrive/$(MODE_DIR)

OBJECTS_BasicArcadeDrive = BasicArcadeDrive_partialImage/$(MODE_DIR)/BasicArcadeDrive_partialImage.o

BasicArcadeDrive/$(MODE_DIR)/BasicArcadeDrive.out : $(OBJECTS_BasicArcadeDrive)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";rm -f "$@";nmppc $(OBJECTS_BasicArcadeDrive) $(IDE_LIBRARIES) | tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -fdollars-in-identifiers -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o $(OBJ_DIR)/ctdt.o -c $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X -T $(WIND_BASE)/target/h/tool/gnu/ldscripts/link.OUT -o "$@" $(OBJ_DIR)/ctdt.o $(OBJECTS_BasicArcadeDrive) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

BasicArcadeDrive/$(MODE_DIR)/BasicArcadeDrive_compile_file : $(FILE) ;

_clean :: BasicArcadeDrive/$(MODE_DIR)/BasicArcadeDrive_clean

BasicArcadeDrive/$(MODE_DIR)/BasicArcadeDrive_clean : 
	$(TRACE_FLAG)if [ -d "BasicArcadeDrive" ]; then cd "BasicArcadeDrive"; rm -rf $(MODE_DIR); fi


# BasicArcadeDrive_partialImage
ifeq ($(DEBUG_MODE),1)
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : IDE_DEFINES = 
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : PROJECT_TYPE = DKM
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : DEFINES = 
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : EXPAND_DBG = 0
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : CPU = PPC603
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : TOOL_FAMILY = gnu
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : TOOL = gnu
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : TOOL_PATH = 
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : LIBPATH = 
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : LIBS = 
BasicArcadeDrive_partialImage/$(MODE_DIR)/% : OBJ_DIR := BasicArcadeDrive_partialImage/$(MODE_DIR)

BasicArcadeDrive_partialImage/$(MODE_DIR)/Objects/BasicArcadeDrive/ArcadeRobot.o : $(PRJ_ROOT_DIR)/ArcadeRobot.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


OBJECTS_BasicArcadeDrive_partialImage = BasicArcadeDrive_partialImage/$(MODE_DIR)/Objects/BasicArcadeDrive/ArcadeRobot.o

BasicArcadeDrive_partialImage/$(MODE_DIR)/BasicArcadeDrive_partialImage.o : $(OBJECTS_BasicArcadeDrive_partialImage)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X  -o "$@" $(OBJECTS_BasicArcadeDrive_partialImage) $(LIBPATH) $(LIBS) $(ADDED_LIBPATH) $(ADDED_LIBS)  && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

BasicArcadeDrive_partialImage/$(MODE_DIR)/BasicArcadeDrive_partialImage_compile_file : $(FILE) ;

_clean :: BasicArcadeDrive_partialImage/$(MODE_DIR)/BasicArcadeDrive_partialImage_clean

BasicArcadeDrive_partialImage/$(MODE_DIR)/BasicArcadeDrive_partialImage_clean : 
	$(TRACE_FLAG)if [ -d "BasicArcadeDrive_partialImage" ]; then cd "BasicArcadeDrive_partialImage"; rm -rf $(MODE_DIR); fi

force : 

DEP_FILES := BasicArcadeDrive_partialImage/$(MODE_DIR)/Objects/BasicArcadeDrive/ArcadeRobot.d
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
