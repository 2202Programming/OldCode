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
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/FourWheel



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
PROJECT_TARGETS = FourWheel/$(MODE_DIR)/FourWheel.out \
	 FourWheel_PartialImage/$(MODE_DIR)/FourWheel_PartialImage.o


#Rules

# FourWheel
ifeq ($(DEBUG_MODE),1)
FourWheel/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
FourWheel/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
FourWheel/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
FourWheel/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
FourWheel/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
FourWheel/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
FourWheel/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
FourWheel/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
FourWheel/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
FourWheel/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
FourWheel/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
FourWheel/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
FourWheel/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 
FourWheel/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
FourWheel/$(MODE_DIR)/% : IDE_DEFINES = 
FourWheel/$(MODE_DIR)/% : PROJECT_TYPE = DKM
FourWheel/$(MODE_DIR)/% : DEFINES = 
FourWheel/$(MODE_DIR)/% : EXPAND_DBG = 0
FourWheel/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
FourWheel/$(MODE_DIR)/% : CPU = PPC603
FourWheel/$(MODE_DIR)/% : TOOL_FAMILY = gnu
FourWheel/$(MODE_DIR)/% : TOOL = gnu
FourWheel/$(MODE_DIR)/% : TOOL_PATH = 
FourWheel/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
FourWheel/$(MODE_DIR)/% : LIBPATH = 
FourWheel/$(MODE_DIR)/% : LIBS = 
FourWheel/$(MODE_DIR)/% : OBJ_DIR := FourWheel/$(MODE_DIR)

OBJECTS_FourWheel = FourWheel_PartialImage/$(MODE_DIR)/FourWheel_PartialImage.o

FourWheel/$(MODE_DIR)/FourWheel.out : $(OBJECTS_FourWheel)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";rm -f "$@";nmppc $(OBJECTS_FourWheel) $(IDE_LIBRARIES) | tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -fdollars-in-identifiers -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o $(OBJ_DIR)/ctdt.o -c $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X -T $(WIND_BASE)/target/h/tool/gnu/ldscripts/link.OUT -o "$@" $(OBJ_DIR)/ctdt.o $(OBJECTS_FourWheel) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

FourWheel/$(MODE_DIR)/FourWheel_compile_file : $(FILE) ;

_clean :: FourWheel/$(MODE_DIR)/FourWheel_clean

FourWheel/$(MODE_DIR)/FourWheel_clean : 
	$(TRACE_FLAG)if [ -d "FourWheel" ]; then cd "FourWheel"; rm -rf $(MODE_DIR); fi


# FourWheel_PartialImage
ifeq ($(DEBUG_MODE),1)
FourWheel_PartialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
FourWheel_PartialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
FourWheel_PartialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
FourWheel_PartialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
FourWheel_PartialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
FourWheel_PartialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
FourWheel_PartialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
FourWheel_PartialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
FourWheel_PartialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
FourWheel_PartialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
FourWheel_PartialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
FourWheel_PartialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
FourWheel_PartialImage/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 
FourWheel_PartialImage/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
FourWheel_PartialImage/$(MODE_DIR)/% : IDE_DEFINES = 
FourWheel_PartialImage/$(MODE_DIR)/% : PROJECT_TYPE = DKM
FourWheel_PartialImage/$(MODE_DIR)/% : DEFINES = 
FourWheel_PartialImage/$(MODE_DIR)/% : EXPAND_DBG = 0
FourWheel_PartialImage/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
FourWheel_PartialImage/$(MODE_DIR)/% : CPU = PPC603
FourWheel_PartialImage/$(MODE_DIR)/% : TOOL_FAMILY = gnu
FourWheel_PartialImage/$(MODE_DIR)/% : TOOL = gnu
FourWheel_PartialImage/$(MODE_DIR)/% : TOOL_PATH = 
FourWheel_PartialImage/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
FourWheel_PartialImage/$(MODE_DIR)/% : LIBPATH = 
FourWheel_PartialImage/$(MODE_DIR)/% : LIBS = 
FourWheel_PartialImage/$(MODE_DIR)/% : OBJ_DIR := FourWheel_PartialImage/$(MODE_DIR)

FourWheel_PartialImage/$(MODE_DIR)/Objects/FourWheel/MyRobot.o : $(PRJ_ROOT_DIR)/MyRobot.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


OBJECTS_FourWheel_PartialImage = FourWheel_PartialImage/$(MODE_DIR)/Objects/FourWheel/MyRobot.o

FourWheel_PartialImage/$(MODE_DIR)/FourWheel_PartialImage.o : $(OBJECTS_FourWheel_PartialImage)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X  -o "$@" $(OBJECTS_FourWheel_PartialImage) $(LIBPATH) $(LIBS) $(ADDED_LIBPATH) $(ADDED_LIBS)  && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

FourWheel_PartialImage/$(MODE_DIR)/FourWheel_PartialImage_compile_file : $(FILE) ;

_clean :: FourWheel_PartialImage/$(MODE_DIR)/FourWheel_PartialImage_clean

FourWheel_PartialImage/$(MODE_DIR)/FourWheel_PartialImage_clean : 
	$(TRACE_FLAG)if [ -d "FourWheel_PartialImage" ]; then cd "FourWheel_PartialImage"; rm -rf $(MODE_DIR); fi

force : 

DEP_FILES := FourWheel_PartialImage/$(MODE_DIR)/Objects/FourWheel/MyRobot.d
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
