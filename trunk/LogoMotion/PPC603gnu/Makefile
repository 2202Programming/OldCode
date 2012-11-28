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
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/LogoMotion



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
PROJECT_TARGETS = LogoMotion/$(MODE_DIR)/LogoMotion.out \
	 LogoMotion_partialImage/$(MODE_DIR)/LogoMotion_partialImage.o


#Rules

# LogoMotion
ifeq ($(DEBUG_MODE),1)
LogoMotion/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
LogoMotion/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
LogoMotion/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
LogoMotion/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
LogoMotion/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
LogoMotion/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
LogoMotion/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
LogoMotion/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
LogoMotion/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
LogoMotion/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
LogoMotion/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
LogoMotion/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
LogoMotion/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 
LogoMotion/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
LogoMotion/$(MODE_DIR)/% : IDE_DEFINES = 
LogoMotion/$(MODE_DIR)/% : PROJECT_TYPE = DKM
LogoMotion/$(MODE_DIR)/% : DEFINES = 
LogoMotion/$(MODE_DIR)/% : EXPAND_DBG = 0
LogoMotion/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
LogoMotion/$(MODE_DIR)/% : CPU = PPC603
LogoMotion/$(MODE_DIR)/% : TOOL_FAMILY = gnu
LogoMotion/$(MODE_DIR)/% : TOOL = gnu
LogoMotion/$(MODE_DIR)/% : TOOL_PATH = 
LogoMotion/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
LogoMotion/$(MODE_DIR)/% : LIBPATH = 
LogoMotion/$(MODE_DIR)/% : LIBS = 
LogoMotion/$(MODE_DIR)/% : OBJ_DIR := LogoMotion/$(MODE_DIR)

OBJECTS_LogoMotion =

LogoMotion/$(MODE_DIR)/LogoMotion.out : $(OBJECTS_LogoMotion)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";rm -f "$@";nmppc $(OBJECTS_LogoMotion) $(IDE_LIBRARIES) | tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -fdollars-in-identifiers -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o $(OBJ_DIR)/ctdt.o -c $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X -T $(WIND_BASE)/target/h/tool/gnu/ldscripts/link.OUT -o "$@" $(OBJ_DIR)/ctdt.o $(OBJECTS_LogoMotion) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

LogoMotion/$(MODE_DIR)/LogoMotion_compile_file : $(FILE) ;

_clean :: LogoMotion/$(MODE_DIR)/LogoMotion_clean

LogoMotion/$(MODE_DIR)/LogoMotion_clean : 
	$(TRACE_FLAG)if [ -d "LogoMotion" ]; then cd "LogoMotion"; rm -rf $(MODE_DIR); fi


# LogoMotion_partialImage
ifeq ($(DEBUG_MODE),1)
LogoMotion_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
LogoMotion_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
LogoMotion_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
LogoMotion_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
LogoMotion_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
LogoMotion_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
LogoMotion_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
LogoMotion_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
LogoMotion_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
LogoMotion_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
LogoMotion_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
LogoMotion_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
LogoMotion_partialImage/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 
LogoMotion_partialImage/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
LogoMotion_partialImage/$(MODE_DIR)/% : IDE_DEFINES = 
LogoMotion_partialImage/$(MODE_DIR)/% : PROJECT_TYPE = DKM
LogoMotion_partialImage/$(MODE_DIR)/% : DEFINES = 
LogoMotion_partialImage/$(MODE_DIR)/% : EXPAND_DBG = 0
LogoMotion_partialImage/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
LogoMotion_partialImage/$(MODE_DIR)/% : CPU = PPC603
LogoMotion_partialImage/$(MODE_DIR)/% : TOOL_FAMILY = gnu
LogoMotion_partialImage/$(MODE_DIR)/% : TOOL = gnu
LogoMotion_partialImage/$(MODE_DIR)/% : TOOL_PATH = 
LogoMotion_partialImage/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
LogoMotion_partialImage/$(MODE_DIR)/% : LIBPATH = 
LogoMotion_partialImage/$(MODE_DIR)/% : LIBS = 
LogoMotion_partialImage/$(MODE_DIR)/% : OBJ_DIR := LogoMotion_partialImage/$(MODE_DIR)

OBJECTS_LogoMotion_partialImage =

LogoMotion_partialImage/$(MODE_DIR)/LogoMotion_partialImage.o : $(OBJECTS_LogoMotion_partialImage)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X  -o "$@" $(OBJECTS_LogoMotion_partialImage) $(LIBPATH) $(LIBS) $(ADDED_LIBPATH) $(ADDED_LIBS)  && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

LogoMotion_partialImage/$(MODE_DIR)/LogoMotion_partialImage_compile_file : $(FILE) ;

_clean :: LogoMotion_partialImage/$(MODE_DIR)/LogoMotion_partialImage_clean

LogoMotion_partialImage/$(MODE_DIR)/LogoMotion_partialImage_clean : 
	$(TRACE_FLAG)if [ -d "LogoMotion_partialImage" ]; then cd "LogoMotion_partialImage"; rm -rf $(MODE_DIR); fi

force : 



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
