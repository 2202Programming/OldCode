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
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/CSimpleTemplate



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

IDE_INCLUDES = -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/WPILib/CInterfaces -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 

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
PROJECT_TARGETS = CSimpleTemplate/$(MODE_DIR)/CSimpleTemplate.out \
	 CSimpleTemplate_partialImage/$(MODE_DIR)/CSimpleTemplate_partialImage.o


#Rules

# CSimpleTemplate
ifeq ($(DEBUG_MODE),1)
CSimpleTemplate/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
CSimpleTemplate/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
CSimpleTemplate/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
CSimpleTemplate/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
CSimpleTemplate/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
CSimpleTemplate/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
CSimpleTemplate/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
CSimpleTemplate/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
CSimpleTemplate/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
CSimpleTemplate/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
CSimpleTemplate/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
CSimpleTemplate/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
CSimpleTemplate/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/WPILib/CInterfaces -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 
CSimpleTemplate/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
CSimpleTemplate/$(MODE_DIR)/% : IDE_DEFINES = 
CSimpleTemplate/$(MODE_DIR)/% : PROJECT_TYPE = DKM
CSimpleTemplate/$(MODE_DIR)/% : DEFINES = 
CSimpleTemplate/$(MODE_DIR)/% : EXPAND_DBG = 0
CSimpleTemplate/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
CSimpleTemplate/$(MODE_DIR)/% : CPU = PPC603
CSimpleTemplate/$(MODE_DIR)/% : TOOL_FAMILY = gnu
CSimpleTemplate/$(MODE_DIR)/% : TOOL = gnu
CSimpleTemplate/$(MODE_DIR)/% : TOOL_PATH = 
CSimpleTemplate/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
CSimpleTemplate/$(MODE_DIR)/% : LIBPATH = 
CSimpleTemplate/$(MODE_DIR)/% : LIBS = 
CSimpleTemplate/$(MODE_DIR)/% : OBJ_DIR := CSimpleTemplate/$(MODE_DIR)

OBJECTS_CSimpleTemplate = CSimpleTemplate_partialImage/$(MODE_DIR)/CSimpleTemplate_partialImage.o

CSimpleTemplate/$(MODE_DIR)/CSimpleTemplate.out : $(OBJECTS_CSimpleTemplate)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";rm -f "$@";nmppc $(OBJECTS_CSimpleTemplate) $(IDE_LIBRARIES) | tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -fdollars-in-identifiers -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o $(OBJ_DIR)/ctdt.o -c $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X -T $(WIND_BASE)/target/h/tool/gnu/ldscripts/link.OUT -o "$@" $(OBJ_DIR)/ctdt.o $(OBJECTS_CSimpleTemplate) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

CSimpleTemplate/$(MODE_DIR)/CSimpleTemplate_compile_file : $(FILE) ;

_clean :: CSimpleTemplate/$(MODE_DIR)/CSimpleTemplate_clean

CSimpleTemplate/$(MODE_DIR)/CSimpleTemplate_clean : 
	$(TRACE_FLAG)if [ -d "CSimpleTemplate" ]; then cd "CSimpleTemplate"; rm -rf $(MODE_DIR); fi


# CSimpleTemplate_partialImage
ifeq ($(DEBUG_MODE),1)
CSimpleTemplate_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
CSimpleTemplate_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
CSimpleTemplate_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
CSimpleTemplate_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
CSimpleTemplate_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
CSimpleTemplate_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
CSimpleTemplate_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
CSimpleTemplate_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
CSimpleTemplate_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
CSimpleTemplate_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
CSimpleTemplate_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
CSimpleTemplate_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
CSimpleTemplate_partialImage/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/WPILib/CInterfaces -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 
CSimpleTemplate_partialImage/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
CSimpleTemplate_partialImage/$(MODE_DIR)/% : IDE_DEFINES = 
CSimpleTemplate_partialImage/$(MODE_DIR)/% : PROJECT_TYPE = DKM
CSimpleTemplate_partialImage/$(MODE_DIR)/% : DEFINES = 
CSimpleTemplate_partialImage/$(MODE_DIR)/% : EXPAND_DBG = 0
CSimpleTemplate_partialImage/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
CSimpleTemplate_partialImage/$(MODE_DIR)/% : CPU = PPC603
CSimpleTemplate_partialImage/$(MODE_DIR)/% : TOOL_FAMILY = gnu
CSimpleTemplate_partialImage/$(MODE_DIR)/% : TOOL = gnu
CSimpleTemplate_partialImage/$(MODE_DIR)/% : TOOL_PATH = 
CSimpleTemplate_partialImage/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
CSimpleTemplate_partialImage/$(MODE_DIR)/% : LIBPATH = 
CSimpleTemplate_partialImage/$(MODE_DIR)/% : LIBS = 
CSimpleTemplate_partialImage/$(MODE_DIR)/% : OBJ_DIR := CSimpleTemplate_partialImage/$(MODE_DIR)

CSimpleTemplate_partialImage/$(MODE_DIR)/Objects/CSimpleTemplate/CSimpleTemplate.o : $(PRJ_ROOT_DIR)/CSimpleTemplate.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


OBJECTS_CSimpleTemplate_partialImage = CSimpleTemplate_partialImage/$(MODE_DIR)/Objects/CSimpleTemplate/CSimpleTemplate.o

CSimpleTemplate_partialImage/$(MODE_DIR)/CSimpleTemplate_partialImage.o : $(OBJECTS_CSimpleTemplate_partialImage)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X  -o "$@" $(OBJECTS_CSimpleTemplate_partialImage) $(LIBPATH) $(LIBS) $(ADDED_LIBPATH) $(ADDED_LIBS)  && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

CSimpleTemplate_partialImage/$(MODE_DIR)/CSimpleTemplate_partialImage_compile_file : $(FILE) ;

_clean :: CSimpleTemplate_partialImage/$(MODE_DIR)/CSimpleTemplate_partialImage_clean

CSimpleTemplate_partialImage/$(MODE_DIR)/CSimpleTemplate_partialImage_clean : 
	$(TRACE_FLAG)if [ -d "CSimpleTemplate_partialImage" ]; then cd "CSimpleTemplate_partialImage"; rm -rf $(MODE_DIR); fi

force : 

DEP_FILES := CSimpleTemplate_partialImage/$(MODE_DIR)/Objects/CSimpleTemplate/CSimpleTemplate.d
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
