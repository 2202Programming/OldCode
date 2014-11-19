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
WS_ROOT_DIR := C:/FrcRepository
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/Hohenheim2_0



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
PROJECT_TARGETS = Hohenheim2_0/$(MODE_DIR)/Hohenheim2_0.out \
	 Hohenheim2_0_partialImage/$(MODE_DIR)/Hohenheim2_0_partialImage.o


#Rules

# Hohenheim2_0
ifeq ($(DEBUG_MODE),1)
Hohenheim2_0/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
Hohenheim2_0/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
Hohenheim2_0/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
Hohenheim2_0/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Hohenheim2_0/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Hohenheim2_0/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
Hohenheim2_0/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
Hohenheim2_0/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
Hohenheim2_0/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
Hohenheim2_0/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Hohenheim2_0/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Hohenheim2_0/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
Hohenheim2_0/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 
Hohenheim2_0/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
Hohenheim2_0/$(MODE_DIR)/% : IDE_DEFINES = 
Hohenheim2_0/$(MODE_DIR)/% : PROJECT_TYPE = DKM
Hohenheim2_0/$(MODE_DIR)/% : DEFINES = 
Hohenheim2_0/$(MODE_DIR)/% : EXPAND_DBG = 0
Hohenheim2_0/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
Hohenheim2_0/$(MODE_DIR)/% : CPU = PPC603
Hohenheim2_0/$(MODE_DIR)/% : TOOL_FAMILY = gnu
Hohenheim2_0/$(MODE_DIR)/% : TOOL = gnu
Hohenheim2_0/$(MODE_DIR)/% : TOOL_PATH = 
Hohenheim2_0/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
Hohenheim2_0/$(MODE_DIR)/% : LIBPATH = 
Hohenheim2_0/$(MODE_DIR)/% : LIBS = 
Hohenheim2_0/$(MODE_DIR)/% : OBJ_DIR := Hohenheim2_0/$(MODE_DIR)

OBJECTS_Hohenheim2_0 = Hohenheim2_0_partialImage/$(MODE_DIR)/Hohenheim2_0_partialImage.o

Hohenheim2_0/$(MODE_DIR)/Hohenheim2_0.out : $(OBJECTS_Hohenheim2_0)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";rm -f "$@";nmppc $(OBJECTS_Hohenheim2_0) $(IDE_LIBRARIES) | tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -fdollars-in-identifiers -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o $(OBJ_DIR)/ctdt.o -c $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X -T $(WIND_BASE)/target/h/tool/gnu/ldscripts/link.OUT -o "$@" $(OBJ_DIR)/ctdt.o $(OBJECTS_Hohenheim2_0) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

Hohenheim2_0/$(MODE_DIR)/Hohenheim2_0_compile_file : $(FILE) ;

_clean :: Hohenheim2_0/$(MODE_DIR)/Hohenheim2_0_clean

Hohenheim2_0/$(MODE_DIR)/Hohenheim2_0_clean : 
	$(TRACE_FLAG)if [ -d "Hohenheim2_0" ]; then cd "Hohenheim2_0"; rm -rf $(MODE_DIR); fi


# Hohenheim2_0_partialImage
ifeq ($(DEBUG_MODE),1)
Hohenheim2_0_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
Hohenheim2_0_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
Hohenheim2_0_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
Hohenheim2_0_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Hohenheim2_0_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Hohenheim2_0_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
Hohenheim2_0_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
Hohenheim2_0_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
Hohenheim2_0_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
Hohenheim2_0_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Hohenheim2_0_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Hohenheim2_0_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
Hohenheim2_0_partialImage/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 
Hohenheim2_0_partialImage/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
Hohenheim2_0_partialImage/$(MODE_DIR)/% : IDE_DEFINES = 
Hohenheim2_0_partialImage/$(MODE_DIR)/% : PROJECT_TYPE = DKM
Hohenheim2_0_partialImage/$(MODE_DIR)/% : DEFINES = 
Hohenheim2_0_partialImage/$(MODE_DIR)/% : EXPAND_DBG = 0
Hohenheim2_0_partialImage/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
Hohenheim2_0_partialImage/$(MODE_DIR)/% : CPU = PPC603
Hohenheim2_0_partialImage/$(MODE_DIR)/% : TOOL_FAMILY = gnu
Hohenheim2_0_partialImage/$(MODE_DIR)/% : TOOL = gnu
Hohenheim2_0_partialImage/$(MODE_DIR)/% : TOOL_PATH = 
Hohenheim2_0_partialImage/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
Hohenheim2_0_partialImage/$(MODE_DIR)/% : LIBPATH = 
Hohenheim2_0_partialImage/$(MODE_DIR)/% : LIBS = 
Hohenheim2_0_partialImage/$(MODE_DIR)/% : OBJ_DIR := Hohenheim2_0_partialImage/$(MODE_DIR)

Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/DriveControl.o : $(PRJ_ROOT_DIR)/DriveControl.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/Hohenheim.o : $(PRJ_ROOT_DIR)/Hohenheim.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/PIDControlSubClass.o : $(PRJ_ROOT_DIR)/PIDControlSubClass.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/PneumaticsControl.o : $(PRJ_ROOT_DIR)/PneumaticsControl.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/ShooterControl.o : $(PRJ_ROOT_DIR)/ShooterControl.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/XboxController.o : $(PRJ_ROOT_DIR)/XboxController.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


OBJECTS_Hohenheim2_0_partialImage = Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/DriveControl.o \
	 Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/Hohenheim.o \
	 Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/PIDControlSubClass.o \
	 Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/PneumaticsControl.o \
	 Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/ShooterControl.o \
	 Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/XboxController.o

Hohenheim2_0_partialImage/$(MODE_DIR)/Hohenheim2_0_partialImage.o : $(OBJECTS_Hohenheim2_0_partialImage)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X  -o "$@" $(OBJECTS_Hohenheim2_0_partialImage) $(LIBPATH) $(LIBS) $(ADDED_LIBPATH) $(ADDED_LIBS)  && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

Hohenheim2_0_partialImage/$(MODE_DIR)/Hohenheim2_0_partialImage_compile_file : $(FILE) ;

_clean :: Hohenheim2_0_partialImage/$(MODE_DIR)/Hohenheim2_0_partialImage_clean

Hohenheim2_0_partialImage/$(MODE_DIR)/Hohenheim2_0_partialImage_clean : 
	$(TRACE_FLAG)if [ -d "Hohenheim2_0_partialImage" ]; then cd "Hohenheim2_0_partialImage"; rm -rf $(MODE_DIR); fi

force : 

DEP_FILES := Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/DriveControl.d Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/Hohenheim.d Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/PIDControlSubClass.d \
	 Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/PneumaticsControl.d Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/ShooterControl.d Hohenheim2_0_partialImage/$(MODE_DIR)/Objects/Hohenheim2_0/XboxController.d
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
