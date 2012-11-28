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
WRVX_COMPBASE := $(subst \,/,$(WRVX_COMPBASE))

all : pre_build main_all post_build

_clean ::
	@echo "make: removing targets and objects of `pwd`"

TRACE=0
TRACEON=$(TRACE:0=@)
TRACE_FLAG=$(TRACEON:1=)

MAKEFILE := Makefile

FLEXIBLE_BUILD := 1

BUILD_SPEC = PPC32diab_RTP
DEBUG_MODE = 1
ifeq ($(DEBUG_MODE),1)
MODE_DIR := Debug
else
MODE_DIR := NonDebug
endif
OBJ_DIR := .
WS_ROOT_DIR := C:/beast
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/TestLibrary



#Global Build Macros
PROJECT_TYPE = SL
DEFINES = 
DO_STRIP = 0
SHAREDLIB_VERSION = 
EXPAND_DBG = 0


#BuildSpec specific Build Macros
VX_CPU_FAMILY = ppc
CPU = PPC32
TOOL_FAMILY = diab
TOOL = diab
TOOL_PATH = 
CC_ARCH_SPEC = -tPPCEH:rtp
LIBPATH = 
LIBS = -lstlstd

IDE_INCLUDES = -I$(WIND_BASE)/target/usr/h -I$(WIND_BASE)/target/usr/h/wrn/coreip 

IDE_LIBRARIES = 

IDE_DEFINES = 



#BuildTool flags
ifeq ($(DEBUG_MODE),1)
DEBUGFLAGS_C-Compiler = -g
DEBUGFLAGS_C++-Compiler = -g
DEBUGFLAGS_Shared-Library-Linker = 
DEBUGFLAGS_Static-Librarian = 
DEBUGFLAGS_Assembler = -g
else
DEBUGFLAGS_C-Compiler = -XO 
DEBUGFLAGS_C++-Compiler = -XO 
DEBUGFLAGS_Shared-Library-Linker = 
DEBUGFLAGS_Static-Librarian = 
DEBUGFLAGS_Assembler = -XO 
endif


#Project Targets
PROJECT_TARGETS = TestLibrary/$(MODE_DIR)/TestLibrary.so


#Rules

# TestLibrary
ifeq ($(DEBUG_MODE),1)
TestLibrary/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
TestLibrary/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
TestLibrary/$(MODE_DIR)/% : DEBUGFLAGS_Shared-Library-Linker = 
TestLibrary/$(MODE_DIR)/% : DEBUGFLAGS_Static-Librarian = 
TestLibrary/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
TestLibrary/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -XO 
TestLibrary/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -XO 
TestLibrary/$(MODE_DIR)/% : DEBUGFLAGS_Shared-Library-Linker = 
TestLibrary/$(MODE_DIR)/% : DEBUGFLAGS_Static-Librarian = 
TestLibrary/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -XO 
endif
TestLibrary/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/usr/h -I$(WIND_BASE)/target/usr/h/wrn/coreip 
TestLibrary/$(MODE_DIR)/% : IDE_LIBRARIES = 
TestLibrary/$(MODE_DIR)/% : IDE_DEFINES = 
TestLibrary/$(MODE_DIR)/% : PROJECT_TYPE = SL
TestLibrary/$(MODE_DIR)/% : DEFINES = 
TestLibrary/$(MODE_DIR)/% : DO_STRIP = 0
TestLibrary/$(MODE_DIR)/% : SHAREDLIB_VERSION = 
TestLibrary/$(MODE_DIR)/% : EXPAND_DBG = 0
TestLibrary/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
TestLibrary/$(MODE_DIR)/% : CPU = PPC32
TestLibrary/$(MODE_DIR)/% : TOOL_FAMILY = diab
TestLibrary/$(MODE_DIR)/% : TOOL = diab
TestLibrary/$(MODE_DIR)/% : TOOL_PATH = 
TestLibrary/$(MODE_DIR)/% : CC_ARCH_SPEC = -tPPCEH:rtp
TestLibrary/$(MODE_DIR)/% : LIBPATH = 
TestLibrary/$(MODE_DIR)/% : LIBS = -lstlstd
TestLibrary/$(MODE_DIR)/% : OBJ_DIR := TestLibrary/$(MODE_DIR)

OBJECTS_TestLibrary =

TestLibrary/$(MODE_DIR)/TestLibrary.so : $(OBJECTS_TestLibrary)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";SONAME=`basename ""$@""`;if [ "$(SHAREDLIB_VERSION)" != "" ]; then SONAME=$$SONAME.$(SHAREDLIB_VERSION);fi; $(TOOL_PATH)dplus $(DEBUGFLAGS_Shared-Library-Linker) $(CC_ARCH_SPEC) -Xansi -Xforce-declarations  -Xmake-dependency=0xd -Xpic -Wl, -Xshared -Wl, -Xdynamic -soname="$$SONAME" -o "$@" $(OBJECTS_TestLibrary) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS) -L$(WIND_BASE)/target/usr/lib/ppc/PPC32/common/PIC $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(DO_STRIP)" = "1" ]; then mv -f "$@" "$@.unstripped" && stripppc -g -o "$@" "$@.unstripped";fi && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi && if [ "$(SHAREDLIB_VERSION)" != "" ]; then cp -f "$@" "$@.$(SHAREDLIB_VERSION)";fi

TestLibrary/$(MODE_DIR)/TestLibrary_compile_file : $(FILE) ;

_clean :: TestLibrary/$(MODE_DIR)/TestLibrary_clean

TestLibrary/$(MODE_DIR)/TestLibrary_clean : 
	$(TRACE_FLAG)if [ -d "TestLibrary" ]; then cd "TestLibrary"; rm -rf $(MODE_DIR); fi

force : 



-include $(PRJ_ROOT_DIR)/*.makefile

-include *.makefile

JOBS?=1
ifeq ($(JOBS),1)
main_all : external_build  $(PROJECT_TARGETS)
	@echo "make: built targets of `pwd`"
else
main_all : external_build 
	@$(MAKE) -f $(MAKEFILE) --jobs $(JOBS) $(MFLAGS) $(PROJECT_TARGETS);\
	echo "make: built targets of `pwd`"
endif 

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
