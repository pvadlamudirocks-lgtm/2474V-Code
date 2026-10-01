################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=-Wno-deprecated-enum-enum-conversion

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
EXCLUDE_COLD_LIBRARIES:= 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS!
LIBNAME:=libbest
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# Mechanisms compile but require explicit initialization and commissioning gates.
# Keep unfinished distance correction excluded from the odometry test program.
EXCLUDE_SRCDIRS+=$(SRCDIR)/distance_correction.cpp

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the the include directory get exported
TEMPLATE_FILES=$(INCDIR)/**/*.h $(INCDIR)/**/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk

# Observe native exit results without replacing the PID or wait implementations.
$(HOT_ELF): private LDFLAGS += -Wl,--wrap=_ZN2ez3PID14exit_conditionESt6vectorIN4pros2v55MotorESaIS4_EEb -Wl,--wrap=_ZN2ez5Drive8pid_waitEv

# Link the original EZ wait machine code into the hot image so its
# exit-check calls can be observed. The rest of EZ remains in the cold image.
$(BINDIR)/exit_conditions.cpp.o: $(FWDIR)/EZ-Template.a Makefile
	$(AR) x --output=$(BINDIR) $< exit_conditions.cpp.o
	$(OBJCOPY) --redefine-sym _ZN2ez5Drive8pid_waitEv=ez_trace_native_pid_wait $@
$(HOT_ELF): $(BINDIR)/exit_conditions.cpp.o

# EZ v3.2.2 PID.cpp, with only the integral sign-reset comparison corrected.
# Replace the archive member so cold-image controllers also use the correction.
# Source: https://github.com/EZ-Robotics/EZ-Template/blob/v3.2.2/src/EZ-Template/PID.cpp
$(FWDIR)/EZ-Template.a: patches/EZ-Template/PID.cpp $(INCDIR)/EZ-Template/PID.hpp patches/EZ-Template/pid_tasks.cpp $(INCDIR)/EZ-Template/path_trace_hook.hpp patches/EZ-Template/exit_conditions.cpp $(INCDIR)/EZ-Template/navigation_hooks.hpp Makefile
	mkdir -p $(BINDIR)/ez_pid_patch
	$(CXX) -c $(INCLUDE) $(CXXFLAGS) $(EXTRA_CXXFLAGS) -o $(BINDIR)/ez_pid_patch/PID.cpp.o $<
	$(AR) r $@ $(BINDIR)/ez_pid_patch/PID.cpp.o
	$(CXX) -c $(INCLUDE) $(CXXFLAGS) $(EXTRA_CXXFLAGS) -o $(BINDIR)/ez_pid_patch/pid_tasks.cpp.o patches/EZ-Template/pid_tasks.cpp
	$(AR) r $@ $(BINDIR)/ez_pid_patch/pid_tasks.cpp.o

	$(CXX) -c $(INCLUDE) $(CXXFLAGS) $(EXTRA_CXXFLAGS) -o $(BINDIR)/ez_pid_patch/exit_conditions.cpp.o patches/EZ-Template/exit_conditions.cpp
	$(AR) r $@ $(BINDIR)/ez_pid_patch/exit_conditions.cpp.o
