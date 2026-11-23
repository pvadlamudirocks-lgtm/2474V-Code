# Run from project root: pros make -- -f diagnostics/lift_conflict/Makefile
ROOT=.
FWDIR=./firmware
BINDIR=./bin/lift_conflict
SRCDIR=./diagnostics/lift_conflict
INCDIR=./include
override DEPDIR=./.d/lift_conflict
CEXTS=c
ASMEXTS=s S
CXXEXTS=cpp c++ cc
USE_PACKAGE=0
IS_LIBRARY=0
LIBNAME=lift_conflict
VERSION=1.0.0
EXTRA_CXXFLAGS=-Wno-deprecated-enum-enum-conversion
.DEFAULT_GOAL=quick
EXCLUDE_SRCDIRS+=$(SRCDIR)/brain_project
include common.mk
$(MONOLITH_ELF): private LDFLAGS += -Wl,-Map=$(BINDIR)/lift_conflict.map,--cref

$(BINDIR):
	mkdir -p $@
