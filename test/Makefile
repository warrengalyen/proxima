.PHONY: all clean

_COLOR_BEGIN := $(shell tput setaf 2)
_COLOR_END := $(shell tput sgr0)

# TODO: Edit this value to match your raylib installation path
RAYLIB_PATH ?= C:/raylib/raylib

PROJECT_NAME := proxima
PROJECT_FULL_NAME := mechanika-design/proxima

PROJECT_PREFIX := $(_COLOR_BEGIN)$(PROJECT_FULL_NAME):$(_COLOR_END)

BINARY_PATH := bin

INCLUDE_PATH := \
	../include  \
	./include

LIBRARY_PATH := ../lib

INCLUDE_PATH += $(RAYLIB_PATH)/src
LIBRARY_PATH += $(RAYLIB_PATH)/src

RESOURCE_PATH := res
SOURCE_PATH := src

TESTS := \
	collision

SOURCES := $(TESTS:%=$(SOURCE_PATH)/%.c)
TARGETS := $(TESTS:%=$(BINARY_PATH)/%)

HOST_PLATFORM := LINUX

ifeq ($(OS),Windows_NT)
	PROJECT_PREFIX := $(PROJECT_NAME):

	HOST_PLATFORM := WINDOWS
else
	UNAME = $(shell uname)

	ifeq ($(UNAME),Linux)
		HOST_PLATFORM = LINUX
	endif
endif

CC := gcc
CFLAGS := -D_DEFAULT_SOURCE -g $(INCLUDE_PATH:%=-I%) -O2
LDFLAGS := $(LIBRARY_PATH:%=-L%)
LDLIBS := -lproxima -lm

PLATFORM := $(HOST_PLATFORM)

ifeq ($(PLATFORM),WINDOWS)
	TARGETS := $(TESTS:%=$(BINARY_PATH)/%.exe)

	ifneq ($(HOST_PLATFORM),WINDOWS)
		CC := x86_64-w64-mingw32-gcc
	endif
endif

all: pre-build build post-build

pre-build:
	@echo "$(PROJECT_PREFIX) Using: '$(CC)' to build all tests."
    
build: $(TARGETS)

$(BINARY_PATH)/%: $(SOURCE_PATH)/%.c
	@mkdir -p $(BINARY_PATH)
	@echo "$(PROJECT_PREFIX) Compiling: $@ (from $<)"
	@$(CC) $< -o $@ $(CFLAGS) $(LDFLAGS) $(LDLIBS) $(WEBFLAGS)
    
$(BINARY_PATH)/%.exe: $(SOURCE_PATH)/%.c
	@mkdir -p $(BINARY_PATH)
	@echo "$(PROJECT_PREFIX) Compiling: $@ (from $<)"
	@$(CC) $< -o $@ $(CFLAGS) $(LDFLAGS) $(LDLIBS) $(WEBFLAGS)

$(BINARY_PATH)/%.html: $(SOURCE_PATH)/%.c
	@mkdir -p $(BINARY_PATH)
	@echo "$(PROJECT_PREFIX) Compiling: $@ (from $<)"
	@$(CC) $< -o $@ $(CFLAGS) $(LDFLAGS) $(LDLIBS) $(WEBFLAGS)
    
post-build:
	@echo "$(PROJECT_PREFIX) Build complete."

clean:
	@echo "$(PROJECT_PREFIX) Cleaning up."
	@rm -rf $(BINARY_PATH)/*