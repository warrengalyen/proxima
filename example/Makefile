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

EXAMPLES := \
	basic   \
	raylib

SOURCES := $(EXAMPLES:%=$(SOURCE_PATH)/%.c)
TARGETS := $(EXAMPLES:%=$(BINARY_PATH)/%)

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
LDLIBS := -lproxima -lraylib -ldl -lGL -lm -lpthread -lrt -lX11

PLATFORM := $(HOST_PLATFORM)

ifeq ($(PLATFORM),WINDOWS)
	TARGETS := $(EXAMPLES:%=$(BINARY_PATH)/%.exe)

	ifneq ($(HOST_PLATFORM),WINDOWS)
		CC := x86_64-w64-mingw32-gcc
	endif

	LDLIBS := -lproxima -lraylib -lopengl32 -lgdi32 -lwinmm -lpthread
else ifeq ($(PLATFORM),WEB)
	TARGETS := $(EXAMPLES:%=$(BINARY_PATH)/%.html)

	CC := emcc
	CFLAGS += -DPLATFORM_$(PLATFORM) -Wno-limited-postlink-optimizations

# https://github.com/emscripten-core/emscripten/blob/main/src/settings.js
	WEBFLAGS := -s FORCE_FILESYSTEM -s INITIAL_MEMORY=67108864 -s USE_GLFW=3
	WEBFLAGS += --preload-file $(RESOURCE_PATH) --shell-file $(RESOURCE_PATH)/html/shell.html
endif

all: pre-build build post-build

pre-build:
	@echo "$(PROJECT_PREFIX) Using: '$(CC)' to build all examples."
    
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