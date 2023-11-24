.PHONY: all clean

_COLOR_BEGIN := $(shell tput setaf 10)
_COLOR_END := $(shell tput sgr0)

PROJECT_NAME := proxima
PROJECT_FULL_NAME := mechanika-design/proxima

PROJECT_PREFIX := $(_COLOR_BEGIN)$(PROJECT_FULL_NAME):$(_COLOR_END)

INCLUDE_PATH := include
LIBRARY_PATH := lib
SOURCE_PATH := src

SOURCES := \
	$(SOURCE_PATH)/broad-phase.c  \
	$(SOURCE_PATH)/collision.c    \
	$(SOURCE_PATH)/geometry.c     \
	$(SOURCE_PATH)/rigid-body.c   \
	$(SOURCE_PATH)/timer.c        \
	$(SOURCE_PATH)/world.c

OBJECTS := $(SOURCES:.c=.o)
TARGETS := $(LIBRARY_PATH)/lib$(PROJECT_NAME).a

HOST_PLATFORM := UNKNOWN

ifeq ($(OS),Windows_NT)
	ifeq "$(findstring ;,$(PATH))" ";"
		PROJECT_PREFIX := $(PROJECT_FULL_NAME):
	endif

# MINGW-W64 or MSYS2...?
	HOST_PLATFORM := WINDOWS
else
	UNAME = $(shell uname 2>/dev/null)

	ifeq ($(UNAME),Linux)
		HOST_PLATFORM = LINUX
	endif
endif

CC := gcc
AR := ar
CFLAGS := -D_DEFAULT_SOURCE -g $(INCLUDE_PATH:%=-I%) -O2 -std=gnu99
LDFLAGS := -lm

PLATFORM := $(HOST_PLATFORM)

ifeq ($(PLATFORM),WINDOWS)
	ifneq ($(HOST_PLATFORM),WINDOWS)
		CC := x86_64-w64-mingw32-gcc
		AR := x86_64-w64-mingw32-ar
	endif
else ifeq ($(PLATFORM),WEB)
	CC := emcc
	AR := emar

#	CFLAGS += -Wno-return-type -Wno-unused-command-line-argument
endif

all: pre-build build post-build

pre-build:
	@echo "$(PROJECT_PREFIX) Using: '$(CC)' and '$(AR)' to build this project."
    
build: $(TARGETS)

$(SOURCE_PATH)/%.o: $(SOURCE_PATH)/%.c
	@echo "$(PROJECT_PREFIX) Compiling: $@ (from $<)"
	@$(CC) -c $< -o $@ $(CFLAGS) $(LDFLAGS) $(LDLIBS)
    
$(TARGETS): $(OBJECTS)
	@mkdir -p $(LIBRARY_PATH)
	@echo "$(PROJECT_PREFIX) Linking: $(TARGETS)"
	@$(AR) rcs $(TARGETS) $(OBJECTS)

post-build:
	@echo "$(PROJECT_PREFIX) Build complete."

clean:
	@echo "$(PROJECT_PREFIX) Cleaning up."
	@rm -rf $(LIBRARY_PATH)/*.a
	@rm -rf $(SOURCE_PATH)/*.o