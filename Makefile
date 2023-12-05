.PHONY: all clean

_COLOR_BEGIN = \033[1;32m
_COLOR_END = \033[m

PROJECT_NAME = proxima
PROJECT_FULL_NAME = mechanika-design/proxima

PROJECT_PREFIX = ${_COLOR_BEGIN}${PROJECT_FULL_NAME}:${_COLOR_END}

INCLUDE_PATH = include
LIBRARY_PATH = lib
SOURCE_PATH = src

OBJECTS = \
	${SOURCE_PATH}/broad-phase.o  \
	${SOURCE_PATH}/collision.o    \
	${SOURCE_PATH}/geometry.o     \
	${SOURCE_PATH}/rigid-body.o   \
	${SOURCE_PATH}/timer.o        \
	${SOURCE_PATH}/world.o

TARGETS = ${LIBRARY_PATH}/lib${PROJECT_NAME}.a

CC = cc
AR = ar
CFLAGS ?= -D_DEFAULT_SOURCE -g -I${INCLUDE_PATH} -O2 -std=gnu99

# CFLAGS += -Wall -Wpedantic
CFLAGS += -Wno-unused-command-line-argument  \
	-Wno-unused-but-set-variable             \
	-Wno-unused-value                        \
	-Wno-unused-variable

all: pre-build build post-build

pre-build:
	@printf "${PROJECT_PREFIX} Using: '${CC}' and '${AR}' to build this project.\n"

build: ${TARGETS}

.c.o:
	@printf "${PROJECT_PREFIX} Compiling: $@ (from $<)\n"
	@${CC} -c $< -o $@ ${CFLAGS} ${LDFLAGS} ${LDLIBS}

${TARGETS}: ${OBJECTS}
	@mkdir -p ${LIBRARY_PATH}
	@printf "${PROJECT_PREFIX} Linking: ${TARGETS}\n"
	@${AR} rcs ${TARGETS} ${OBJECTS}

post-build:
	@printf "${PROJECT_PREFIX} Build complete.\n"

clean:
	@printf "${PROJECT_PREFIX} Cleaning up.\n"
	@rm -f ${LIBRARY_PATH}/*.a
	@rm -f ${SOURCE_PATH}/*.o