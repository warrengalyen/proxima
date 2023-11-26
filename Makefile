.PHONY: all clean

_COLOR_BEGIN != tput setaf 10
_COLOR_END != tput sgr0

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

CC = gcc
AR = ar
CFLAGS = -D_DEFAULT_SOURCE -g -I${INCLUDE_PATH} -O2 -std=gnu99
LDLIBS = -lm

all: pre-build build post-build

pre-build:
	@echo "${PROJECT_PREFIX} Using: '${CC}' and '${AR}' to build this project."

build: ${TARGETS}

.c.o:
	@echo "${PROJECT_PREFIX} Compiling: $@ (from $<)"
	@${CC} -c $< -o $@ ${CFLAGS} ${LDFLAGS} ${LDLIBS}

${TARGETS}: ${OBJECTS}
	@mkdir -p ${LIBRARY_PATH}
	@echo "${PROJECT_PREFIX} Linking: ${TARGETS}"
	@${AR} rcs ${TARGETS} ${OBJECTS}

post-build:
	@echo "${PROJECT_PREFIX} Build complete."

clean:
	@echo "${PROJECT_PREFIX} Cleaning up."
	@rm -f ${LIBRARY_PATH}/*.a
	@rm -f ${SOURCE_PATH}/*.o