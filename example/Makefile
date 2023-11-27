.PHONY: all clean
.SUFFIXES: .c .out

_COLOR_BEGIN != tput setaf 10
_COLOR_END != tput sgr0

PROJECT_NAME = proxima
PROJECT_FULL_NAME = mechanika-design/proxima

PROJECT_PREFIX = ${_COLOR_BEGIN}${PROJECT_FULL_NAME}:${_COLOR_END}

INCLUDE_PATH = include
LIBRARY_PATH = ../lib
SOURCE_PATH = src

TARGETS = \
	${SOURCE_PATH}/basic.out   \
	${SOURCE_PATH}/bricks.out  \
	${SOURCE_PATH}/raycast.out \
	${SOURCE_PATH}/raylib.out

CC = gcc
CFLAGS = -D_DEFAULT_SOURCE -g -I${INCLUDE_PATH} -I../${INCLUDE_PATH} -O2 -std=gnu99
LDFLAGS = -L${LIBRARY_PATH}
LDLIBS = -lproxima -lraylib -ldl -lGL -lm -lpthread -lrt -lX11

all: pre-build build post-build

pre-build:
	@echo "${PROJECT_PREFIX} Using: '${CC}' to build all examples."

build: ${TARGETS}

.c.out:
	@echo "${PROJECT_PREFIX} Compiling: $@ (from $<)"
	@${CC} $< -o $@ ${CFLAGS} ${LDFLAGS} ${LDLIBS} ${WEBFLAGS}
	@mv $@ ${BINARY_PATH}
post-build:
	@echo "${PROJECT_PREFIX} Build complete."

clean:
	@echo "${PROJECT_PREFIX} Cleaning up."
	@rm -f ${SOURCE_PATH}/*.data ${SOURCE_PATH}/*.exe ${SOURCE_PATH}/*.js \
		   ${SOURCE_PATH}/*.html ${SOURCE_PATH}/*.out ${SOURCE_PATH}/*.wasm