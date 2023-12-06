.PHONY: all clean
.SUFFIXES: .c .exe .html .out

_COLOR_BEGIN = \033[1;32m
_COLOR_END = \033[m

PROJECT_NAME = proxima
PROJECT_FULL_NAME = mechanika-design/proxima

PROJECT_PREFIX = ${_COLOR_BEGIN}${PROJECT_FULL_NAME}:${_COLOR_END}

# TODO: Edit these values to match your raylib installation path!
RAYLIB_INCLUDE_PATH ?= .
RAYLIB_LIBRARY_PATH ?= .

INCLUDE_PATH = include
LIBRARY_PATH = ../lib
SOURCE_PATH = src

TARGETS ?= \
	${SOURCE_PATH}/basic.out   \
	${SOURCE_PATH}/bricks.out  \
	${SOURCE_PATH}/raycast.out \
	${SOURCE_PATH}/raylib.out  \
	${SOURCE_PATH}/shoot.out

CC = cc
CFLAGS ?= -D_DEFAULT_SOURCE -g -I${INCLUDE_PATH} -I../${INCLUDE_PATH} -O2 -std=gnu99
LDFLAGS ?= -L${LIBRARY_PATH}
LDLIBS ?= -lproxima -lraylib -ldl -lGL -lm -lpthread -lrt -lX11

all: pre-build build post-build

pre-build:
	@echo -e "${PROJECT_PREFIX} Using: '${CC}' to build all examples."

build: ${TARGETS}

.c.exe .c.out:
	@echo -e "${PROJECT_PREFIX} Compiling: $@ (from $<)"
	@${CC} $< -o $@ ${CFLAGS} ${LDFLAGS} ${LDLIBS}

.c.html:
	@echo -e "${PROJECT_PREFIX} Compiling: $@ (from $<)"
	@${CC} $< -o $@ ${CFLAGS} ${LDFLAGS} ${LDLIBS} ${WEBFLAGS}
	@mv $@ ${BINARY_PATH}
post-build:
	@echo -e "${PROJECT_PREFIX} Build complete."

clean:
	@echo -e "${PROJECT_PREFIX} Cleaning up."
	@rm -f ${SOURCE_PATH}/*.data ${SOURCE_PATH}/*.exe ${SOURCE_PATH}/*.js \
	${SOURCE_PATH}/*.html ${SOURCE_PATH}/*.out ${SOURCE_PATH}/*.wasm