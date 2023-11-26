<div align="center">

// TODO: ...

[![Version Badge](https://img.shields.io/github/v/release/warrengalyen/proxima?include_prereleases)](https://github.com/warrengalyen/proxima/releases)
[![Codefactor Badge](https://www.codefactor.io/repository/github/warrengalyen/proxima/badge)](https://www.codefactor.io/repository/github/warrengalyen/proxima)
[![Code Size Badge](https://img.shields.io/github/languages/code-size/warrengalyen/proxima?color=brightgreen)](https://github.com/warrengalyen/proxima)
[![License Badge](https://img.shields.io/github/license/warrengalyen/proxima)](https://github.com/warrengalyen/proxima/blob/master/LICENSE)

A lightweight 2D physics engine written in C, for educational purposes.

Documentation &mdash;
[Examples](./examples/src) &mdash;
[Prerequisites](#prerequisites)

</div>

## Features

> *NOTE: This project was made for educational purposes (mainly for me to learn how a physics engine works), and therefore it is not recommended to use this library in production. Consider using other 2D physics engines with better performance such as [Box2D](https://github.com/erincatto/box2d) and [Chipmunk2D](https://github.com/slembcke/Chipmunk2D).*

- Broad-phase collision detection with spatial hashing algorithm
- Narrow-phase collision detection with SAT (Separating Axis Theorem)
- Numerical integration with semi-implicit Euler method
- Projected Gauss-Seidel iterative constraint solver
- Support for basic collision event callbacks
- WebAssembly examples powered by [raylib](https://github.com/raysan5/raylib)

// TODO: ...

## Prerequisites

- GCC version 6.4.0+
- Git version 2.14.0+
- GNU Make version 4.1+ (or BSD Make 20181221+)

### Optional

Make sure you have installed [raylib 4.5.0+](https://github.com/raysan5/raylib/releases/tag/4.5.0) to build all examples.

## Building

<details>
<summary>Compiling for GNU/Linux</summary>

### Ubuntu

```console
sudo apt install build-essential git
git clone https://github.com/warrengalyen/proxima && cd proxima
make
```

</details>

<details>
<summary>Compiling for the Web (WebAssembly)</summary>

<br />

Compiling for the Web requires installation of the [Emscripten SDK](https://emscripten.org/).

### Ubuntu

```console
sudo apt install build-essential git
git clone https://github.com/emscripten-core/emsdk && cd emsdk
./emsdk install latest
./emsdk activate latest
source ./emsdk_env.sh
```

After setting up the environment variables for Emscripten SDK, do:

```console
git clone https://github.com/warrengalyen/proxima && cd proxima
make CC=emcc AR=emar
```

</details>

<details>
<summary>Cross-compiling from GNU/Linux to Windows (WSL2)</summary>

### Ubuntu

```console
sudo apt install build-essential git mingw-w64
git clone https://github.com/warrengalyen/proxima && cd proxima
make -f Makefile.win
```

You may need to recompile raylib for Windows before building this library.

```console
git clone https://github.com/raysan5/raylib && cd raylib/src
make -j`nproc` CC=x86_64-w64-mingw32-gcc AR=x86_64-w64-mingw32-ar OS=Windows_NT
```

After recompiling raylib for Windows, you should be able to build the examples:

```console
cd examples
make -f Makefile.win RAYLIB_PATH=../../raylib
```

</details>

## References

### Introduction

- [Hecker, Chris. “Behind the Screen: Physics, Parts 1-3.” Game Developer Magazine. 1996–1997.](https://www.chrishecker.com/Rigid_Body_Dynamics)
- [Catto, Erin. “How Do Physics Engines Work?” USC GamePipe Laboratory. January, 2019.](https://github.com/erincatto/box2d-lite/blob/master/docs/HowDoPhysicsEnginesWork.pdf)
- [Gaul, Randy. “How to Create a Custom Physics Engine.” Envato Tuts+. April–June, 2013.](https://gamedevelopment.tutsplus.com/series/how-to-create-a-custom-physics-engine--gamedev-12715)

### Collision Detection

- [Bittle, William. “Contact Points Using Clipping.” dyn4j.org. November 17, 2011.](https://dyn4j.org/2011/11/contact-points-using-clipping/)
- [Bittle, William. “SAT (Separating Axis Theorem).” dyn4j.org. January 01, 2010.](https://dyn4j.org/2010/01/sat/)
- [Bostock, Mike. “Sutherland–Hodgman Clipping.” observablehq.com. August 02, 2020.](https://observablehq.com/@mbostock/sutherland-hodgman-clipping)
- [Boyd, Stephen P. “Convex Optimization.”, 46–51. Cambridge University Press. March, 2004.](https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf)
- [Coumans, Erwin. “Collision Detection – Contact Generation and GPU Acceleration.” ACM SIGGRAPH ASIA 2010. July 26, 2010.](https://sgvr.kaist.ac.kr/~sungeui/Collision_tutorial/Erwin.pdf)
- [E. J. Hastings, J. Mesit, and R. K. Guha. “Optimization of Large-Scale, Real-Time Simulations by Spatial Hashing. Proc. 2005 Summer Computer Simulation Conference, Cherry Hill, NJ, USA. July 24–28, 2005.](https://scholar.google.com/citations?view_op=view_citation&hl=en&user=u_GkP-EAAAAJ&citation_for_view=u_GkP-EAAAAJ:UeHWp8X0CEIC)
- [MacDonald, Tristam. “Spatial Hashing.” GameDev.net. October 01, 2009.](https://www.gamedev.net/tutorials/programming/general-and-gameplay-programming/spatial-hashing-r2697/)
- [Teschner, Matthias, Bruno Heidelberger, Matthias Müller, Danat Pomeranets, and Markus Gross. n.d. “Optimized Spatial Hashing for Collision Detection of Deformable Objects.” Computer Graphics Laboratory, ETH Zurich. January 2005.](https://matthias-research.github.io/pages/publications/tetraederCollision.pdf)
- [Van den Bergen, Gino. “Physics for Game Programmers: Spatial Data Structures.” Game Developers Conference 2013. March, 2013.](https://storage.googleapis.com/google-code-archive-downloads/v2/code.google.com/box2d/GDC13_vandenBergen_Gino_Physics_Tut.pdf)

### Constraint Solver

- [Chou, Ming-Lun. “Game Physics: Resolution – Contact Constraints.” allenchou.net. December 31, 2013.](https://allenchou.net/2013/12/game-physics-resolution-contact-constraints/)
- [Chou, Ming-Lun. “Game Physics: Stability – Warm Starting.” allenchou.net. January 04, 2014.](http://allenchou.net/2014/01/game-physics-stability-warm-starting/)
- [Catto, Erin. “Fast and Simple Physics Using Sequential Impulses.” San Jose, CA. March, 2006.](https://box2d.org/files/ErinCatto_SequentialImpulses_GDC2006.pdf)
- [Catto, Erin. “Iterative Dynamics with Temporal Coherence.” Crystal Dynamics, Menlo Park, CA. June, 2005.](https://box2d.org/files/ErinCatto_IterativeDynamics_GDC2005.pdf)
- [Strunk, Oliver. “Stop my Constraints from Blowing Up!” Game Developers Conference 2013. March, 2013.](https://storage.googleapis.com/google-code-archive-downloads/v2/code.google.com/box2d/Strunk_Oliver_Stop_My_Constraints_From_Blowing_Up.pdf)
- [Tonge, Richard. “Iterative Rigid Body Solvers.” Game Developers Conference 2013. March, 2013.](https://archive.org/details/GDC2013Tonge)

### Numerical Methods

- [Catto, Erin. “Numerical Methods.” San Jose, CA. March, 2015.](https://box2d.org/files/ErinCatto_NumericalMethods_GDC2015.pdf)
- [Fiedler, Glenn. “Integration Basics.” Gaffer on Games. June 01, 2004.](https://gafferongames.com/post/integration_basics/)
- [Witkin, Andrew, and David Baraff. n.d. “An Introduction to Physically Based Modeling: Differential Equation Basics.” ACM SIGGRAPH 1995. August, 1995.](http://www.cs.cmu.edu/~baraff/sigcourse/index.html)

### Raycast Queries

- [Rees, Gareth. “How do you detect where two line segments intersect?” stackoverflow.com. February, 2009.](https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/565282#565282)
- [Scratchapixel. “A Minimal Ray-Tracer: Rendering Simple Shapes (Sphere, Cube, Disk, Plane, etc.).” scratchapixel.com. November, 2022.](https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection.html)

## License

MIT License
