# Galileo-API

Repository to share Galileo's user code.

Website: [www.galileo-fea.com](https://www.galileo-fea.com)

Contact: [mvbentosantana@gmail.com](mailto:mvbentosantana@gmail.com)

Author: Murillo Vinicius Bento Santana

## Setup

### Setup in Windows

The MSVC C++ compiler is used for the build. Be sure to have it [installed](https://visualstudio.microsoft.com/vs/community/) and [available](https://learn.microsoft.com/en-us/visualstudio/ide/reference/command-prompt-powershell?view=vs-2022) in the path. All used external libraries are packed with the code.

Generate an executable (make.exe) that will be used to build the code: `win\make.bat`

### Setup in Unix

The [GNU C++](https://gcc.gnu.org/) compiler and the [GNU Make](https://www.gnu.org/software/make/) tool are used for the build. The [GNU GDB](https://www.sourceware.org/gdb/) debugger is used to debug the code. Be sure to have them installed by running the following commands:

`sudo apt-get install g++`

`sudo apt-get install gdb`

`sudo apt-get install make`

The following libraries are also required for the build:

* [GMSH](https://gmsh.info/) for mesh generation: `sudo apt-get install libgmsh-dev`

* [Quadrule](https://people.math.sc.edu/Burkardt/cpp_src/quadrule/quadrule.html) for numerical integration: `sudo apt-get install libquadrule-dev`

* [Suite-sparse](https://people.engr.tamu.edu/davis/suitesparse.html) for sparse matrix decomposition: `sudo apt-get install libsuitesparse-dev`

## Build and Run

Build the debug version of the code: `make`

Build the release version of the code: `make m=r`

Run the debug version of the code: `make run`

Run the release version of the code: `make run m=r`

Clean the debug version of the code: `make clean`

Clean the release version of the code: `make clean m=r`

## Debug

### Debug in Windows

Debug the code: `make debug`

### Debug in Unix

Debug the code: `gdb ./dist/debug/ben.out`
