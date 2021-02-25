# Path Planning Project

> This repository contains the work for the Path Planning project in the Self-Driving Car Engineer Nanodegree Program of Udacity.

The required writeup can be read from [WRITEUP](./WRITEUP.md).

## Prerequisites

- [Eigen](https://eigen.tuxfamily.org/dox-devel/index.html) as 'production' dependency

Eigen was added to the project by using `git submodule add https://gitlab.com/libeigen/eigen.git extern/eigen` and then adding required config to `./CMakeLists.txt`. See running section for making it available to build step.

The adding of [submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules) after cloning is typically done once with `git submodule update --init` ran from the project root directory.

## Project structure

The project aims to follow a [Modern CMake](https://cliutils.gitlab.io/modern-cmake/) setup. It has several targets, an app target containing the complete application (`./app`), a Planner library (`./src` and `./include`) and the tests (`./test`). Each part has its own `CMakeList.txt` file which come together in the main `CMakeList.txt` found in the root of the project.

```bash
```

## Running the Code

This project involves the Term 3 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)
Then with the running simulator this PID controller can be build and run with

```bash
./build.sh
./run.sh
```

## Testing the code

Test support is added to the project. Test are not extensive yet.
To run them:

```bash
./build.sh
./test.sh
```

## Extra

### CppCheck

With [CppCheck](http://cppcheck.sourceforge.net/manual.pdf) one can improve code quality by checking common errors.

Installation and dependencies

```bash
brew install cppcheck
pip install pygments # only once if complaining about missing pygments...
```

Now reports can be generated by running the `report.sh` script

```bash
./report.sh
```

### Doxygen

With [Doxygen]() and [Graphviz]() installed, documentation can be written in Doxygen/Javadoc style and an html version generated.

```bash
brew install doxygen
brew install graphviz
```

Running the supplied `./docs.sh` script will generate html documentations that can be viewed (opened by script) in the browser.

```bash
```

## Resources

- [Interesting read about C++ tooling](https://invisible-island.net/personal/lint-tools.html#background)
- [Debug Macros](https://stackoverflow.com/questions/14251038/debug-macros-in-c)
- [Modern CMake](https://cliutils.gitlab.io/modern-cmake/)