# Path Planning Project

> This repository contains the work for the Path Planning project in the Self-Driving Car Engineer Nanodegree Program of Udacity.

The required Model Documentation can be read from [ModelDocumentation](./ModelDocumentation.md).

## Build and Run code

0. Clone this repo with git
```bash
git clone git@github.com:joustava/CarND-Path-Planning-Project.git
```
1. Make sure you meet the dependency requirement [instructions](https://github.com/udacity/CarND-Path-Planning-Project#dependencies).

2. This project involves the Term 3 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2). Once unzipped you might need to change permissions of the simulator executable by running: `chmod u+x term3_sim_mac/term3_sim.app/Contents/MacOS/term3_sim_mac` from the folder you've extracted it.

3. **IMPORTANT:** Initialise git submodules by running (from project root).

```bash
git submodule update --init # only once
```
This will add [Eigen](https://eigen.tuxfamily.org/dox-devel/index.html) to the project via a git submodule as 'production' dependency.

4. Then with the running simulator this project can be build and run with the following scripts

```bash
./build.sh
./run.sh
```

NOTE: This project was **not** build to support Docker.

## Testing the code

Test support is added to the project. Test are not extensive yet.
To run them:

```bash
./build.sh
./test.sh
```

## Project structure

The project aims to follow a [Modern CMake](https://cliutils.gitlab.io/modern-cmake/) setup. It has several targets, an app target containing the complete application (`./app`), a Planner library (`./src` and `./include`) and the tests (`./test`). Each part has its own `CMakeList.txt` file which come together in the main `CMakeList.txt` found in the root of the project.

## Extra (not needed for building or running)

### CppCheck

With [CppCheck](http://cppcheck.sourceforge.net/manual.pdf) one can improve code quality by checking common errors.

Installation and dependencies

```bash
brew install cppcheck
pip install pygments # run once if complaining about missing pygments...
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


### Eigen as submodule

Eigen was added to the project by using `git submodule add https://gitlab.com/libeigen/eigen.git extern/eigen` and then adding required config to `./CMakeLists.txt`. See running section for making it available to build step.

## Resources

- [Interesting read about C++ tooling](https://invisible-island.net/personal/lint-tools.html#background)
- [Debug Macros](https://stackoverflow.com/questions/14251038/debug-macros-in-c)
- [Modern CMake](https://cliutils.gitlab.io/modern-cmake/)
- [spline interpolation](https://en.wikipedia.org/wiki/Spline_interpolation)
- [spline library](https://github.com/ttk592/spline/)