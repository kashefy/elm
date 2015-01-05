Getting your SEM build up and running.

A C++ implementation of Spike-based Expectation Maximization (SEM), a biologically inspired model, for object detection and recognition.

# Background #

Nessler, Pfeiffer and Maass originally propose the SEM model in 2010. They demonstrate how a spiking neuronal network governed by spike-timing-dependent-plasticity (STDP) and a stochastic winner-take-all (WTA) can learn and predict hidden causes through unsupervised learning from visual input.

This implementation is work by Youssef Kashef. It extends the original SEM with hierarchical learning of features and selective visual attention. This work was as part of a Masters Thesis at ETH Zürich titled "Scale- and Translation-Invariant Unsupervised Learning of Hidden Causes Using Spiking Neurons with Selective Visual Attention"

## Milestones ##

1. Matlab implementation by Michael Pfeiffer and authors of 2010 SEM paper (not included).
2. Java implementation + Evaluation in Matlab
3. SEM model extended with scale and translation invariance
4. C++ implementation and code refactoring **in progress**

### How do I get set up? ###

The build is IDE agnostic and platform independent. After setting up all required dependencies, you configure the build for your toolchain/IDE of choice and build it.
Required tools:

* C++ build tools (e.g. gcc, clang,...)
* CMake
  
Dependencies:
  
You'll need to set up the following dependencies.You can build them from source or install them via your platforms package management software (e.g. Ubuntu's apt-get, Fedora's yum)
  
* OpenCV, version 2.4.x or later
* Google Test framework (gtest), version 1.7 or later
* Boost Libraries, version 1.54.0 or later (components used: system, filesystem)
  
  
Configuration:
  
We use cmake for configuring the build. This allows building for a variety of toolchains and IDEs and keeping the code independent of such.
  
Example cmake configuration command if dependencies are install in the usual system paths (e.g. /usr/lib):
cmake $SRC_DIR/sem
  
Example cmake configuration command for when dependencies are in custom paths, or if you want to link against dependencies other than those installed under the usual system paths:
  
    cmake -DGTEST_ROOT=<gtest build dir> -DOpenCV_DIR=<opencv build dir> -DBOOST_ROOT=<boost build dir> -DBoost_NO_SYSTEM_PATHS=ON $SRC_DIR/sem
  
&nbsp;&nbsp;&nbsp;&nbsp;cmake -DGTEST_ROOT=~/src/gtest-1.7.0 -DOpenCV_DIR=~/build/opencv/share/ -DBOOST_ROOT=~/build/boost/boost_1_57_0/ -DBoost_NO_SYSTEM_PATHS=ON $SRC_DIR/sem

* How to run tests
Build the run_unittests target and running the resulting executable binary runs the tests. Running the binary in a terminal displays test results.

* Deployment instructions
The installation step produces all artifacts to link against this framework.

### Build issues ###

Linking error when building in Ubuntu:
The error message looks something like
...undefined reference to symbol 'pthread_key_delete@@GLIBC_2.2.5'...
...'pthread_key_delete@@GLIBC_2.2.5' is defined in DSO /lib/x86_64-linux-gnu/libpthread.so.0 so try adding it to the linker command line...

Resolve by adding -pthread or -lpthread to the linker. You can do so by reconifuring CMake and adding the option:
-DCMAKE_EXE_LINKER_FLAGS="-pthread"

### Contribution guidelines ###

* Writing tests: Google Test framework used for writing C++ unit tests. Please see source files under sub directories **/test/ for examples. modules/ts contains utilities used for testing (e.g. custom assertions)
* Code review: Create a pull request to start a review. Describe the main contribution of this pull request. The changes in a pull request should be self contained and not pending approval of other pull requests. Purpose of changes should be concise. Pull requests with code styling changes are permitted, 
* Other guidelines: [Google's C++ Code Style guidelines](http://google-styleguide.googlecode.com/svn/trunk/cppguide.html)

### How do I generate the documentation? ###

cd <source.dir>
doxygen ./docs/doxygenConf*

### Who do I talk to? ###

* alias "kashefy"