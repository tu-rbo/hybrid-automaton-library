# hybrid-automaton-library
Library for creating feedback-based motion descriptions for robots.

## Installation

### Required 3rd party dependencies
* Install boost (>= 1.54.0) e.g. libboost-dev
* Install [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) (>= ?) e.g. libeigen3-dev
* Install tinyxml  (for Hybrid Automaton) e.g. libtinyxml2-dev

### Recommended 3rd party dependencies
* Install doxygen
* Install graphviz
* Install [log4cpp](http://log4cpp.sourceforge.net/) and compile into static library (on windows you will need Multi-threaded (Debug) Dll setting) e.g liblog4cpp5-dev

### Installation on Linux
   * Build gmock and gtest
      ```bash
      cd $(GMOCK_ROOT)/gmock-1.7.0
      mkdir build && cd build
      cmake ..   #use  cmake .. -DCMAKE_BUILD_TYPE=DEBUG for a configuring a debug build
      make
      make install
      ```

   * Build hybrid_automaton
      ```bash
      cd $(hybrid_automaton_library_ROOT)
      mkdir build && cd build
      cmake ..    #-DUNIT_TESTS=ON  for compiling the unit tests
      make
      make doc
      sudo make install   # sudo because it's copied to /usr/local/lib
      ```

### Installation on Windows

 * Set $BOOST_DIR as environmental variable to the installation directory (s.t. boost/xxx.hpp will include headers)
 * Set $EIGEN_DIR as environmental variable to the installation directory (s.t. Eigen/xxx.hpp will include headers)
 * Set $LOG4CPP_INCLUDE_DIR and $LOG4CPP_LIB_DIR as environmental variables to the installation directory if you want to use log4cpp for logging
 * Build gmock and gtest
 * Build hybrid_automaton
     * Go to $(hybrid_automaton_library_ROOT)/
     * Create a folder "build" (or whatever you want)
     * Open CMake GUI in Admin mode
         * Where is the source code: $(hybrid_automaton_library_ROOT)/
         * Where to build the binaries: $(hybrid_automaton_library_ROOT)/build
     * Click Configure (select Visual Studio 9 2008 - Native compilers)
     * If you want to generate the unit tests look for the entry "UNIT_TESTS" and activate it. Then press "Confgure" again
     * Click Generate
     * Open VS 2008 admin mode, open solution $(hybrid_automaton_library_ROOT)/build/hybrid_automaton.sln
     * Build
