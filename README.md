# rofl
RIMLab Open Factotum Library

#### Copyright (C) 2021-2024 Dario Lodi Rizzini, Ernesto Fontana.


OVERVIEW
-------------------------------------------------

Library **rofl** implements the Angular Radon Spectrum method 
for estimation of rotation. 
It has been kept to a minimal design. 

If you use this library, please cite the following paper: 

D. Lodi Rizzini. 
Angular Radon Spectrum for Rotation Estimation. 
Pattern Recognition, Volume 84, Dec. 2018, Pages 182-196, 
DOI [10.1016/j.patcog.2018.07.017](https://doi.org/10.1016/j.patcog.2018.07.017).

````
  @article{lodirizzini2018pr,
    author={Lodi Rizzini, D.},
    title={{Angular Radon Spectrum for Rotation Estimation}}
    journal={Pattern Recognition},
    volume={84},
    pages={182--196},
    month={dec},
    year={2018},
    publisher={Elsevier},
    issn = {},
    doi = {10.1016/j.patcog.2018.07.017},
    note = {DOI 10.1016/j.patcog.2018.07.017, EID 2-s2.0-85050072081},
  }
````

or the most relevant associated publications by visiting: 
https://rimlab.ce.unipr.it/


DEPENDENCIES
-------------------------------------------------

The software depends on the following external libraries

- Boost (submodule lexical_cast)
- Eigen 3.0 
- ROFL (https://github.com/dlr1516/rofl)

Other dependencies are placed in directory thirdparty. 
Some examples require the external application "gnuplot" to display 
results. 


HOW TO COMPILE
-------------------------------------------------

Let ${rofl_ROOT} be the install directory of your local copy 
of library rofl. 
The following standard commands are required to compile it:

-  cd ${rofl_ROOT}
-  mkdir build
-  cd build
-  cmake ..
-  make

You can also install the library into a system directory. 
To change the install directory you must set cmake environment
variable ${CMAKE_INSTALL_PREFIX} (e.g. using command "ccmake .."
before calling "cmake .."). 
Its default value on UNIX-like/Linux systems is "/usr/local".
After compiling library rofl, run the command:

-  sudo make install

The command "sudo" is required only if ${CMAKE_INSTALL_PREFIX} 
is a system diretory managed by administrator user root.
Such command copies:

1. header files of
> \${rofl_ROOT}/include/rofl

&ensp; to 
  
> \${CMAKE_INSTALL_PREFIX}/include/rofl/ 

2. library files
> \${rofl_ROOT}/lib/librofl.a

&ensp; to 

> \${CMAKE_INSTALL_PREFIX}/lib/

3. cmake script
> \${rofl_ROOT}/cmake_modules/roflConfig.cmake

&ensp; to 

> \${CMAKE_INSTALL_PREFIX}/share/rofl/


HOW TO USE LIBRARY rofl IN YOUR PROJECT
-------------------------------------------------

If library rofl has been installed in system directory "/usr/local",
then it is straighforward to use it in your projects.
You need to add the following lines to your project as in this example:


> CMAKE_MINIMUM_REQUIRED(VERSION 2.8)  
> PROJECT(foobar)  
> 
> find_package(rofl REQUIRED)  
> message(STATUS "rofl_FOUND ${rofl_FOUND}")  
> message(STATUS "rofl_INCLUDE_DIRS ${rofl_INCLUDE_DIRS}")  
> message(STATUS "rofl_LIBRARY_DIRS ${rofl_LIBRARY_DIRS}")  
> message(STATUS "rofl_LIBRARIES ${rofl_LIBRARIES}")  
>  
> if(${rofl_FOUND})   
>   include_directories(${rofl_INCLUDE_DIRS})  
>   link_directories(${rofl_LIBRARY_DIRS})  
> endif()  
> 
> add_executable(foobar foobar.cpp)  
> target_link_libraries(foobar ${rofl_LIBRARIES})  

The above example uses the variables defined in roflConfig.cmake:

-  rofl_FOUND - system has rofl module
-  rofl_INCLUDE_DIRS - the rofl include directories
-  rofl_LIBRARY_DIRS - the rofl library directories
-  rofl_LIBRARIES - link these to use rofl
