# - Try to find Library rofl
# Once done, this will define
#
#  rofl_FOUND - system has rofl module
#  rofl_INCLUDE_DIRS - the rofl include directories
#  rofl_LIBRARY_DIRS - the rofl library directories
#  rofl_LIBRARIES - link these to use rofl


# Uses  directory to search mrf_segmentation directory!
set(rofl_PREFIX_DIR /usr/local)
message(STATUS "Searching rofl in directory ${rofl_PREFIX_DIR}." )

set(rofl_COMPONENTS common geometry pcl)

# Searches include directory /usr/local/include/rofl
find_path(rofl_INCLUDE_DIR 
  NAMES rofl 
  HINTS ${rofl_PREFIX_DIR}/include)
message(STATUS "    rofl_INCLUDE_DIR ${rofl_INCLUDE_DIR}." )
set(rofl_INCLUDE_DIRS ${rofl_INCLUDE_DIR})
  
# Searches library  in /usr/local/lib
find_path(rofl_LIBRARY_DIR 
  NAMES librofl_common.so 
  HINTS ${rofl_PREFIX_DIR}/lib)
message(STATUS "    rofl_LIBRARY_DIR ${rofl_LIBRARY_DIR}." )
set(rofl_LIBRARY_DIRS ${rofl_PREFIX_DIR}/lib)

# Sets the names of library components (actually A name and A component)
#find_library(rofl_LIBRARY 
#  NAMES rofl rofl_common rofl_geometry rofl_pcl 
#  HINTS ${rofl_PREFIX_DIR}/lib)
#message(STATUS "    rofl_LIBRARY ${rofl_LIBRARY}." )
#set(rofl_LIBRARIES ${rofl_LIBRARY})

foreach(component ${rofl_COMPONENTS}) 
  message(STATUS "    searching component ${component}." )
  find_library(rofl_${component}_LIBRARY 
    NAMES rofl rofl_${component} 
    HINTS ${rofl_PREFIX_DIR}/lib)
  message(STATUS "    rofl_${component}_LIBRARY ${rofl_${component}_LIBRARY}." )
  set(rofl_LIBRARIES ${rofl_LIBRARIES} ${rofl_${component}_LIBRARY})
  #set(boost_LIBRARIES ${boost_LIBRARIES} debug ${boost_LIBRARY_DIR}/libboost_${component}-vc110-mt-gd-1_50.lib)
  #set(boost_LIBRARIES ${boost_LIBRARIES} optimized ${boost_LIBRARY_DIR}/libboost_${component}-vc110-mt-1_50.lib)
endforeach()
message(STATUS "    rofl_LIBRARIES ${rofl_LIBRARIES}." )


if(("${rofl_INCLUDE_DIR}" STREQUAL "rofl_INCLUDE_DIR-NOTFOUND") OR
   ("${rofl_LIBRARY_DIRS}" STREQUAL "rofl_LIBRARY_DIRS-NOTFOUND") OR
   ("${rofl_LIBRARY}" STREQUAL "rofl_LIBRARY-NOTFOUND")
  )
  message(STATUS "Library rofl NOT found")
  unset(rofl_FOUND)
  unset(rofl_INCLUDE_DIR)
  unset(rofl_LIBRARY_DIR)
  unset(rofl_LIBRARY)
  unset(rofl_LIBRARIES)
endif()

mark_as_advanced(rofl_INCLUDE_DIRS rofl_LIBRARY_DIRS rofl_LIBRARIES)

