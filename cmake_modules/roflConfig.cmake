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

# Searches include directory /usr/local/include/rofl
find_path(rofl_INCLUDE_DIR rofl ${rofl_PREFIX_DIR}/include)
message(STATUS "    rofl_INCLUDE_DIR ${rofl_INCLUDE_DIR}." )
set(rofl_INCLUDE_DIRS ${rofl_INCLUDE_DIR})
  
# Searches library  in /usr/local/lib
find_path(rofl_LIBRARY_DIR librofl_common.a ${rofl_PREFIX_DIR}/lib)
message(STATUS "    rofl_LIBRARY_DIR ${rofl_LIBRARY_DIR}." )
set(rofl_LIBRARY_DIRS ${rofl_PREFIX_DIR}/lib)

# Sets the names of library components (actually A name and A component)
find_library(rofl_LIBRARY rofl ${rofl_LIBRARY_DIRS})
message(STATUS "    rofl_LIBRARY ${rofl_LIBRARY}." )
set(rofl_LIBRARIES ${rofl_LIBRARY})

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

