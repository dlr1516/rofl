set(SUBLIB_SUFFIX geometry)
set(SUBLIB_NAME ${PROJECT_NAME}_${SUBLIB_SUFFIX})

message(STATUS "*****")
message(STATUS "PROJECT_NAME: " ${PROJECT_NAME})
message(STATUS "SUBLIB_SUFFIX: " ${SUBLIB_SUFFIX})
message(STATUS "SUBLIB_NAME: " ${SUBLIB_NAME})
message(STATUS "CMAKE_CURRENT_SOURCE_DIR: " ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "CMAKE_INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX})

message(STATUS "Eigen3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})

include_directories(
  "../common/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${SUBLIB_NAME} SHARED
  src/base_geometry2.cpp
  src/box_registration.cpp
  src/correspondence_graph_association.cpp
  src/gis.cpp 
  src/hough_line_detector.cpp 
  src/hough_plane_detector.cpp 
)

if(PCL_FOUND)
  add_executable(test_hough_plane_detector app/test_hough_plane_detector.cpp)
  target_link_libraries(test_hough_plane_detector ${SUBLIB_NAME} rofl_common ${PCL_LIBRARIES})
endif()

add_executable(test_correspondence_graph app/test_correspondence_graph.cpp)
target_link_libraries(test_correspondence_graph ${SUBLIB_NAME} rofl_common ${PCL_LIBRARIES})


# Option "make install": copy binaries
install(TARGETS ${SUBLIB_NAME}
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

## Option "make install": copy headers
file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/${SUBLIB_SUFFIX}/*.h")
install(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/${SUBLIB_SUFFIX}/)

### Option "make install": copy cmake script
#file(GLOB cmake_script "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/roflConfig.cmake")
#message(STATUS "cmake_script " ${cmake_script})
#INSTALL(FILES ${cmake_script} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/rofl/)
