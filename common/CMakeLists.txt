set(SUBLIB_SUFFIX common)
set(SUBLIB_NAME ${PROJECT_NAME}_${SUBLIB_SUFFIX})

message(STATUS "*****")
message(STATUS "PROJECT_NAME: " ${PROJECT_NAME})
message(STATUS "SUBLIB_SUFFIX: " ${SUBLIB_SUFFIX})
message(STATUS "SUBLIB_NAME: " ${SUBLIB_NAME})
message(STATUS "CMAKE_CURRENT_SOURCE_DIR: " ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "CMAKE_INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX})

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(${SUBLIB_NAME} SHARED
  src/bit_manip.cpp 
  src/disjoint_set.cpp
  src/functions.cpp
  src/io.cpp
  src/label_set.cpp
  src/morton_tree.cpp
  src/param_map.cpp
  src/profiler.cpp
)

add_executable(test_min_max_heap
  app/test_min_max_heap.cpp
)
target_link_libraries(test_min_max_heap ${Boost_LIBRARIES})

add_executable(test_interval_indices
  app/test_interval_indices.cpp
)
target_link_libraries(test_interval_indices ${Boost_LIBRARIES})

add_executable(test_peak_finder_d
  app/test_peak_finder_d.cpp
)
target_link_libraries(test_peak_finder_d ${Boost_LIBRARIES})

add_executable(test_io
  app/test_io.cpp
)
target_link_libraries(test_io ${SUBLIB_NAME} ${Boost_LIBRARIES})

add_executable(test_numeric_traits
  app/test_numeric_traits.cpp
)
target_link_libraries(test_numeric_traits ${SUBLIB_NAME} ${Boost_LIBRARIES})

add_executable(test_label_set
  app/test_label_set.cpp
)
target_link_libraries(test_label_set ${SUBLIB_NAME} ${Boost_LIBRARIES})

add_executable(test_morton_functions
  app/test_morton_functions.cpp
)
target_link_libraries(test_morton_functions ${SUBLIB_NAME} ${Boost_LIBRARIES})

add_executable(test_morton_order
  app/test_morton_order.cpp
)
target_link_libraries(test_morton_order ${SUBLIB_NAME} ${Boost_LIBRARIES})

add_executable(test_morton_tree
  app/test_morton_tree.cpp
)
target_link_libraries(test_morton_tree ${SUBLIB_NAME} ${Boost_LIBRARIES})

add_executable(test_tls_scalar_consensus
  app/test_tls_scalar_consensus.cpp
)
target_link_libraries(test_tls_scalar_consensus ${SUBLIB_NAME} ${Boost_LIBRARIES})


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
