cmake_minimum_required (VERSION 3.21)

project (samplePCL)

set (royale_DIR CACHE PATH "Path to the share folder inside the Royale package")

find_package (PCL 1.3 QUIET)

if (NOT PCL_FOUND)
    message ("PCL example will not be build as no PCL was found!")
    return ()
endif ()

find_package (royale REQUIRED)

link_directories (${royale_LIB_DIR} ${PCL_LIBRARY_DIRS})

include_directories (${royale_INCLUDE_DIRS} .)

add_definitions (${PCL_DEFINITIONS} -D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS)

add_executable (samplePCL samplePCL.cpp)

copy_royale_libs (samplePCL)

target_link_libraries (samplePCL ${royale_LIBS} ${PCL_LIBRARIES})

add_custom_command(TARGET samplePCL POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:samplePCL> $<TARGET_FILE_DIR:samplePCL>
  COMMAND_EXPAND_LISTS
  )