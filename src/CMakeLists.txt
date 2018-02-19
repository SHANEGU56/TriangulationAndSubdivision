set(SRCS
        main.cpp
        point_collection.cpp point_collection.h
        greedy_projection.cpp greedy_projection.h general_hd.h Txt2pcd.cpp Txt2pcd.h Subdivision.cpp Subdivision.h)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(landmarks_triangulation ${SRCS})
target_link_libraries (landmarks_triangulation ${PCL_LIBRARIES})