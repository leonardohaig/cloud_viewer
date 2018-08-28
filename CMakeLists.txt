cmake_minimum_required(VERSION 3.10)
project(cloud_viewer)

set(CMAKE_CXX_STANDARD 11)

#add_executable(cloud_viewer main.cpp)

# Find Packages
find_package( PCL 1.8 REQUIRED )

find_package(Qt5Widgets)

#if( PCL_FOUND )
    # Additional Include Directories
    # [C/C++]>[General]>[Additional Include Directories]
    include_directories( ${PCL_INCLUDE_DIRS} )

    # Preprocessor Definitions
    # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
    add_definitions( ${PCL_DEFINITIONS} )
    #add_definitions( -DPCL_NO_PRECOMPILE )

    # Additional Library Directories
    # [Linker]>[General]>[Additional Library Directories]
    link_directories( ${PCL_LIBRARY_DIRS} )

add_executable(cloud_viewer main.cpp)


    # Additional Dependencies
    # [Linker]>[Input]>[Additional Dependencies]
    target_link_libraries( cloud_viewer ${PCL_LIBRARIES} Qt5::Widgets )

#endif()

