cmake_minimum_required(VERSION 3.10)
project(cloud_viewer)

set(CMAKE_CXX_STANDARD 11)

#add_executable(cloud_viewer main.cpp)


find_package(Boost REQUIRED thread)


include_directories(${Boost_INCLUDE_DIRS})

# Find Packages
find_package( PCL 1.8 REQUIRED )
find_package(Qt5Widgets)



include_directories(${Boost_INCLUDE_DIRS})

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
    target_link_libraries( cloud_viewer ${Boost_LIBRARIES})
    target_link_libraries( cloud_viewer ${PCL_LIBRARIES})
    target_link_libraries( cloud_viewer Qt5::Widgets )
    #target_link_libraries( cloud_viewer ${PCL_COMMON_LIBRARIES} Qt5::Widgets )
    #target_link_libraries( cloud_viewer pthread)
    target_link_libraries( cloud_viewer boost_thread boost_system boost_chrono pthread)

#endif()

