cmake_minimum_required(VERSION 3.14)
project(ISO_object LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tell the linker to use RPATH instead of RUNPATH
# RPATH/RUNPATH tells a binary where to look for libraries at runtime
# It is useful e.g. when multiple copies of the same shared library exist in various paths (such as LD_LIBRARY_PATH, /usr/local/lib, etc..).
# However, RPATH has higher priority than LD_LIBRARY_PATH, while RUNPATH has lower priority
#set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--disable-new-dtags") #throws error?


# Build flags
set(BUILDING_FOR_ANDROID OFF CACHE BOOL "Building for Android")
set(BUILD_TESTING OFF CACHE BOOL "Enable testing")

# Swig flags
set(WITH_SWIG OFF CACHE BOOL "Enable swigging")
set(SWIG_WITH_JAVA OFF CACHE BOOL "Swig to target-language java")
set(SWIG_WITH_PYTHON OFF CACHE BOOL "Swig to target-language python")

if(SWIG_WITH_JAVA)
    set(SWIG_TARGET_LANG java)
    add_compile_definitions(WITH_SWIG)
elseif(SWIG_WITH_PYTHON)
    set(SWIG_TARGET_LANG python)
    add_compile_definitions(WITH_SWIG)
endif()

# Manage namespace of the project
set(ISOOBJECT_LIBRARY ${PARENT_TARGET_NAMESPACE}${PROJECT_NAME})
set(ISOOBJECT_TEST ${ISOOBJECT_LIBRARY}Demo)

#ISO22133 library/headers
add_subdirectory(iso22133)
set(ISO22133_LIBRARY ISO22133)


# Set the RPATH to the install directory
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/")
# Tell linker to set RPATH to the install directory, outside the build tree
# Note: this has to be done before add_executable/add_library.
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

include(GNUInstallDirs)




find_package(Boost 1.83.0 REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
if(Boost_FOUND)
    set(BOOST_INCLUDE_DIR "${Boost_ROOT}")
    set(Boost_DEBUG ON)
    set(Boost_USE_STATIC_LIBS   OFF) #Set to on
    message(STATUS "Boost found: ${Boost_VERSION}")
    message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost libraries: ${Boost_LIBRARIES}")
else()
    message(FATAL_ERROR "Boost not found")
endif()
# add_definitions(-DBOOST_ALL_NO_LIB)
# add_subdirectory(${Boost_INSTALL_DIR}/boost)
set(Boost_USE_STATIC_ ON)


# Library target
add_library(${ISOOBJECT_LIBRARY} SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/src/iso22133object.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/iso22133state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/trajDecoder.cpp
)
target_compile_definitions(${ISOOBJECT_LIBRARY} PRIVATE ISO_OBJECT_EXPORTS)

target_include_directories(${ISOOBJECT_LIBRARY} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/sigslot/include/sigslot
    ${CMAKE_CURRENT_SOURCE_DIR}/iso22133
    ${Boost_INCLUDE_DIRS}
)
set(ISO_OBJ_LIBS 
    ${ISO22133_LIBRARY})# Boost::system)

target_link_libraries(${ISOOBJECT_LIBRARY}
    ${ISO_OBJ_LIBS}
    ${Boost_LIBRARIES}
)


set(CMAKE_VERBOSE_MAKEFILE ON)

set_property(TARGET ${ISOOBJECT_LIBRARY} PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/iso22133object.hpp
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/iso22133state.hpp
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/trajDecoder.hpp
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/sigslot/include/sigslot/signal.hpp
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/iso22133/iso22133.h
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/tcpServer.hpp
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/udpServer.hpp
)

install(CODE "MESSAGE(STATUS \"Installing target ${ISOOBJECT_LIBRARY}\")")
install(TARGETS ${ISOOBJECT_LIBRARY}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
