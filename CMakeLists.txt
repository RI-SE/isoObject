project(ISO_object LANGUAGES C CXX)
cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tell the linker to use RPATH instead of RUNPATH
# RPATH/RUNPATH tells a binary where to look for libraries at runtime
# It is useful e.g. when multiple copies of the same shared library exist in various paths (such as LD_LIBRARY_PATH, /usr/local/lib, etc..).
# However, RPATH has higher priority than LD_LIBRARY_PATH, while RUNPATH has lower priority
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--disable-new-dtags")


# Build flags
set(BUILDING_FOR_ANDROID OFF CACHE BOOL "Building for Android")

# Swig flags
set(WITH_SWIG OFF CACHE BOOL "Enable swigging")
set(SWIG_WITH_JAVA OFF CACHE BOOL "Swig to target-language java")
set(SWIG_WITH_PYTHON OFF CACHE BOOL "Swig to target-language python")

if(SWIG_WITH_JAVA)
    set(SWIG_TARGET_LANG java)
elseif(SWIG_WITH_PYTHON)
    set(SWIG_TARGET_LANG python)
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

if (BUILDING_FOR_ANDROID) # Boost is found in variables set by android toolchain upstream
    add_library(boost_system SHARED IMPORTED)
    set_target_properties(boost_system PROPERTIES IMPORTED_LOCATION
                        ${ANDROID_BOOST_LIB_DIR}/libboost_system.so)

    include_directories(${ANDROID_BOOST_INCLUDE_DIR})
else() # Find boost normally
    find_package(Boost REQUIRED COMPONENTS system)
endif()

# Library target
add_library(${ISOOBJECT_LIBRARY} SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/src/iso22133object.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/iso22133state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/trajDecoder.cpp
)

target_include_directories(${ISOOBJECT_LIBRARY} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/sigslot/include/sigslot
    ${CMAKE_CURRENT_SOURCE_DIR}/iso22133
)
set(ISO_OBJ_LIBS 
    ${ISO22133_LIBRARY} atomic pthread Boost::system)
set(ISO_OBJ_LIBS_ANDROID 
    ${ISO22133_LIBRARY} atomic ${boost_system})
if (BUILDING_FOR_ANDROID)
    target_link_libraries(${ISOOBJECT_LIBRARY}
        ${ISO_OBJ_LIBS_ANDROID}
    )
else()
    target_link_libraries(${ISOOBJECT_LIBRARY}
        ${ISO_OBJ_LIBS}
)
endif()

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

if (NOT WITH_SWIG AND NOT BUILDING_FOR_ANDROID)
find_package(Boost REQUIRED COMPONENTS program_options)

# Demo target, only build if not swigging
add_executable(${ISOOBJECT_TEST} demoIsoObject.cpp)

target_include_directories(${ISOOBJECT_TEST} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

target_link_libraries(${ISOOBJECT_TEST} LINK_PUBLIC
    ${ISOOBJECT_LIBRARY}
    Boost::program_options
)

endif()

install(CODE "MESSAGE(STATUS \"Installing target ${ISOOBJECT_LIBRARY}\")")
install(TARGETS ${ISOOBJECT_LIBRARY}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# SWIG
if (WITH_SWIG)
    find_package(SWIG REQUIRED)
    if(NOT SWIG_FOUND)
        message(WARNING "SWIG not found")
    else()
        #Policy for swig_add_library to put target inside first argument to the function
        cmake_policy(SET CMP0078 NEW)
        #Policy for passing SWIG_MODULE_NAME to swig compiler as -module <name>
        cmake_policy(SET CMP0086 NEW)

        include(${SWIG_USE_FILE})

        if(SWIG_WITH_JAVA)
            if (NOT ${CMAKE_SYSTEM_NAME} STREQUAL "Android")
                find_package(JNI REQUIRED)
            endif()
            SET(CMAKE_SWIG_FLAGS -package org.asta.isoObject)
        elseif(SWIG_WITH_PYTHON)
            find_package(Python COMPONENTS Development)
            SET(CMAKE_SWIG_FLAGS "-threads")
        endif()

        set(ISO22133_SWIGGED isoObject_wrap)

        set_property(SOURCE isoObject.i PROPERTY CPLUSPLUS ON)
        set_property(SOURCE isoObject.i PROPERTY INCLUDE_DIRECTORIES
            ${CMAKE_CURRENT_SOURCE_DIR}/inc
            ${CMAKE_CURRENT_SOURCE_DIR}/sigslot/include/sigslot
            ${CMAKE_CURRENT_SOURCE_DIR}/iso22133
            ${Python_INCLUDE_DIRS} #Note that cmake ignored empty variables
            ${JNI_INCLUDE_DIRS}
            ${Boost_INCLUDE_DIRS}
        )
        swig_add_library(${ISO22133_SWIGGED}
            TYPE SHARED
            LANGUAGE ${SWIG_TARGET_LANG}
            SOURCES isoObject.i
        )
        set_property(TARGET ${ISO22133_SWIGGED} PROPERTY USE_TARGET_INCLUDE_DIRECTORIES TRUE)
        target_include_directories(${ISO22133_SWIGGED} PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/inc
            ${CMAKE_CURRENT_SOURCE_DIR}/sigslot/include/sigslot
            ${CMAKE_CURRENT_SOURCE_DIR}/iso22133
            ${Python_INCLUDE_DIRS}
            ${JNI_INCLUDE_DIRS}
        )

        target_link_libraries(${ISO22133_SWIGGED}
            ${ISOOBJECT_LIBRARY}
            ${ISO22133_LIBRARY}
            atomic
            ${Python_LIBRARIES}
            ${JNI_LIBRARIES}
        )

        if(SWIG_WITH_PYTHON)
            execute_process(COMMAND python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())" OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)
            install(TARGETS ${ISO22133_SWIGGED} DESTINATION ${PYTHON_SITE_PACKAGES})
            install(FILES ${CMAKE_BINARY_DIR}/isoObject_wrap.py DESTINATION ${PYTHON_SITE_PACKAGES})
        endif()
    endif()
endif()

# Build tests 
if (BUILD_TESTING)
	enable_testing()
	file(GLOB TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
	add_executable(${ISOOBJECT_LIBRARY}_test
		${TEST_SOURCES}
	)
	target_link_libraries(${ISOOBJECT_LIBRARY}_test
		gtest_main
		${ISOOBJECT_LIBRARY}
	)
	target_include_directories(${ISOOBJECT_LIBRARY}_test PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/inc
	)
	include(GoogleTest)
	gtest_discover_tests(${ISOOBJECT_LIBRARY}_test)
endif()