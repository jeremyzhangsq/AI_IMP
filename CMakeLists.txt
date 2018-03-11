cmake_minimum_required(VERSION 3.8)
project(AI_IMP)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
#set(CMAKE_CXX_FLAGS "-lmysqlclient")

set(BOOST_ROOT "C:/local/boost_1_66_0")

find_package(Boost)
IF (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIR})
    ADD_DEFINITIONS( "-DHAS_BOOST" )
endif()


file(GLOB SOURCES "src/*.cpp")

add_executable(AI_IMP ${SOURCES})

#target_link_libraries(AI_IMP mysqlclient)

if(WIN32)
elseif(unix)
    SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -lprofiler -ltcmalloc -lunwind" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lprofiler -ltcmalloc")
    include_directories(/home/tools/gperftools/include)
    LINK_DIRECTORIES("/home/tools/gperftools/lib")
    FIND_LIBRARY(UNWIND_LIB NAMES unwind PATHS "/home/tools/gperftools/lib/")
    MESSAGE(STATUS "Library path UNWIND_LIB is " ${UNWIND_LIB})
    FIND_LIBRARY(PROFILER_LIB NAMES profiler PATHS "/home/tools/gperftools/lib/")
    MESSAGE(STATUS "Library path PROFILER_LIB is " ${PROFILER_LIB})
    TARGET_LINK_LIBRARIES(AI_IMP ${UNWIND_LIB} ${PROFILER_LIB})
endif()
