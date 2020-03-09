cmake_minimum_required(VERSION 3.3)
project(lsmServer)

if (BOOST_DIR)
  set(BOOST_ROOT ${BOOST_DIR})
endif (BOOST_DIR)

find_package( Threads REQUIRED)
find_package( Boost 1.70.0 REQUIRED COMPONENTS filesystem system regex locale)

set(lsmServerSrc 
    asioLSMserver.cpp 
    lsmSession.hpp
   )

add_executable(${PROJECT_NAME} ${lsmServerSrc})
target_include_directories(
	${PROJECT_NAME}
	PUBLIC
        ${Boost_INCLUDE_DIRS}
	)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic ) #-Werror )
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
target_link_libraries(${PROJECT_NAME} 
                       Threads::Threads
	               ${Boost_LIBRARIES}
 )

