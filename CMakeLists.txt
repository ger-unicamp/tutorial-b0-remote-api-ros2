cmake_minimum_required(VERSION 3.3)
project(tutorial-b0-remote-api-ros2)

cmake_policy(SET CMP0057 NEW)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


if(UNIX AND NOT APPLE)
    set(LINUX TRUE)
elseif(WIN32)
    set(WINDOWS TRUE)
endif()
    
if(LINUX)
    set(COPPELIA_ROOT_DIR "$ENV{HOME}/CoppeliaSim_Edu")  
elseif(WINDOWS)
    set(COPPELIA_ROOT_DIR "C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu")
    set(BOOST_DIR "C:/dev/boost_1_76_0") # caminho no pc do natan (vamos tentar padronizar)
    add_compile_options(-DWIN2 -D_WINDOWS -DNDEBUG)
endif()

set(COPPELIA_PROGRAMMING_DIR "${COPPELIA_ROOT_DIR}/programming")
set(COPPELIA_B0BASED_DIR "${COPPELIA_PROGRAMMING_DIR}/b0RemoteApiBindings/cpp")
set(COPPELIA_MSGPACK_DIR "${COPPELIA_B0BASED_DIR}/msgpack-c/include")
set(COPPELIA_B0INCLUDE_DIR "${COPPELIA_PROGRAMMING_DIR}/bluezero/include/b0/bindings")

message(STATUS "[PB] Incluindo diretórios")

include_directories(${COPPELIA_B0BASED_DIR})
include_directories(${COPPELIA_MSGPACK_DIR})
include_directories(${COPPELIA_B0INCLUDE_DIR})
include_directories(${COPPELIA_ROOT_DIR})
if(WINDOWS)
    include_directories(${BOOST_DIR})
endif()

link_directories(SHARED ${COPPELIA_ROOT_DIR})

message(STATUS "[PB] Adicionando os executáveis front-end-node e b0RemoteApi")

add_executable(front-end-node src/front-end-node.cpp ${COPPELIA_B0BASED_DIR}/b0RemoteApi.cpp)
add_executable(back-end-node src/back-end-node.cpp)

target_include_directories(front-end-node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PUBLIC ${COPPELIA_ROOT_DIR} # pasta raiz do Coppelia e localizacao do boost
  PUBLIC ${COPPELIA_B0BASED_DIR} # pasta do b0
  PUBLIC ${COPPELIA_MSGPACK_DIR} # do msgpack
  PUBLIC ${COPPELIA_B0INCLUDE_DIR} # do c.h
  if(WINDOWS) PUBLIC ${BOOST_DIR} endif() # pasta raiz do boost no windows
)

target_include_directories(back-end-node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

message(STATUS "[PB] Encontrando os pacotes das dependencias")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(projeto_bixo_interfaces REQUIRED)
# find_package(b0 REQUIRED)

message(STATUS "[PB] Adicionando dependencias do ROS aos targets")

ament_target_dependencies(front-end-node rclcpp std_msgs projeto_bixo_interfaces)
ament_target_dependencies(back-end-node rclcpp std_msgs projeto_bixo_interfaces)

if(LINUX)
  target_link_libraries(front-end-node b0 boost_system)
elseif(WINDOWS)
  target_link_libraries(front-end-node b0)
endif()

message(STATUS "[PB] Instalando os targets")

install(TARGETS 
  front-end-node
  back-end-node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
