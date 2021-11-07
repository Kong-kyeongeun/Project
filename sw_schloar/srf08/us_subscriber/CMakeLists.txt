cmake_minimum_required(VERSION 3.0.2) 
project(us) 
## 캐킨빌드를할때요구되는구성요소패키지이다.
## 의존성패키지로message_generation, std_msgs, roscpp이며이패키지들이존재하지않으면빌드도중에에러가난다.
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp)
## 메시지선언: MsgTutorial.msg 
add_message_files(FILES Msgus.msg)
## 의존하는메시지를설정하는옵션이다.
## std_msgs가설치되어있지않다면빌드도중에에러가난다.
generate_messages(DEPENDENCIES std_msgs)
## 캐킨패키지옵션으로라이브러리, 캐킨빌드의존성, 시스템의존패키지를기술한다. 
catkin_package( 
LIBRARIES us
CATKIN_DEPENDS std_msgs roscpp
)

## 인클루드디렉터리를설정한다.
include_directories(${catkin_INCLUDE_DIRS})
## topic_publisher노드에대한빌드옵션이다. 
## 실행파일, 타깃링크라이브러리, 추가의존성등을설정한다.

## topic_subscriber노드에대한빌드옵션이다. 
add_executable(us_subscriber src/us_subscriber.cpp)
add_dependencies(us_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(us_subscriber ${catkin_LIBRARIES})

