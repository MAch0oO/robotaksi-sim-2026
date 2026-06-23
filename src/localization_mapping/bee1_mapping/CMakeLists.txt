cmake_minimum_required(VERSION 3.5)
project(bee1_mapping)

find_package(ament_cmake REQUIRED)

# Launch ve config klasörlerini paket paylaşım dizinine kur
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
