cmake_minimum_required(VERSION 3.16)
project(XgcLightweightSimInterfaces VERSION 0.3.0 LANGUAGES NONE)
include(CMakePackageConfigHelpers)
set(CMAKE_INSTALL_INCLUDEDIR "include" CACHE PATH "Header installation directory")
set(CMAKE_INSTALL_DATADIR "share" CACHE PATH "Architecture-independent data directory")
set(package_dir "${CMAKE_INSTALL_DATADIR}/cmake/XgcLightweightSimInterfaces")
add_library(xgc_lightweight_sim_interfaces INTERFACE)
add_library(XgcLightweightSim::Interfaces ALIAS xgc_lightweight_sim_interfaces)
set_target_properties(xgc_lightweight_sim_interfaces PROPERTIES EXPORT_NAME Interfaces)
target_include_directories(xgc_lightweight_sim_interfaces INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
install(FILES include/xgc-lightweight-sim/simulation_records_v1.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/xgc-lightweight-sim)
install(TARGETS xgc_lightweight_sim_interfaces EXPORT XgcLightweightSimInterfacesTargets)
install(EXPORT XgcLightweightSimInterfacesTargets NAMESPACE XgcLightweightSim::
  DESTINATION ${package_dir})
configure_package_config_file(cmake/XgcLightweightSimInterfacesConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/XgcLightweightSimInterfacesConfig.cmake
  INSTALL_DESTINATION ${package_dir})
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/XgcLightweightSimInterfacesConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion ARCH_INDEPENDENT)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/XgcLightweightSimInterfacesConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/XgcLightweightSimInterfacesConfigVersion.cmake
  DESTINATION ${package_dir})
