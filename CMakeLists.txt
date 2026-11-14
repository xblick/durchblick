# Durchblick for C++ components, and the Python module xblick._core (scikit-build-core, pyproject.toml).
#
# Targets (installed as a CMake package: find_package(xblick) gives xblick::core and xblick::xblick):
#   xblick::core    the shared-memory and CUDA rings (ring.hpp, cuda_ring.hpp, protocol.hpp); no Zenoh
#   xblick::xblick  the camera API on zenoh-cpp (camera.hpp: CameraPublisher, ImageSubscriber), messages,
#                   JPEG/PNG; needs zenohc and zenohcxx (1.10), libjpeg(-turbo) and libpng
#   xblick-cpp-camera, xblick-cpp-probe   command-line tools (and the C++ side of the interop tests)
#   xblick::points  with XBLICK_POINTS (needs a CUDA toolkit with nvcc): the point cloud node
#                   (point_cloud_node.hpp) and its program xblick-points
#
#   cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH=<zenoh-c/zenoh-cpp prefix> [-DXBLICK_CUDA_INCLUDE_DIR=<dir>]
#         [-DXBLICK_POINTS=ON -DCUDAToolkit_ROOT=<toolkit>]
#   cmake --build build && ctest --test-dir build
#
# The Python wheel builds only xblick::core and the module (Python has its own Zenoh).
cmake_minimum_required(VERSION 3.24)
# the one place of the package version: pyproject.toml reads it from here, C++ gets it as xblick/version.hpp
project(xblick VERSION 0.5.6 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(SKBUILD)
  set(xblick_camera_default OFF)
else()
  set(xblick_camera_default ON)
endif()
option(XBLICK_CAMERA "build the C++ camera API (needs zenohc, zenohcxx, JPEG, PNG)" ${xblick_camera_default})
option(XBLICK_POINTS "build the point cloud node (needs the camera API and a CUDA toolkit with nvcc)" OFF)
if(XBLICK_POINTS AND NOT XBLICK_CAMERA)
  message(FATAL_ERROR "XBLICK_POINTS needs XBLICK_CAMERA")
endif()

# Only the header cuda.h is needed at build time; libcuda.so.1 is opened at run time. Without a CUDA toolkit it comes
# from the pip package nvidia-cuda-runtime (the Python build finds it there).
set(XBLICK_CUDA_INCLUDE_DIR "" CACHE PATH "directory that contains cuda.h")
if(NOT XBLICK_CUDA_INCLUDE_DIR AND SKBUILD)
  # the build environment of pyproject.toml has the pip package nvidia-cuda-runtime
  find_package(Python 3.10 REQUIRED COMPONENTS Interpreter)
  execute_process(COMMAND "${Python_EXECUTABLE}" -c
                  "import nvidia, os; print(os.path.join(list(nvidia.__path__)[0], 'cu13', 'include'))"
                  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE XBLICK_CUDA_INCLUDE_DIR)
endif()
if(NOT XBLICK_CUDA_INCLUDE_DIR)
  find_package(CUDAToolkit QUIET)
  if(CUDAToolkit_FOUND)
    set(XBLICK_CUDA_INCLUDE_DIR "${CUDAToolkit_INCLUDE_DIRS}")
  endif()
endif()
if(NOT EXISTS "${XBLICK_CUDA_INCLUDE_DIR}/cuda.h")
  message(FATAL_ERROR "cuda.h not found: install a CUDA toolkit or set XBLICK_CUDA_INCLUDE_DIR "
                      "(e.g. to site-packages/nvidia/cu13/include of the pip package nvidia-cuda-runtime)")
endif()

configure_file(cpp/include/xblick/version.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/include/xblick/version.hpp @ONLY)
find_package(Threads REQUIRED)

# ------------------------------------------------------------------------------------------------ core

add_library(xblick_core cpp/src/ring.cpp cpp/src/cuda_ring.cpp cpp/src/rvl.cpp cpp/src/ray_cloud.cpp
            cpp/src/rectify.cpp cpp/src/pairing.cpp cpp/src/paths.cpp)
add_library(xblick::core ALIAS xblick_core)
set_target_properties(xblick_core PROPERTIES EXPORT_NAME core)
target_include_directories(xblick_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include> $<INSTALL_INTERFACE:include>
  PRIVATE ${XBLICK_CUDA_INCLUDE_DIR})
target_link_libraries(xblick_core PUBLIC ${CMAKE_DL_LIBS} Threads::Threads rt)
target_compile_options(xblick_core PRIVATE -Wall -Wextra)

# ---------------------------------------------------------------------------------------- camera API

if(XBLICK_CAMERA)
  find_package(zenohc 1.10 REQUIRED)
  find_package(zenohcxx 1.10 REQUIRED)
  find_package(JPEG REQUIRED)
  find_package(PNG REQUIRED)
  add_library(xblick_camera cpp/src/msg.cpp cpp/src/camera_info_file.cpp cpp/src/image.cpp cpp/src/stream.cpp
              cpp/src/camera.cpp cpp/src/points.cpp cpp/src/parameters.cpp cpp/src/options.cpp)
  add_library(xblick::xblick ALIAS xblick_camera)
  set_target_properties(xblick_camera PROPERTIES EXPORT_NAME xblick OUTPUT_NAME xblick)
  target_link_libraries(xblick_camera PUBLIC xblick_core zenohcxx::zenohc PRIVATE JPEG::JPEG PNG::PNG)
  target_compile_options(xblick_camera PRIVATE -Wall -Wextra)

  add_executable(xblick-cpp-camera cpp/tools/camera_tool.cpp)
  target_link_libraries(xblick-cpp-camera PRIVATE xblick_camera)
  add_executable(xblick-cpp-probe cpp/tools/probe_tool.cpp)
  target_link_libraries(xblick-cpp-probe PRIVATE xblick_camera)
endif()

# ---------------------------------------------------------------------------------------- point clouds

if(XBLICK_POINTS)
  # every major GPU generation from the toolkit's oldest on, plus PTX for newer ones (robots: RTX 2080 to Blackwell)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES all-major)
  endif()
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  add_library(xblick_points cpp/points/point_cloud_node.cpp cpp/points/camera_model.cpp cpp/points/kernels.cu)
  add_library(xblick::points ALIAS xblick_points)
  # the shared CUDA runtime, as the other CUDA users of a process (e.g. xblick-azure-kinect's nvJPEG) link it
  set_target_properties(xblick_points PROPERTIES EXPORT_NAME points OUTPUT_NAME xblick_points
                                                     CUDA_RUNTIME_LIBRARY Shared)
  target_link_libraries(xblick_points PUBLIC xblick_camera PRIVATE CUDA::cudart)
  # all-major includes generations before the robots' oldest (Turing); nvcc warns that it will drop them
  target_compile_options(xblick_points PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>
                                                   $<$<COMPILE_LANGUAGE:CUDA>:-Wno-deprecated-gpu-targets>)

  add_executable(xblick-points cpp/points/main.cpp)
  target_link_libraries(xblick-points PRIVATE xblick_points)
  # installed, it finds the toolkit's libcudart where it was linked against it
  set_target_properties(xblick-points PROPERTIES INSTALL_RPATH_USE_LINK_PATH ON)
endif()

# ------------------------------------------------------------------------------------- Python module

if(SKBUILD)
  find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development.Module)
  execute_process(COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
                  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
  find_package(nanobind CONFIG REQUIRED)
  nanobind_add_module(_core NB_STATIC cpp/python/bindings.cpp)
  target_link_libraries(_core PRIVATE xblick_core)
  install(TARGETS _core LIBRARY DESTINATION xblick)
  return()
endif()

# ---------------------------------------------------------------------------------- tests and install

include(CTest)
if(BUILD_TESTING)
  add_executable(test_rings cpp/tests/test_rings.cpp)
  target_link_libraries(test_rings PRIVATE xblick_core)
  add_test(NAME host_ring COMMAND test_rings host)
  add_executable(test_rectify cpp/tests/test_rectify.cpp)
  target_link_libraries(test_rectify PRIVATE xblick_core)
  add_test(NAME rectify COMMAND test_rectify)
  add_executable(test_paths cpp/tests/test_paths.cpp)
  target_link_libraries(test_paths PRIVATE xblick_core)
  add_test(NAME paths COMMAND test_paths)
  add_test(NAME cuda_ring COMMAND test_rings cuda)
  set_tests_properties(cuda_ring PROPERTIES SKIP_RETURN_CODE 77)  # no GPU: skipped, not failed
  if(XBLICK_CAMERA)
    add_executable(test_messages cpp/tests/test_messages.cpp)
    target_link_libraries(test_messages PRIVATE xblick_camera)
    add_test(NAME messages COMMAND test_messages)
    add_executable(test_parameters cpp/tests/test_parameters.cpp)
    target_link_libraries(test_parameters PRIVATE xblick_camera)
    add_test(NAME parameters COMMAND test_parameters)
    add_executable(test_options cpp/tests/test_options.cpp)
    target_link_libraries(test_options PRIVATE xblick_camera)
    add_test(NAME options COMMAND test_options)
    add_executable(test_partner cpp/tests/test_partner.cpp)
    target_link_libraries(test_partner PRIVATE xblick_camera)
    add_test(NAME partner COMMAND test_partner)
    add_executable(test_lifetime cpp/tests/test_lifetime.cpp)
    target_link_libraries(test_lifetime PRIVATE xblick_camera)
    add_test(NAME lifetime COMMAND test_lifetime)
    add_executable(test_latest cpp/tests/test_latest.cpp)
    target_link_libraries(test_latest PRIVATE xblick_camera)
    target_include_directories(test_latest PRIVATE cpp/src)  # detail::Latest, an internal of the library
    add_test(NAME latest COMMAND test_latest)
  endif()
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(xblick_targets xblick_core)
if(XBLICK_CAMERA)
  list(APPEND xblick_targets xblick_camera)
  install(TARGETS xblick-cpp-camera xblick-cpp-probe RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
if(XBLICK_POINTS)
  list(APPEND xblick_targets xblick_points)
  install(TARGETS xblick-points RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
install(TARGETS ${xblick_targets} EXPORT xblickTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY cpp/include/xblick DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} PATTERN "*.in" EXCLUDE)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/xblick/version.hpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/xblick)
install(EXPORT xblickTargets NAMESPACE xblick:: FILE xblickTargets.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/xblick)
set(XBLICK_WITH_CAMERA ${XBLICK_CAMERA})
set(XBLICK_WITH_POINTS ${XBLICK_POINTS})
# the CUDA toolkit the points library was built with: its consumers find the same one (their build may name none)
set(XBLICK_CUDA_TOOLKIT_ROOT "")
if(XBLICK_POINTS)
  get_filename_component(XBLICK_CUDA_TOOLKIT_ROOT "${CUDAToolkit_BIN_DIR}" DIRECTORY)
endif()
configure_package_config_file(cmake/xblickConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/xblickConfig.cmake
                              INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/xblick)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/xblickConfigVersion.cmake
                                 COMPATIBILITY SameMinorVersion)  # 0.x: the minor version is the protocol line
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/xblickConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/xblickConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/xblick)
