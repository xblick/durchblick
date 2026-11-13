# Durchblick for C++ components, and the Python module blick._core (scikit-build-core, pyproject.toml).
#
# Targets (installed as a CMake package: find_package(durchblick) gives durchblick::core and durchblick::durchblick):
#   durchblick::core        the shared-memory and CUDA rings (ring.hpp, cuda_ring.hpp, protocol.hpp); no Zenoh
#   durchblick::durchblick  the camera API on zenoh-cpp (camera.hpp: CameraPublisher, ImageSubscriber), messages,
#                           JPEG/PNG; needs zenohc and zenohcxx (1.10), libjpeg(-turbo) and libpng
#   durchblick-cpp-camera, durchblick-cpp-probe   command-line tools (and the C++ side of the interop tests)
#   durchblick::points      with DURCHBLICK_POINTS (needs a CUDA toolkit with nvcc): the point cloud node
#                           (point_cloud_node.hpp) and its program durchblick-points
#
#   cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH=<zenoh-c/zenoh-cpp prefix> [-DDURCHBLICK_CUDA_INCLUDE_DIR=<dir>]
#         [-DDURCHBLICK_POINTS=ON -DCUDAToolkit_ROOT=<toolkit>]
#   cmake --build build && ctest --test-dir build
#
# The Python wheel builds only durchblick::core and the module (Python has its own Zenoh).
cmake_minimum_required(VERSION 3.24)
# the one place of the package version: pyproject.toml reads it from here, C++ gets it as durchblick/version.hpp
project(durchblick VERSION 0.4.3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(SKBUILD)
  set(durchblick_camera_default OFF)
else()
  set(durchblick_camera_default ON)
endif()
option(DURCHBLICK_CAMERA "build the C++ camera API (needs zenohc, zenohcxx, JPEG, PNG)" ${durchblick_camera_default})
option(DURCHBLICK_POINTS "build the point cloud node (needs the camera API and a CUDA toolkit with nvcc)" OFF)
if(DURCHBLICK_POINTS AND NOT DURCHBLICK_CAMERA)
  message(FATAL_ERROR "DURCHBLICK_POINTS needs DURCHBLICK_CAMERA")
endif()

# Only the header cuda.h is needed at build time; libcuda.so.1 is opened at run time. Without a CUDA toolkit it comes
# from the pip package nvidia-cuda-runtime (the Python build finds it there).
set(DURCHBLICK_CUDA_INCLUDE_DIR "" CACHE PATH "directory that contains cuda.h")
if(NOT DURCHBLICK_CUDA_INCLUDE_DIR AND SKBUILD)
  # the build environment of pyproject.toml has the pip package nvidia-cuda-runtime
  find_package(Python 3.10 REQUIRED COMPONENTS Interpreter)
  execute_process(COMMAND "${Python_EXECUTABLE}" -c
                  "import nvidia, os; print(os.path.join(list(nvidia.__path__)[0], 'cu13', 'include'))"
                  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE DURCHBLICK_CUDA_INCLUDE_DIR)
endif()
if(NOT DURCHBLICK_CUDA_INCLUDE_DIR)
  find_package(CUDAToolkit QUIET)
  if(CUDAToolkit_FOUND)
    set(DURCHBLICK_CUDA_INCLUDE_DIR "${CUDAToolkit_INCLUDE_DIRS}")
  endif()
endif()
if(NOT EXISTS "${DURCHBLICK_CUDA_INCLUDE_DIR}/cuda.h")
  message(FATAL_ERROR "cuda.h not found: install a CUDA toolkit or set DURCHBLICK_CUDA_INCLUDE_DIR "
                      "(e.g. to site-packages/nvidia/cu13/include of the pip package nvidia-cuda-runtime)")
endif()

configure_file(cpp/include/durchblick/version.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/include/durchblick/version.hpp @ONLY)
find_package(Threads REQUIRED)

# ------------------------------------------------------------------------------------------------ core

add_library(durchblick_core cpp/src/ring.cpp cpp/src/cuda_ring.cpp cpp/src/rvl.cpp cpp/src/ray_cloud.cpp
            cpp/src/rectify.cpp cpp/src/pairing.cpp)
add_library(durchblick::core ALIAS durchblick_core)
set_target_properties(durchblick_core PROPERTIES EXPORT_NAME core)
target_include_directories(durchblick_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include> $<INSTALL_INTERFACE:include>
  PRIVATE ${DURCHBLICK_CUDA_INCLUDE_DIR})
target_link_libraries(durchblick_core PUBLIC ${CMAKE_DL_LIBS} Threads::Threads rt)
target_compile_options(durchblick_core PRIVATE -Wall -Wextra)

# ---------------------------------------------------------------------------------------- camera API

if(DURCHBLICK_CAMERA)
  find_package(zenohc 1.10 REQUIRED)
  find_package(zenohcxx 1.10 REQUIRED)
  find_package(JPEG REQUIRED)
  find_package(PNG REQUIRED)
  add_library(durchblick_camera cpp/src/msg.cpp cpp/src/camera_info_file.cpp cpp/src/image.cpp cpp/src/stream.cpp
              cpp/src/camera.cpp cpp/src/points.cpp cpp/src/parameters.cpp cpp/src/options.cpp)
  add_library(durchblick::durchblick ALIAS durchblick_camera)
  set_target_properties(durchblick_camera PROPERTIES EXPORT_NAME durchblick OUTPUT_NAME durchblick)
  target_link_libraries(durchblick_camera PUBLIC durchblick_core zenohcxx::zenohc PRIVATE JPEG::JPEG PNG::PNG)
  target_compile_options(durchblick_camera PRIVATE -Wall -Wextra)

  add_executable(durchblick-cpp-camera cpp/tools/camera_tool.cpp)
  target_link_libraries(durchblick-cpp-camera PRIVATE durchblick_camera)
  add_executable(durchblick-cpp-probe cpp/tools/probe_tool.cpp)
  target_link_libraries(durchblick-cpp-probe PRIVATE durchblick_camera)
endif()

# ---------------------------------------------------------------------------------------- point clouds

if(DURCHBLICK_POINTS)
  # every major GPU generation from the toolkit's oldest on, plus PTX for newer ones (robots: RTX 2080 to Blackwell)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES all-major)
  endif()
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  add_library(durchblick_points cpp/points/point_cloud_node.cpp cpp/points/camera_model.cpp cpp/points/kernels.cu)
  add_library(durchblick::points ALIAS durchblick_points)
  # the shared CUDA runtime, as the other CUDA users of a process (e.g. durchblick-azure-kinect's nvJPEG) link it
  set_target_properties(durchblick_points PROPERTIES EXPORT_NAME points OUTPUT_NAME durchblick_points
                                                     CUDA_RUNTIME_LIBRARY Shared)
  target_link_libraries(durchblick_points PUBLIC durchblick_camera PRIVATE CUDA::cudart)
  # all-major includes generations before the robots' oldest (Turing); nvcc warns that it will drop them
  target_compile_options(durchblick_points PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>
                                                   $<$<COMPILE_LANGUAGE:CUDA>:-Wno-deprecated-gpu-targets>)

  add_executable(durchblick-points cpp/points/main.cpp)
  target_link_libraries(durchblick-points PRIVATE durchblick_points)
  # installed, it finds the toolkit's libcudart where it was linked against it
  set_target_properties(durchblick-points PROPERTIES INSTALL_RPATH_USE_LINK_PATH ON)
endif()

# ------------------------------------------------------------------------------------- Python module

if(SKBUILD)
  find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development.Module)
  execute_process(COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
                  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
  find_package(nanobind CONFIG REQUIRED)
  nanobind_add_module(_core NB_STATIC cpp/python/bindings.cpp)
  target_link_libraries(_core PRIVATE durchblick_core)
  install(TARGETS _core LIBRARY DESTINATION blick)
  return()
endif()

# ---------------------------------------------------------------------------------- tests and install

include(CTest)
if(BUILD_TESTING)
  add_executable(test_rings cpp/tests/test_rings.cpp)
  target_link_libraries(test_rings PRIVATE durchblick_core)
  add_test(NAME host_ring COMMAND test_rings host)
  add_executable(test_rectify cpp/tests/test_rectify.cpp)
  target_link_libraries(test_rectify PRIVATE durchblick_core)
  add_test(NAME rectify COMMAND test_rectify)
  add_test(NAME cuda_ring COMMAND test_rings cuda)
  set_tests_properties(cuda_ring PROPERTIES SKIP_RETURN_CODE 77)  # no GPU: skipped, not failed
  if(DURCHBLICK_CAMERA)
    add_executable(test_messages cpp/tests/test_messages.cpp)
    target_link_libraries(test_messages PRIVATE durchblick_camera)
    add_test(NAME messages COMMAND test_messages)
    add_executable(test_parameters cpp/tests/test_parameters.cpp)
    target_link_libraries(test_parameters PRIVATE durchblick_camera)
    add_test(NAME parameters COMMAND test_parameters)
    add_executable(test_options cpp/tests/test_options.cpp)
    target_link_libraries(test_options PRIVATE durchblick_camera)
    add_test(NAME options COMMAND test_options)
    add_executable(test_partner cpp/tests/test_partner.cpp)
    target_link_libraries(test_partner PRIVATE durchblick_camera)
    add_test(NAME partner COMMAND test_partner)
    add_executable(test_lifetime cpp/tests/test_lifetime.cpp)
    target_link_libraries(test_lifetime PRIVATE durchblick_camera)
    add_test(NAME lifetime COMMAND test_lifetime)
    add_executable(test_latest cpp/tests/test_latest.cpp)
    target_link_libraries(test_latest PRIVATE durchblick_camera)
    target_include_directories(test_latest PRIVATE cpp/src)  # detail::Latest, an internal of the library
    add_test(NAME latest COMMAND test_latest)
  endif()
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(durchblick_targets durchblick_core)
if(DURCHBLICK_CAMERA)
  list(APPEND durchblick_targets durchblick_camera)
  install(TARGETS durchblick-cpp-camera durchblick-cpp-probe RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
if(DURCHBLICK_POINTS)
  list(APPEND durchblick_targets durchblick_points)
  install(TARGETS durchblick-points RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
install(TARGETS ${durchblick_targets} EXPORT durchblickTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY cpp/include/durchblick DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} PATTERN "*.in" EXCLUDE)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/durchblick/version.hpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/durchblick)
install(EXPORT durchblickTargets NAMESPACE durchblick:: FILE durchblickTargets.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/durchblick)
set(DURCHBLICK_WITH_CAMERA ${DURCHBLICK_CAMERA})
set(DURCHBLICK_WITH_POINTS ${DURCHBLICK_POINTS})
# the CUDA toolkit the points library was built with: its consumers find the same one (their build may name none)
set(DURCHBLICK_CUDA_TOOLKIT_ROOT "")
if(DURCHBLICK_POINTS)
  get_filename_component(DURCHBLICK_CUDA_TOOLKIT_ROOT "${CUDAToolkit_BIN_DIR}" DIRECTORY)
endif()
configure_package_config_file(cmake/durchblickConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/durchblickConfig.cmake
                              INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/durchblick)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/durchblickConfigVersion.cmake
                                 COMPATIBILITY SameMinorVersion)  # 0.x: the minor version is the protocol line
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/durchblickConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/durchblickConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/durchblick)
