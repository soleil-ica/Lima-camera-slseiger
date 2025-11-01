# --------------------------------------------------------------------------
# Dependencies
# --------------------------------------------------------------------------

find_package(yat CONFIG REQUIRED)

# --------------------------------------------------------------------------
# Collect sources and includes
# --------------------------------------------------------------------------

file(GLOB_RECURSE sources src/*.cpp)
set(includedirs
    include/
    slsDetectorPackage/slsDetectorSoftware/commonFiles/
    slsDetectorPackage/slsDetectorSoftware/multiSlsDetector/
    slsDetectorPackage/slsDetectorSoftware/slsDetector/
    slsDetectorPackage/slsDetectorSoftware/slsDetectorAnalysis/
    slsDetectorPackage/slsDetectorSoftware/slsReceiverInterface/
    slsDetectorPackage/slsDetectorSoftware/threadFiles/
    slsDetectorPackage/slsReceiverSoftware/include/
)

# --------------------------------------------------------------------------
# Target
# --------------------------------------------------------------------------

add_library(limaslseiger ${sources})

target_include_directories(limaslseiger
    PUBLIC 
        ${includedirs}
)

add_library(SlsDetector STATIC IMPORTED)
set_target_properties(SlsDetector PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/slsDetectorPackage/bin/libSlsDetector.a"
)

add_library(SlsReceiver STATIC IMPORTED)
set_target_properties(SlsReceiver PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/slsDetectorPackage/bin/libSlsReceiver.a"
)

target_link_libraries(limaslseiger 
    PUBLIC
        limacore
        yat::yat
        SlsDetector
        SlsReceiver
)


if (CONAN_LIBCXX STREQUAL "libstdc++")
    target_compile_features(limaslseiger
        PRIVATE
            cxx_std_11
    )
endif()

limatools_set_library_soversion(limaslseiger "VERSION")

# --------------------------------------------------------------------------
# Compile definitions
# --------------------------------------------------------------------------

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" SLSEIGER_VERSION_RAW)
string(STRIP "${SLSEIGER_VERSION_RAW}" SLSEIGER_VERSION)

target_compile_definitions(limaslseiger
    PUBLIC
        SLSEIGER_NAME=SlsEiger
        SLSEIGER_VERSION=${SLSEIGER_VERSION}
        SLSEIGER_ENABLED
)

message(STATUS "Camera enabled: SlsEiger ${SLSEIGER_VERSION}")

# --------------------------------------------------------------------------
# Install
# --------------------------------------------------------------------------

install(
    TARGETS limaslseiger 
    LIBRARY DESTINATION lib
)
