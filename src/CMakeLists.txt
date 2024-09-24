cmake_minimum_required(VERSION 3.2)
project(client)

#
# [lib] libsoundplaneclient
#

set(LIB_SOURCE_FILES
  # ml altered
  MLDSPDeprecated.cpp
  MLVectorDeprecated.cpp
  MLModel.cpp
  MLParameter.cpp
  MLPropertySet.cpp
  # sp altered
  TouchTracker.cpp
  ZoneSpec.cpp
  Zone.cpp
  # new
  ConsoleOutput.cpp
  Logging.cpp
  Client.cpp
  main.cpp
)

add_library(soundplaneclient STATIC ${LIB_SOURCE_FILES})
set_target_properties(soundplaneclient PROPERTIES
                      POSITION_INDEPENDENT_CODE ON)


target_include_directories(soundplaneclient PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# FIXME: temporary until there is a better depdency management strategy
target_include_directories(soundplaneclient PUBLIC "${SPLIB_SOURCE_DIR}/include")
target_include_directories(soundplaneclient PUBLIC "${SPLIB_SOURCE_DIR}/source")

# FIXME: the madronalib install doesn't include all the dependencies referenced
# in the installed headers, such as sse2neon. Instead of fixing the install of
# the static library and headers for now the code is directly added as a subdir
# and here we add the internal source dirs to find the headers.
target_include_directories(soundplaneclient PUBLIC ${MLLIB_SOURCE_DIR}/external/aes256)
target_include_directories(soundplaneclient PUBLIC ${MLLIB_SOURCE_DIR}/external/cJSON)
target_include_directories(soundplaneclient PUBLIC ${MLLIB_SOURCE_DIR}/external/ffft)
target_include_directories(soundplaneclient PUBLIC ${MLLIB_SOURCE_DIR}/external/utf)
target_include_directories(soundplaneclient PUBLIC ${MLLIB_SOURCE_DIR}/external/sse2neon)
target_include_directories(soundplaneclient PUBLIC ${MLLIB_SOURCE_DIR}/external/oscpack/ip)
target_include_directories(soundplaneclient PUBLIC ${MLLIB_SOURCE_DIR}/external/oscpack/osc)

target_include_directories(soundplaneclient PUBLIC ${MLLIB_SOURCE_DIR}/source/app)
target_include_directories(soundplaneclient PUBLIC ${MLLIB_SOURCE_DIR}/source/DSP)
target_include_directories(soundplaneclient PUBLIC ${MLLIB_SOURCE_DIR}/source/matrix)
target_include_directories(soundplaneclient PUBLIC ${MLLIB_SOURCE_DIR}/source/networking)
target_include_directories(soundplaneclient PUBLIC ${MLLIB_SOURCE_DIR}/source/procs)

target_include_directories(soundplaneclient PUBLIC ${MLLIB_SOURCE_DIR}/include)


target_link_libraries(soundplaneclient madronalib soundplanelib pthread)

#
# [cli] soundplane
#

set(CLI_SOURCE_FILES
  main.cpp
)

add_executable(soundplane ${CLI_SOURCE_FILES})
target_link_libraries(soundplane soundplaneclient)
