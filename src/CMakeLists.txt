cmake_minimum_required(VERSION 3.2)
project(client)

# FIXME: temporary until there is a better depdency management strategy
include_directories("${SPLIB_SOURCE_DIR}/include")
include_directories("${SPLIB_SOURCE_DIR}/source")

# FIXME: the madronalib install doesn't include all the dependencies referenced
# in the installed headers, such as sse2neon. Instead of fixing the install of
# the static library and headers for now the code is directly added as a subdir
# and here we add the internal source dirs to find the headers.
include_directories(${MLLIB_SOURCE_DIR}/external/aes256)
# include_directories(${MLLIB_SOURCE_DIR}/external/cJSON)
include_directories(${MLLIB_SOURCE_DIR}/external/ffft)
include_directories(${MLLIB_SOURCE_DIR}/external/utf)
include_directories(${MLLIB_SOURCE_DIR}/external/sse2neon)
include_directories(${MLLIB_SOURCE_DIR}/external/oscpack/ip)
include_directories(${MLLIB_SOURCE_DIR}/external/oscpack/osc)

include_directories(${MLLIB_SOURCE_DIR}/source/app)
include_directories(${MLLIB_SOURCE_DIR}/source/DSP)
include_directories(${MLLIB_SOURCE_DIR}/source/matrix)
include_directories(${MLLIB_SOURCE_DIR}/source/networking)
include_directories(${MLLIB_SOURCE_DIR}/source/procs)

include_directories(${MLLIB_SOURCE_DIR}/include)

set(SOURCE_FILES
  # ml altered
  MLDSPDeprecated.cpp
  MLVectorDeprecated.cpp
  MLModel.cpp
  MLParameter.cpp
  MLPropertySet.cpp
  # sp altered
  TouchTracker.cpp
  Zone.cpp
  # new
  ConsoleOutput.cpp
  Logging.cpp
  Client.cpp
  main.cpp
)

# add_library(soundplaneclient ${SOURCE_FILES})
add_executable(soundplaneclient ${SOURCE_FILES})
target_link_libraries(soundplaneclient madronalib soundplanelib)
target_link_libraries(soundplaneclient pthread)
