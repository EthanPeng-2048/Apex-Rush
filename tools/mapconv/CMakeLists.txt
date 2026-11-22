cmake_minimum_required(VERSION 3.28)
project(mapconv LANGUAGES CXX)

# CSV map directory -> <name>.rushmap (KVRec). Built only when
# APEX_BUILD_TOOLS=ON (debug-tools preset; never in release).
add_executable(mapconv main.cpp)
target_link_libraries(mapconv PRIVATE rush::engine)
target_compile_features(mapconv PRIVATE cxx_std_23)
