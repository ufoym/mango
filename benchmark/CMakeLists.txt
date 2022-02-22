project(benchmark)

add_compile_options(-Wall -O3 -g)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    add_compile_options(-mavx)
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "(arm)")
    if (!APPLE)
        # The compiler for Apple Mx CPUs don't recognize these
        add_definitions (-mfpu=neon -mfloat-abi=hard)
    endif()
    set(CMAKE_CXX_FLAGS "-fpermissive -Wno-psabi")
endif ()

link_libraries(mango)

# add "-lpthread" linker option when platform has support
if (UNIX)
    set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
    find_package(Threads REQUIRED)
    if (CMAKE_USE_PTHREADS_INIT)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    endif ()
    link_libraries("${CMAKE_THREAD_LIBS_INIT}")
endif ()

if (APPLE)
    # fix wrapped compiler dropping /usr/local
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/local/include")
    link_directories(/usr/local/lib)

    if (CMAKE_SYSTEM_PROCESSOR MATCHES "(arm)")
        # fix brew paths on M1 silicon
        include_directories("/opt/homebrew/include")
        include_directories("/opt/homebrew/opt/jpeg-turbo/include ")
        link_directories("/opt/homebrew/lib")
        link_directories("/opt/homebrew/opt/jpeg-turbo/lib")
    endif ()
else ()
    link_libraries(dl)
endif ()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_executable(benchmark
    jpeg_benchmark.cpp
    jpeg-compressor/jpgd.cpp
    jpeg-compressor/jpge.cpp
)
target_link_libraries(benchmark jpeg)
