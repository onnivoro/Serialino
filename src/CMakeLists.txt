find_package( Threads REQUIRED )

# Create a library called "Serialino" which includes the source file "Serialino.cpp".
# The extension is already found. Any number of sources could be listed here.
add_library (Serialino Serialino.cpp)

target_link_libraries( Serialino ${CMAKE_THREAD_LIBS_INIT} )

# Make sure the compiler can find include files for our Hello library
# when other libraries or executables link to Hello
target_include_directories (Serialino PUBLIC ${PROJECT_SOURCE_DIR}/header)

# Add executable called "helloDemo" that is built from the source files
# "demo.cxx" and "demo_b.cxx". The extensions are automatically found.
add_executable (SerialinoTest Main.cpp)

# Link the executable to the Hello library. Since the Hello library has
# public include directories we will use those link directories when building
# helloDemo
target_link_libraries (SerialinoTest LINK_PUBLIC Serialino)


