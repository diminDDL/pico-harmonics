# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dmytro/Documents/projects/pico-harmonics/SW

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dmytro/Documents/projects/pico-harmonics/SW/build

# Utility rule file for FinalMessage.

# Include any custom commands dependencies for this target.
include CMakeFiles/FinalMessage.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/FinalMessage.dir/progress.make

CMakeFiles/FinalMessage:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/dmytro/Documents/projects/pico-harmonics/SW/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Done 🥺"
	/usr/bin/cmake -E cmake_echo_color --cyan Compilation\ is\ over!

FinalMessage: CMakeFiles/FinalMessage
FinalMessage: CMakeFiles/FinalMessage.dir/build.make
.PHONY : FinalMessage

# Rule to build all files generated by this target.
CMakeFiles/FinalMessage.dir/build: FinalMessage
.PHONY : CMakeFiles/FinalMessage.dir/build

CMakeFiles/FinalMessage.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FinalMessage.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FinalMessage.dir/clean

CMakeFiles/FinalMessage.dir/depend:
	cd /home/dmytro/Documents/projects/pico-harmonics/SW/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dmytro/Documents/projects/pico-harmonics/SW /home/dmytro/Documents/projects/pico-harmonics/SW /home/dmytro/Documents/projects/pico-harmonics/SW/build /home/dmytro/Documents/projects/pico-harmonics/SW/build /home/dmytro/Documents/projects/pico-harmonics/SW/build/CMakeFiles/FinalMessage.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/FinalMessage.dir/depend

