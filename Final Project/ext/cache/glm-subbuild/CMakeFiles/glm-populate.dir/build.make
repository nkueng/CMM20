# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.16.4/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.16.4/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild

# Utility rule file for glm-populate.

# Include the progress variables for this target.
include CMakeFiles/glm-populate.dir/progress.make

CMakeFiles/glm-populate: CMakeFiles/glm-populate-complete


CMakeFiles/glm-populate-complete: glm-populate-prefix/src/glm-populate-stamp/glm-populate-install
CMakeFiles/glm-populate-complete: glm-populate-prefix/src/glm-populate-stamp/glm-populate-mkdir
CMakeFiles/glm-populate-complete: glm-populate-prefix/src/glm-populate-stamp/glm-populate-download
CMakeFiles/glm-populate-complete: glm-populate-prefix/src/glm-populate-stamp/glm-populate-update
CMakeFiles/glm-populate-complete: glm-populate-prefix/src/glm-populate-stamp/glm-populate-patch
CMakeFiles/glm-populate-complete: glm-populate-prefix/src/glm-populate-stamp/glm-populate-configure
CMakeFiles/glm-populate-complete: glm-populate-prefix/src/glm-populate-stamp/glm-populate-build
CMakeFiles/glm-populate-complete: glm-populate-prefix/src/glm-populate-stamp/glm-populate-install
CMakeFiles/glm-populate-complete: glm-populate-prefix/src/glm-populate-stamp/glm-populate-test
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'glm-populate'"
	/usr/local/Cellar/cmake/3.16.4/bin/cmake -E make_directory /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/CMakeFiles
	/usr/local/Cellar/cmake/3.16.4/bin/cmake -E touch /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/CMakeFiles/glm-populate-complete
	/usr/local/Cellar/cmake/3.16.4/bin/cmake -E touch /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix/src/glm-populate-stamp/glm-populate-done

glm-populate-prefix/src/glm-populate-stamp/glm-populate-install: glm-populate-prefix/src/glm-populate-stamp/glm-populate-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "No install step for 'glm-populate'"
	cd /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/build/_deps/glm-build && /usr/local/Cellar/cmake/3.16.4/bin/cmake -E echo_append
	cd /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/build/_deps/glm-build && /usr/local/Cellar/cmake/3.16.4/bin/cmake -E touch /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix/src/glm-populate-stamp/glm-populate-install

glm-populate-prefix/src/glm-populate-stamp/glm-populate-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'glm-populate'"
	/usr/local/Cellar/cmake/3.16.4/bin/cmake -E make_directory /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-src
	/usr/local/Cellar/cmake/3.16.4/bin/cmake -E make_directory /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/build/_deps/glm-build
	/usr/local/Cellar/cmake/3.16.4/bin/cmake -E make_directory /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix
	/usr/local/Cellar/cmake/3.16.4/bin/cmake -E make_directory /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix/tmp
	/usr/local/Cellar/cmake/3.16.4/bin/cmake -E make_directory /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix/src/glm-populate-stamp
	/usr/local/Cellar/cmake/3.16.4/bin/cmake -E make_directory /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix/src
	/usr/local/Cellar/cmake/3.16.4/bin/cmake -E make_directory /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix/src/glm-populate-stamp
	/usr/local/Cellar/cmake/3.16.4/bin/cmake -E touch /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix/src/glm-populate-stamp/glm-populate-mkdir

glm-populate-prefix/src/glm-populate-stamp/glm-populate-download: glm-populate-prefix/src/glm-populate-stamp/glm-populate-urlinfo.txt
glm-populate-prefix/src/glm-populate-stamp/glm-populate-download: glm-populate-prefix/src/glm-populate-stamp/glm-populate-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (download, verify and extract) for 'glm-populate'"
	cd /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache && /usr/local/Cellar/cmake/3.16.4/bin/cmake -P /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix/src/glm-populate-stamp/download-glm-populate.cmake
	cd /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache && /usr/local/Cellar/cmake/3.16.4/bin/cmake -P /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix/src/glm-populate-stamp/verify-glm-populate.cmake
	cd /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache && /usr/local/Cellar/cmake/3.16.4/bin/cmake -P /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix/src/glm-populate-stamp/extract-glm-populate.cmake
	cd /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache && /usr/local/Cellar/cmake/3.16.4/bin/cmake -E touch /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix/src/glm-populate-stamp/glm-populate-download

glm-populate-prefix/src/glm-populate-stamp/glm-populate-update: glm-populate-prefix/src/glm-populate-stamp/glm-populate-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No update step for 'glm-populate'"
	/usr/local/Cellar/cmake/3.16.4/bin/cmake -E echo_append
	/usr/local/Cellar/cmake/3.16.4/bin/cmake -E touch /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix/src/glm-populate-stamp/glm-populate-update

glm-populate-prefix/src/glm-populate-stamp/glm-populate-patch: glm-populate-prefix/src/glm-populate-stamp/glm-populate-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "No patch step for 'glm-populate'"
	/usr/local/Cellar/cmake/3.16.4/bin/cmake -E echo_append
	/usr/local/Cellar/cmake/3.16.4/bin/cmake -E touch /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix/src/glm-populate-stamp/glm-populate-patch

glm-populate-prefix/src/glm-populate-stamp/glm-populate-configure: glm-populate-prefix/tmp/glm-populate-cfgcmd.txt
glm-populate-prefix/src/glm-populate-stamp/glm-populate-configure: glm-populate-prefix/src/glm-populate-stamp/glm-populate-update
glm-populate-prefix/src/glm-populate-stamp/glm-populate-configure: glm-populate-prefix/src/glm-populate-stamp/glm-populate-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "No configure step for 'glm-populate'"
	cd /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/build/_deps/glm-build && /usr/local/Cellar/cmake/3.16.4/bin/cmake -E echo_append
	cd /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/build/_deps/glm-build && /usr/local/Cellar/cmake/3.16.4/bin/cmake -E touch /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix/src/glm-populate-stamp/glm-populate-configure

glm-populate-prefix/src/glm-populate-stamp/glm-populate-build: glm-populate-prefix/src/glm-populate-stamp/glm-populate-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "No build step for 'glm-populate'"
	cd /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/build/_deps/glm-build && /usr/local/Cellar/cmake/3.16.4/bin/cmake -E echo_append
	cd /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/build/_deps/glm-build && /usr/local/Cellar/cmake/3.16.4/bin/cmake -E touch /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix/src/glm-populate-stamp/glm-populate-build

glm-populate-prefix/src/glm-populate-stamp/glm-populate-test: glm-populate-prefix/src/glm-populate-stamp/glm-populate-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "No test step for 'glm-populate'"
	cd /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/build/_deps/glm-build && /usr/local/Cellar/cmake/3.16.4/bin/cmake -E echo_append
	cd /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/build/_deps/glm-build && /usr/local/Cellar/cmake/3.16.4/bin/cmake -E touch /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/glm-populate-prefix/src/glm-populate-stamp/glm-populate-test

glm-populate: CMakeFiles/glm-populate
glm-populate: CMakeFiles/glm-populate-complete
glm-populate: glm-populate-prefix/src/glm-populate-stamp/glm-populate-install
glm-populate: glm-populate-prefix/src/glm-populate-stamp/glm-populate-mkdir
glm-populate: glm-populate-prefix/src/glm-populate-stamp/glm-populate-download
glm-populate: glm-populate-prefix/src/glm-populate-stamp/glm-populate-update
glm-populate: glm-populate-prefix/src/glm-populate-stamp/glm-populate-patch
glm-populate: glm-populate-prefix/src/glm-populate-stamp/glm-populate-configure
glm-populate: glm-populate-prefix/src/glm-populate-stamp/glm-populate-build
glm-populate: glm-populate-prefix/src/glm-populate-stamp/glm-populate-test
glm-populate: CMakeFiles/glm-populate.dir/build.make

.PHONY : glm-populate

# Rule to build all files generated by this target.
CMakeFiles/glm-populate.dir/build: glm-populate

.PHONY : CMakeFiles/glm-populate.dir/build

CMakeFiles/glm-populate.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/glm-populate.dir/cmake_clean.cmake
.PHONY : CMakeFiles/glm-populate.dir/clean

CMakeFiles/glm-populate.dir/depend:
	cd /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild /Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/glm-subbuild/CMakeFiles/glm-populate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/glm-populate.dir/depend

