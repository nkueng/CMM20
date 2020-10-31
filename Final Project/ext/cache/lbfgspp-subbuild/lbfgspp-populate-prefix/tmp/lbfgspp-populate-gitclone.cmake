
if(NOT "/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/lbfgspp-subbuild/lbfgspp-populate-prefix/src/lbfgspp-populate-stamp/lbfgspp-populate-gitinfo.txt" IS_NEWER_THAN "/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/lbfgspp-subbuild/lbfgspp-populate-prefix/src/lbfgspp-populate-stamp/lbfgspp-populate-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/lbfgspp-subbuild/lbfgspp-populate-prefix/src/lbfgspp-populate-stamp/lbfgspp-populate-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E remove_directory "/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/lbfgspp-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/lbfgspp-src'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/local/bin/git"  clone --no-checkout "https://github.com/yixuan/LBFGSpp" "lbfgspp-src"
    WORKING_DIRECTORY "/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/yixuan/LBFGSpp'")
endif()

execute_process(
  COMMAND "/usr/local/bin/git"  checkout f047ef4586869855f00e72312e7b4d78d11694b1 --
  WORKING_DIRECTORY "/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/lbfgspp-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'f047ef4586869855f00e72312e7b4d78d11694b1'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/local/bin/git"  submodule update --recursive --init 
    WORKING_DIRECTORY "/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/lbfgspp-src"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/lbfgspp-src'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/lbfgspp-subbuild/lbfgspp-populate-prefix/src/lbfgspp-populate-stamp/lbfgspp-populate-gitinfo.txt"
    "/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/lbfgspp-subbuild/lbfgspp-populate-prefix/src/lbfgspp-populate-stamp/lbfgspp-populate-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/Users/Nicola/Documents/Code/CMM20/final-project-gradedescent/ext/cache/lbfgspp-subbuild/lbfgspp-populate-prefix/src/lbfgspp-populate-stamp/lbfgspp-populate-gitclone-lastrun.txt'")
endif()

