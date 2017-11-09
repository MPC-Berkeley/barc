if (_ROSLINT_EXTRAS_INCLUDED_)
  return()
endif()
set(_ROSLINT_EXTRAS_INCLUDED_ TRUE)

@[if INSTALLSPACE]@
# bin and template dir variables in installspace
set(ROSLINT_SCRIPTS_DIR "${roslint_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)")
@[else]@
# bin and template dir variables in develspace
set(ROSLINT_SCRIPTS_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/scripts")
@[end if]@

macro(_roslint_create_targets)
  # Create the master "roslint" target if it doesn't exist yet.
  if (NOT TARGET roslint)
    add_custom_target(roslint)
  endif()

  # Create the "roslint_pkgname" target if it doesn't exist yet. Doing this
  # with a check means that multiple linters can share the same target.
  if (NOT TARGET roslint_${PROJECT_NAME})
    add_custom_target(roslint_${PROJECT_NAME})
    add_dependencies(roslint roslint_${PROJECT_NAME})
  endif()
endmacro()

# Run a custom lint command on a list of file names.
#
# :param linter: linter command name.
# :param lintopts: linter options.
# :param argn: a non-empty list of files to process.
# :type string
#
function(roslint_custom linter lintopts)
  if ("${ARGN}" STREQUAL "")
    message(WARNING "roslint: no files provided for command")
  else ()
    _roslint_create_targets()
    add_custom_command(TARGET roslint_${PROJECT_NAME} POST_BUILD
                       WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                       COMMAND ${linter} ${lintopts} ${ARGN} VERBATIM)
  endif()
endfunction()

# Run cpplint on a list of file names.
#
function(roslint_cpp)
  if ("${ARGN}" STREQUAL "")
    file(GLOB_RECURSE ARGN *.cpp *.h)
  endif()
  if (NOT DEFINED ROSLINT_CPP_CMD)
    set(ROSLINT_CPP_CMD ${ROSLINT_SCRIPTS_DIR}/cpplint)
  endif()
  roslint_custom("${ROSLINT_CPP_CMD}" "${ROSLINT_CPP_OPTS}" ${ARGN})
endfunction()

# Run pep8 on a list of file names.
#
function(roslint_python)
  if ("${ARGN}" STREQUAL "")
    file(GLOB_RECURSE ARGN *.py)
  endif()
  if (NOT DEFINED ROSLINT_PYTHON_CMD)
    set(ROSLINT_PYTHON_CMD ${ROSLINT_SCRIPTS_DIR}/pep8)
  endif()
  roslint_custom("${ROSLINT_PYTHON_CMD}" "${ROSLINT_PYTHON_OPTS}" ${ARGN})
endfunction()

# Run roslint for this package as a test.
function(roslint_add_test)
  catkin_run_tests_target("roslint" "package" "roslint-${PROJECT_NAME}.xml"
    COMMAND "${ROSLINT_SCRIPTS_DIR}/test_wrapper ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/roslint-${PROJECT_NAME}.xml make roslint_${PROJECT_NAME}"
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endfunction()
