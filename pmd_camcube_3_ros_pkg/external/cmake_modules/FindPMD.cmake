# - Try to find PMD headers and plugins
# Once done this will define
#
#  PMD_INCLUDE_DIR - if PMD Header Include Directory is found
#
# The configuration directory PMD_PLUGIN_DIR will always be
# set, either to a directory containing the "camcube3.L32.pap" file,
# or if no such file is found, to the local folder ".". In the last
# case a warning message is  issued.
#

FIND_PATH(PMD_INCLUDE_DIR NAMES pmdsdk2.h
  PATHS
  $ENV{PMDDIR}/include/
  ENV CPATH
  /usr/include/pmd/include/
  /usr/local/include/pmd/include/
  /opt/local/include/pmd/include/
  /usr/local/lib/
  NO_DEFAULT_PATH
)

FIND_PATH(PMD_PLUGIN_DIR NAMES camcube3.pap
  PATHS
  $ENV{PMDDIR}/plugins/
  /usr/include/pmd/plugins/
  /usr/local/include/pmd/plugins/
  /opt/local/include/pmd/plugins/
  /usr/local/lib/
  NO_DEFAULT_PATH
)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set OODL_YOUBOT_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(PMD_INCLUDE DEFAULT_MSG PMD_INCLUDE_DIR)

find_package_handle_standard_args(PMD_PLUGIN DEFAULT_MSG PMD_PLUGIN_DIR)


# show warning that no include directory was found; set it to "./" by default
IF (NOT PMD_INCLUDE_FOUND)
 MESSAGE(STATUS "Warning: pmd include directory not found, using default folder \".\"")
 SET(PMD_INCLUDE_DIR .)
ENDIF (NOT PMD_INCLUDE_FOUND)

# show warning that no include directory was found; set it to "./" by default
IF (NOT PMD_PLUGIN_FOUND)
 MESSAGE(STATUS "Warning: pmd plugins directory not found, using default folder \".\"")
 SET(PMD_PLUGIN_DIR .)
ENDIF (NOT PMD_PLUGIN_FOUND)

