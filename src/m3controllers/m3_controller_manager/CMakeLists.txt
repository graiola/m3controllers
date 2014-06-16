set(BINNAME "m3_controller_manager")

###############################################################################
# Create the binary
add_executable(${BINNAME} ${BINNAME}.cpp)

###############################################################################
# Add dependency to m3_controllers
add_dependencies(${BINNAME} m3controllers)

###############################################################################
# Linking
# Note: Dl libraries needed (for the dynamic linking)
target_link_libraries(${BINNAME} ${LIBRARIES} ${CMAKE_DL_LIBS} m3controllers)

###############################################################################
# Install the binary
#install(TARGETS ${BINNAME} DESTINATION ${BIN_INSTALL_DIR})
