set(LIBNAME "sin_controller")

###############################################################################
# Create the library
add_library(${LIBNAME} ${SHARED_OR_STATIC} ${LIBNAME}.cpp)

###############################################################################
# Add dependency to m3_controller_interface
add_dependencies(${LIBNAME} m3_controller_interface)

###############################################################################
# Linking
target_link_libraries(${LIBNAME} ${LIBRARIES})

###############################################################################
# Export libname
set(LIBNAME_LIST ${LIBNAME_LIST} ${LIBNAME} PARENT_SCOPE)

###############################################################################
# Insert the controller name into the factory
set(FACTORY_DEFINITIONS ${FACTORY_DEFINITIONS} SIN_CONTROLLER_NAME=\"${LIBNAME}\" PARENT_SCOPE)
