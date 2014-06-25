set(LIBNAME "vf_force_controller")

###############################################################################
# Create the library
add_library(${LIBNAME} ${SHARED_OR_STATIC} ${LIBNAME}.cpp)

###############################################################################
# Add dependency to m3_controller_interface
add_dependencies(${LIBNAME} cart_controller m3_controller_interface)

###############################################################################
# Export libname
set(LIBNAME_LIST ${LIBNAME_LIST} ${LIBNAME} PARENT_SCOPE)

###############################################################################
# Insert the controller name into the factory
set(FACTORY_DEFINITIONS ${FACTORY_DEFINITIONS} VF_FORCE_CONTROLLER_NAME=\"${LIBNAME}\" PARENT_SCOPE)
