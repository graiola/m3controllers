set(LIBNAME "cart_controller")

###############################################################################
# Protobuff stuff
file(GLOB ControllerProtoFiles "${PROJECT_PROTO_DIR}/${PROJECT_NAME}/${LIBNAME}/*.proto")
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${CMAKE_CURRENT_BINARY_DIR} ${ControllerProtoFiles})
add_custom_target(${LIBNAME}_cpp_protobuf_output  ALL DEPENDS ${ProtoSources}) # force the generation
PROTOBUF_GENERATE_PYTHON(PySources ${PROJECT_PYTHON_DIR}/${PROJECT_NAME}/${LIBNAME} ${ControllerProtoFiles})
add_custom_target(${LIBNAME}_python_protobuf_output ALL DEPENDS ${PySources}) # force the generation
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_PYTHON_DIR}/${PROJECT_NAME}/${LIBNAME}) # To create a directory at build time

###############################################################################
# Create the library
add_library(${LIBNAME} ${SHARED_OR_STATIC} ${LIBNAME}.cpp ${LIBNAME}.pb.cc)

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
set(FACTORY_DEFINITIONS ${FACTORY_DEFINITIONS} CART_CONTROLLER_NAME=\"${LIBNAME}\" PARENT_SCOPE)

###############################################################################
# Create the binary
add_executable(cart_controller_listener cart_controller_listener.cpp)

###############################################################################
# Linking
# Note: Dl libraries needed (for the dynamic linking)
target_link_libraries(cart_controller_listener ${LIBRARIES} ${CMAKE_DL_LIBS} m3controllers)
