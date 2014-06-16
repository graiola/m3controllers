set(LIBNAME "m3_controller_interface")

###############################################################################
# Protobuff stuff
file(GLOB ControllerProtoFiles "${PROJECT_PROTO_DIR}/${PROJECT_NAME}/${LIBNAME}/*.proto")
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${CMAKE_CURRENT_BINARY_DIR} ${ControllerProtoFiles})
add_custom_target(${LIBNAME}_cpp_protobuf_output  ALL DEPENDS ${ProtoSources}) # force the generation
PROTOBUF_GENERATE_PYTHON(PySources ${PROJECT_PYTHON_DIR}/${PROJECT_NAME}/${LIBNAME} ${ControllerProtoFiles})
add_custom_target(${LIBNAME}_python_protobuf_output ALL DEPENDS ${PySources}) # force the generation
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_PYTHON_DIR}/${PROJECT_NAME}/${LIBNAME}) # To create a directory at build time

###############################################################################
# Create a fake target
# NOTE: It's necessary because the other controllers depend on m3_controller_interface, so we need to enforce it's "compilation"
#add_custom_target(${LIBNAME} ALL DEPENDS ${LIBNAME}_cpp_protobuf_output ${LIBNAME}_python_protobuf_output)

###############################################################################
# Create the library (Needed by the factory to find the protobuf functions)
add_library(${LIBNAME} ${SHARED_OR_STATIC} ${LIBNAME}.pb.cc)

###############################################################################
# Linking
#target_link_libraries(${LIBNAME} ${LIBRARIES})

###############################################################################
# Export libname
set(LIBNAME_LIST ${LIBNAME_LIST} ${LIBNAME} PARENT_SCOPE)
