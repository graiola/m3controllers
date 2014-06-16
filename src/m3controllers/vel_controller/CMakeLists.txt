set(LIBNAME "torque_controller")

###############################################################################
# Protobuff stuff
#file(GLOB ControllerProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/*.proto" ${M3SysProtoFiles})
#PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${CMAKE_CURRENT_SOURCE_DIR} ${ControllerProtoFiles})

###############################################################################
# Create the library
#add_library(${LIBNAME} ${SHARED_OR_STATIC} ${LIBNAME}.h ${LIBNAME}.pb.h ${LIBNAME}.pb.cc ${LIBNAME}.cpp)
add_library(${LIBNAME} ${SHARED_OR_STATIC} ${LIBNAME}.h ${LIBNAME}.cpp)

###############################################################################
# Add dependency to m3_controller_interface
add_dependencies(${LIBNAME} m3_controller_interface)

###############################################################################
# Linking
target_link_libraries(${LIBNAME} ${LIBRARIES})

###############################################################################
# Install the library and the headers
install(TARGETS ${LIBNAME} DESTINATION ${LIB_INSTALL_DIR}) 
#install(TARGETS ${LIBNAME} DESTINATION ${LIB_INSTALL_DIR_LOCAL})
file(GLOB HEADERS *.h *.hpp)
install(FILES ${HEADERS} DESTINATION ${INCLUDE_INSTALL_DIR}/${LIBNAME})

###############################################################################
# Export libname
set(LIBNAME_LIST ${LIBNAME_LIST} ${LIBNAME} PARENT_SCOPE)
