add_subdirectory_if_exist(aht_20)
add_subdirectory_if_exist(soft_uart)

set(SOURCES "${SOURCES}"
"${CMAKE_CURRENT_SOURCE_DIR}/main.c"
 PARENT_SCOPE)
set(PUBLIC_HEADER "${PUBLIC_HEADER}" PARENT_SCOPE)