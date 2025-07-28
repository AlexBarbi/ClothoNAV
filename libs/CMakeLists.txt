message("[LIBS]")

set(LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DIR}/bin/libs)
set(LIBS_DIR
    ${LIBS_DIR}
    PARENT_SCOPE)

include_directories(${EXTERNAL_DIR} ${LIBS_DIR})

add_library(CsvParser ${LIBS_DIR}/CsvParser/CsvParser.cpp)
target_link_libraries(CsvParser geographic raylib)
# target_include_directories(CsvParser PUBLIC ${LIBS_DIR}/CsvParser)