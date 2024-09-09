if (NOT JPH_SAMPLES)
    return ()
endif ()

function(add_sample SAMPLE_NAME)
    file(GLOB SOURCE_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/${SAMPLE_NAME}/*.h"
	    "${CMAKE_CURRENT_SOURCE_DIR}/${SAMPLE_NAME}/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/${SAMPLE_NAME}/*.cpp"
    )

    if (ANDROID)
        add_library(${SAMPLE_NAME} SHARED ${SOURCE_FILES})
    else ()
    	add_executable(${SAMPLE_NAME} ${SOURCE_FILES})
    endif ()
    target_link_libraries(${SAMPLE_NAME} LINK_PUBLIC joltc)
	if (MSVC)
		target_link_options(${SAMPLE_NAME} PUBLIC "/SUBSYSTEM:CONSOLE")
	endif()

    set_target_properties(${SAMPLE_NAME} PROPERTIES FOLDER "samples")
endfunction()

# Add samples
add_sample(01_HelloWorld)
