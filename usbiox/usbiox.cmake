cmake_minimum_required(VERSION 3.1.0)

if (NOT TARGET usbiox)
    add_library(usbiox STATIC IMPORTED GLOBAL)

    set_target_properties(usbiox PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/include")

    if (x64)
        set(USBIOX_DLL_LIBRARY_DIR ${CMAKE_CURRENT_LIST_DIR}/lib64)
    else()
        set(USBIOX_DLL_LIBRARY_DIR ${CMAKE_CURRENT_LIST_DIR}/lib)
    endif()
    
    set_target_properties(usbiox PROPERTIES
        IMPORTED_LOCATION_DEBUG "${USBIOX_DLL_LIBRARY_DIR}/USBIOX.lib"
        IMPORTED_LOCATION_RELEASE "${USBIOX_DLL_LIBRARY_DIR}/USBIOX.lib")
endif()