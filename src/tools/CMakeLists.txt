# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


macro(IDYNTREE_ADD_TOOL tool_name tool_code)
    add_executable(${tool_name} ${tool_code})
    target_link_libraries(${tool_name} PRIVATE idyntree-core
                                               idyntree-model
                                               idyntree-high-level
                                               idyntree-modelio
                                               idyntree-visualization
                                               Eigen3::Eigen)
    target_compile_options(${tool_name} PRIVATE ${IDYNTREE_WARNING_FLAGS})
    if(TARGET idyntree-solid-shapes)
        target_link_libraries(${tool_name} PRIVATE idyntree-solid-shapes)
    endif()
    install(TARGETS ${tool_name} DESTINATION bin)
endmacro(IDYNTREE_ADD_TOOL tool_name tool_code)

idyntree_add_tool(idyntree-model-info idyntree-model-info.cpp)
idyntree_add_tool(idyntree-model-convert idyntree-model-convert.cpp)

if(IDYNTREE_USES_IRRLICHT)
    idyntree_add_tool(idyntree-model-view idyntree-model-view.cpp)
endif()

if(IDYNTREE_USES_ASSIMP)
    idyntree_add_tool(idyntree-model-simplify-shapes idyntree-model-simplify-shapes.cpp)
endif()
