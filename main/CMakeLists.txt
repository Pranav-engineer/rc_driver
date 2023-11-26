idf_component_register(SRCS "ps4Handler.cpp" "speedMapper.cpp" "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES ps4-idf micro_ros_espidf_component qmd)
