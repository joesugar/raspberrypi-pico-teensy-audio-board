target_sources(i2s_example PRIVATE
    i2s_example.cpp
    i2s.cpp 
    sgtl5000.cpp    
)

pico_generate_pio_header(i2s_example ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)
