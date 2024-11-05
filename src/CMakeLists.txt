# Subdirectory cmake file for building the example.
#
target_sources(i2s_example PRIVATE
    i2s_example.cpp
    i2s.cpp 
    sgtl5000.cpp    
)

# You have to generate the header file here since the 
# PIO file is contained in this folder.
#
pico_generate_pio_header(i2s_example ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)
