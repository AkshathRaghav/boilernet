set(srcs "read_uart.c")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "." 
                       REQUIRES fatfs sd_card
                       WHOLE_ARCHIVE)
