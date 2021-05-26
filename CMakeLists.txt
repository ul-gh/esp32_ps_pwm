# Requires only ESP-IDF
#set(requires
#)

set(sources
    "ps_pwm.c"
)

idf_component_register(SRCS "${sources}"
                       INCLUDE_DIRS "."
                       #REQUIRES "${requires}"
)
