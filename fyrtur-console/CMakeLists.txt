idf_component_register(SRCS "console.c" 
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES console esp-si7021 esp-wifi-manager
                    )