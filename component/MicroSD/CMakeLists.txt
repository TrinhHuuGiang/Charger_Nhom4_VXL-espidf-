set(app_src MicroSD.c)
set(pri_req fatfs)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pri_req})

                        