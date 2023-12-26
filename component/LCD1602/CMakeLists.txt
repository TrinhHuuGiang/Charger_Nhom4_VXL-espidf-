set(app_src LCD1602.C)
set(pri_req)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pri_req})

                        