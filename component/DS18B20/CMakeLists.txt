set(app_src DS18B20.C)
set(pri_req)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pri_req})

                        