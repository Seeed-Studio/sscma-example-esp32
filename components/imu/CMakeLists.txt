set(src_dirs src)

set(include_dirs src)

set(requires bus esp_timer driver)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3)

