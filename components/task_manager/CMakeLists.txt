set(COMPONENT_SRCS "task_manager.c")

set(COMPONENT_REQUIRES io_define isr_manager)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
