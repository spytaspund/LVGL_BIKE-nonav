SET(SOURCES screens/ui_MainApp.c
    screens/ui_RoadApp.c
    screens/ui_SettingsApp.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    fonts/ui_font_seven_segment.c)

add_library(ui ${SOURCES})
