SET(SOURCES ui_comp_gauge.c
    ui_comp.c
    ui_StartupScr.c
    ui_SingleScr.c
    ui_SettingsScr.c
    ui_DashDisplay.c
    ui_theme_manager.c
    ui_themes.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    ui_font_GilBig.c
    ui_font_GilMedium.c
    ui_font_GilSm.c
    ui_font_SeveSegLg.c
    ui_font_SeveSegSm.c
    ui_font_SevenSegLbl.c
    ui_font_SevenSegMd.c)

add_library(ui ${SOURCES})
