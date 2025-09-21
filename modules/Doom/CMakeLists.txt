
# zephyr_library_named(Doom)

# set_target_properties(Doom PROPERTIES LINKER_LANGUAGE C)

# zephyr_library_compile_definitions(
#     NORMALUNIX
#     __LINUX__
#     SNDSERV=1
# )

# zephyr_library_compile_options(
#     -Wno-unused-function
#     -Wno-unused-variable
#     -Wno-unused-parameter
#     -Wno-maybe-uninitialized
#     -Wno-sign-compare
#     -Wno-implicit-function-declaration
# )

# # Core engine files - minimal set for game loop
# set(DOOM_CORE_SOURCES
#     # Platform abstraction (will need porting)
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/i_system.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/i_sound.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/i_video.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/i_net.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/i_main.c
    
#     # Core definitions and data
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/doomdef.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/doomstat.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/dstrings.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/tables.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/info.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/sounds.c
    
#     # Main game loop
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/d_main.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/d_net.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/d_items.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/g_game.c
    
#     # Math and utilities
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/m_argv.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/m_bbox.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/m_fixed.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/m_swap.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/m_cheat.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/m_random.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/m_menu.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/m_misc.c
    
#     # Memory management
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/z_zone.c
    
#     # WAD file system
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/w_wad.c
    
#     # Renderer - core rendering engine
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/r_main.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/r_bsp.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/r_data.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/r_draw.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/r_plane.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/r_segs.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/r_sky.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/r_things.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/v_video.c
    
#     # Game logic and physics
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_setup.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_mobj.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_map.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_enemy.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_user.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_spec.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_sight.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_maputl.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_tick.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_saveg.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_inter.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_ceilng.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_doors.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_floor.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_lights.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_plats.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_pspr.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_switch.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/p_telept.c
    
#     # UI and HUD (minimal for game loop)
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/st_lib.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/st_stuff.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/hu_stuff.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/hu_lib.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/am_map.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/wi_stuff.c
    
#     # Screen effects
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/f_finale.c
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/f_wipe.c
    
#     # Sound system
#     ${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10/s_sound.c
# )

# zephyr_library_sources(${DOOM_CORE_SOURCES})

# zephyr_library_include_directories(${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10)
# zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/DOOM/linuxdoom-1.10)

# # Note: The i_* files (especially i_video.c, i_sound.c, i_system.c) will need 
# # significant porting to work with Zephyr instead of Linux/X11