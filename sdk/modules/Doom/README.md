# DOOM Source Code (linuxdoom-1.10)

This directory contains the original DOOM source code from id Software, released December 23, 1997. This is the actual codebase that powered one of the most influential games in computing history.

## Historical Context

This source code represents several important firsts in gaming:
- Popularized the first-person shooter genre
- Introduced WAD files, enabling widespread game modding
- Demonstrated efficient 3D rendering on limited hardware
- Pioneered networked multiplayer gaming

The code was written primarily by John Carmack and the id Software team for release in 1993.

## Major Architectural Components

### 1. Platform Abstraction Layer (I_* files)
- **`i_video.c`** - X11/Linux graphics implementation, handles windowing, input events, and screen rendering
- **`i_sound.c`** - Audio system interface 
- **`i_system.c`** - System-specific functions (memory, file I/O, timing)
- **`i_net.c`** - Network communication for multiplayer
- **`i_main.c`** - Platform entry point

### 2. Core Game Engine (D_* files)
- **`d_main.c`** - Main game loop, initialization, demo system
- **`d_net.c`** - Network game synchronization
- **`d_items.c`** - Game item definitions

### 3. Rendering Engine (R_* files)
- **`r_main.c`** - Main renderer, view setup, BSP traversal
- **`r_bsp.c`** - Binary Space Partitioning for visibility determination
- **`r_draw.c`** - Low-level drawing functions (columns, spans)
- **`r_plane.c`** - Floor and ceiling rendering
- **`r_segs.c`** - Wall segment rendering
- **`r_things.c`** - Sprite rendering (enemies, items, decorations)
- **`r_data.c`** - Texture and graphics data management

### 4. Game Logic & Physics (P_* files)
- **`p_setup.c`** - Level loading, WAD file parsing, map data structures
- **`p_mobj.c`** - Moving objects (players, monsters, projectiles)
- **`p_map.c`** - Collision detection and movement
- **`p_enemy.c`** - AI behavior for monsters
- **`p_user.c`** - Player movement and actions
- **`p_spec.c`** - Special effects (doors, lifts, triggers)
- **`p_sight.c`** - Line-of-sight calculations

### 5. User Interface (M_*, HU_*, ST_* files)
- **`m_menu.c`** - Menu system
- **`hu_stuff.c`** - Heads-up display, chat messages
- **`st_stuff.c`** - Status bar (health, ammo, etc.)

### 6. Audio System (S_* files)
- **`s_sound.c`** - Sound effects and music management
- **`sounds.c`** - Sound definitions and lookup tables

### 7. Utilities & Support (M_*, W_*, Z_* files)
- **`w_wad.c`** - WAD file format handling (game data archives)
- **`z_zone.c`** - Memory management system
- **`m_fixed.c`** - Fixed-point arithmetic (no floating point used!)
- **`tables.c`** - Precomputed trigonometric lookup tables

## Technical Innovations

1. **Fixed-Point Mathematics** - Used integer arithmetic instead of floating point for performance
2. **BSP Trees** - Binary space partitioning for efficient visibility culling
3. **WAD File System** - Modular data files that enabled easy content modification
4. **Memory Management** - Custom zone-based allocators
5. **Networking** - Real-time multiplayer over serial and network connections

## File Naming Convention

- **`i_`** - Implementation/Platform layer
- **`d_`** - Doom main engine
- **`r_`** - Renderer
- **`p_`** - Play/Physics/Game logic
- **`m_`** - Math/Menu/Misc utilities
- **`w_`** - WAD file system
- **`s_`** - Sound
- **`g_`** - Game state
- **`f_`** - Finale/Intermission screens

## Building the Code

The original Makefile is included in the `linuxdoom-1.10` directory. To build:

```bash
cd DOOM/linuxdoom-1.10
make
```

This will create the `linuxxdoom` executable in the `linux/` subdirectory.

## Historical Significance

This codebase represents a significant milestone in software development:

- Released December 10, 1993, establishing many conventions for 3D games
- Source code made available December 23, 1997
- Influenced numerous subsequent game engines and development practices
- Created a large modding community that continues today
- Demonstrates effective techniques for real-time 3D rendering on 1990s hardware

The code serves as an excellent study in game engine architecture and optimization techniques for resource-constrained environments.

## License

This source code is available under the DOOM Source Code License as published by id Software. See the individual source files for full license details.
