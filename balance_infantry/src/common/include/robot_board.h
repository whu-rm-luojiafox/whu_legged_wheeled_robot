#ifndef ROBOT_BOARD_H
#define ROBOT_BOARD_H

/* CMake chooses one source manifest and defines its board macro. */
#if defined(ROBOT_BOARD_CHASSIS) && defined(ROBOT_BOARD_GIMBAL)
#error "Select exactly one robot board"
#endif
#if !defined(ROBOT_BOARD_CHASSIS) && !defined(ROBOT_BOARD_GIMBAL)
#error "Missing robot board; configure with the chassis or gimbal preset"
#endif

#endif
