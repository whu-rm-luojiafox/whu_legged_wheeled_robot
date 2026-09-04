//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_frame1_H
#define UI_frame1_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_frame1_now_figures[5];
extern uint8_t ui_frame1_dirty_figure[5];
extern ui_interface_string_t ui_frame1_now_strings[4];
extern uint8_t ui_frame1_dirty_string[4];

#define ui_frame1_StaticNumberGroup_NewNumber1 ((ui_interface_number_t*)&(ui_frame1_now_figures[0]))
#define ui_frame1_StaticNumberGroup_NewNumber2 ((ui_interface_number_t*)&(ui_frame1_now_figures[1]))
#define ui_frame1_StaticNumberGroup_NewNumber3 ((ui_interface_number_t*)&(ui_frame1_now_figures[2]))
#define ui_frame1_StaticNumberGroup_NewNumber4 ((ui_interface_number_t*)&(ui_frame1_now_figures[3]))
#define ui_frame1_StaticNumberGroup_NewNumber5 ((ui_interface_number_t*)&(ui_frame1_now_figures[4]))

#define ui_frame1_StaticTextGroup_HeatText (&(ui_frame1_now_strings[0]))
#define ui_frame1_StaticTextGroup_PowerText (&(ui_frame1_now_strings[1]))
#define ui_frame1_Ungroup_FricText (&(ui_frame1_now_strings[2]))
#define ui_frame1_Ungroup_AutoText (&(ui_frame1_now_strings[3]))

#ifdef MANUAL_DIRTY
#define ui_frame1_StaticNumberGroup_NewNumber1_dirty (ui_frame1_dirty_figure[0])
#define ui_frame1_StaticNumberGroup_NewNumber2_dirty (ui_frame1_dirty_figure[1])
#define ui_frame1_StaticNumberGroup_NewNumber3_dirty (ui_frame1_dirty_figure[2])
#define ui_frame1_StaticNumberGroup_NewNumber4_dirty (ui_frame1_dirty_figure[3])
#define ui_frame1_StaticNumberGroup_NewNumber5_dirty (ui_frame1_dirty_figure[4])

#define ui_frame1_StaticTextGroup_HeatText_dirty (ui_frame1_dirty_string[0])
#define ui_frame1_StaticTextGroup_PowerText_dirty (ui_frame1_dirty_string[1])
#define ui_frame1_Ungroup_FricText_dirty (ui_frame1_dirty_string[2])
#define ui_frame1_Ungroup_AutoText_dirty (ui_frame1_dirty_string[3])
#endif

void ui_init_frame1();
void ui_update_frame1();

#endif // UI_frame1_H
