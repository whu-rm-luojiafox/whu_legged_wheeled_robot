//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_round_t *ui_g_DynamicGroup_FricRound;
extern ui_interface_round_t *ui_g_DynamicGroup_AutoRound;
extern ui_interface_arc_t *ui_g_DynamicGroup_DirectionArc;
extern ui_interface_number_t *ui_g_DynamicGroup_HightNum;
extern ui_interface_line_t *ui_g_DynamicGroup_JumpLine;
extern ui_interface_number_t *ui_g_DynamicGroup_FricNum;
extern ui_interface_round_t *ui_g_DynamicGroup_ResvRound;

void ui_init_g_DynamicGroup();
void ui_update_g_DynamicGroup();
void ui_remove_g_DynamicGroup();

extern ui_interface_rect_t *ui_g_StaticGraphicGroup_SelfaimRect;
extern ui_interface_rect_t *ui_g_StaticGraphicGroup_HeatRect;
extern ui_interface_line_t *ui_g_StaticGraphicGroup_CrosshairLine_1;
extern ui_interface_line_t *ui_g_StaticGraphicGroup_GuideLine_1;
extern ui_interface_line_t *ui_g_StaticGraphicGroup_CrosshairLine_2;
extern ui_interface_line_t *ui_g_StaticGraphicGroup_CrosshairLine_3;
extern ui_interface_line_t *ui_g_StaticGraphicGroup_GuideLine_2;

void ui_init_g_StaticGraphicGroup();
void ui_update_g_StaticGraphicGroup();
void ui_remove_g_StaticGraphicGroup();

extern ui_interface_string_t *ui_g_StaticTextGroup_FricText;
extern ui_interface_string_t *ui_g_StaticTextGroup_AutoText;
extern ui_interface_string_t *ui_g_StaticTextGroup_FricSpeedText;
extern ui_interface_string_t *ui_g_StaticTextGroup_DirectText;
extern ui_interface_string_t *ui_g_StaticTextGroup_HightText;

void ui_init_g_StaticTextGroup();
void ui_update_g_StaticTextGroup();
void ui_remove_g_StaticTextGroup();


#endif // UI_g_H
