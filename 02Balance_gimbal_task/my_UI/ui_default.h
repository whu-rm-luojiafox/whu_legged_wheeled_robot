//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_default_H
#define UI_default_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_default_now_figures[16];
extern uint8_t ui_default_dirty_figure[16];

#define ui_default_DynamicBottomGroup_FricRound ((ui_interface_round_t*)&(ui_default_now_figures[0]))
#define ui_default_DynamicBottomGroup_AimRound ((ui_interface_round_t*)&(ui_default_now_figures[1]))
#define ui_default_StaticGroup_PitchArc5 ((ui_interface_arc_t*)&(ui_default_now_figures[2]))
#define ui_default_DynamicRightGroup_DynamicPitchArc ((ui_interface_arc_t*)&(ui_default_now_figures[3]))
#define ui_default_DynamicLeftGroup_ShootHeatArc ((ui_interface_arc_t*)&(ui_default_now_figures[4]))
#define ui_default_DynamicRightGroup_ChassisArc ((ui_interface_arc_t*)&(ui_default_now_figures[5]))
#define ui_default_StaticGroup_CrossLine1 ((ui_interface_line_t*)&(ui_default_now_figures[6]))
#define ui_default_StaticGroup_CrossLine2 ((ui_interface_line_t*)&(ui_default_now_figures[7]))
#define ui_default_DynamicBottomGroup_Round ((ui_interface_round_t*)&(ui_default_now_figures[8]))
#define ui_default_StaticGroup_GroundLineLeft ((ui_interface_line_t*)&(ui_default_now_figures[9]))
#define ui_default_StaticGroup_GroundLineRight ((ui_interface_line_t*)&(ui_default_now_figures[10]))
#define ui_default_DynamicLeftGroup_PowerArc ((ui_interface_arc_t*)&(ui_default_now_figures[11]))
#define ui_default_StaticGroup_PitchArc1 ((ui_interface_arc_t*)&(ui_default_now_figures[12]))
#define ui_default_StaticGroup_PitchArc2 ((ui_interface_arc_t*)&(ui_default_now_figures[13]))
#define ui_default_StaticGroup_PitchArc3 ((ui_interface_arc_t*)&(ui_default_now_figures[14]))
#define ui_default_StaticGroup_PitchArc4 ((ui_interface_arc_t*)&(ui_default_now_figures[15]))


#ifdef MANUAL_DIRTY
#define ui_default_DynamicBottomGroup_FricRound_dirty (ui_default_dirty_figure[0])
#define ui_default_DynamicBottomGroup_AimRound_dirty (ui_default_dirty_figure[1])
#define ui_default_StaticGroup_PitchArc5_dirty (ui_default_dirty_figure[2])
#define ui_default_DynamicRightGroup_DynamicPitchArc_dirty (ui_default_dirty_figure[3])
#define ui_default_DynamicLeftGroup_ShootHeatArc_dirty (ui_default_dirty_figure[4])
#define ui_default_DynamicRightGroup_ChassisArc_dirty (ui_default_dirty_figure[5])
#define ui_default_StaticGroup_CrossLine1_dirty (ui_default_dirty_figure[6])
#define ui_default_StaticGroup_CrossLine2_dirty (ui_default_dirty_figure[7])
#define ui_default_DynamicBottomGroup_Round_dirty (ui_default_dirty_figure[8])
#define ui_default_StaticGroup_GroundLineLeft_dirty (ui_default_dirty_figure[9])
#define ui_default_StaticGroup_GroundLineRight_dirty (ui_default_dirty_figure[10])
#define ui_default_DynamicLeftGroup_PowerArc_dirty (ui_default_dirty_figure[11])
#define ui_default_StaticGroup_PitchArc1_dirty (ui_default_dirty_figure[12])
#define ui_default_StaticGroup_PitchArc2_dirty (ui_default_dirty_figure[13])
#define ui_default_StaticGroup_PitchArc3_dirty (ui_default_dirty_figure[14])
#define ui_default_StaticGroup_PitchArc4_dirty (ui_default_dirty_figure[15])

#endif

void ui_init_default();
void ui_update_default();

#endif // UI_default_H
