//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_default.h"

#define TOTAL_FIGURE 16
#define TOTAL_STRING 0

ui_interface_figure_t ui_default_now_figures[TOTAL_FIGURE];
uint8_t ui_default_dirty_figure[TOTAL_FIGURE];

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_default_last_figures[TOTAL_FIGURE];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, NULL, NULL, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_default() {
    ui_default_DynamicBottomGroup_FricRound->figure_type = 2;
    ui_default_DynamicBottomGroup_FricRound->operate_type = 1;
    ui_default_DynamicBottomGroup_FricRound->layer = 0;
    ui_default_DynamicBottomGroup_FricRound->color = 5;
    ui_default_DynamicBottomGroup_FricRound->start_x = 880;
    ui_default_DynamicBottomGroup_FricRound->start_y = 250;
    ui_default_DynamicBottomGroup_FricRound->width = 10;
    ui_default_DynamicBottomGroup_FricRound->r = 13;

    ui_default_DynamicBottomGroup_AimRound->figure_type = 2;
    ui_default_DynamicBottomGroup_AimRound->operate_type = 1;
    ui_default_DynamicBottomGroup_AimRound->layer = 0;
    ui_default_DynamicBottomGroup_AimRound->color = 8;
    ui_default_DynamicBottomGroup_AimRound->start_x = 960;
    ui_default_DynamicBottomGroup_AimRound->start_y = 250;
    ui_default_DynamicBottomGroup_AimRound->width = 10;
    ui_default_DynamicBottomGroup_AimRound->r = 13;

    ui_default_StaticGroup_PitchArc5->figure_type = 4;
    ui_default_StaticGroup_PitchArc5->operate_type = 1;
    ui_default_StaticGroup_PitchArc5->layer = 0;
    ui_default_StaticGroup_PitchArc5->color = 8;
    ui_default_StaticGroup_PitchArc5->start_x = 960;
    ui_default_StaticGroup_PitchArc5->start_y = 529;
    ui_default_StaticGroup_PitchArc5->width = 15;
    ui_default_StaticGroup_PitchArc5->start_angle = 70;
    ui_default_StaticGroup_PitchArc5->end_angle = 72;
    ui_default_StaticGroup_PitchArc5->rx = 400;
    ui_default_StaticGroup_PitchArc5->ry = 400;

    ui_default_DynamicRightGroup_DynamicPitchArc->figure_type = 4;
    ui_default_DynamicRightGroup_DynamicPitchArc->operate_type = 1;
    ui_default_DynamicRightGroup_DynamicPitchArc->layer = 0;
    ui_default_DynamicRightGroup_DynamicPitchArc->color = 7;
    ui_default_DynamicRightGroup_DynamicPitchArc->start_x = 953;
    ui_default_DynamicRightGroup_DynamicPitchArc->start_y = 531;
    ui_default_DynamicRightGroup_DynamicPitchArc->width = 18;
    ui_default_DynamicRightGroup_DynamicPitchArc->start_angle = 90;
    ui_default_DynamicRightGroup_DynamicPitchArc->end_angle = 91;
    ui_default_DynamicRightGroup_DynamicPitchArc->rx = 380;
    ui_default_DynamicRightGroup_DynamicPitchArc->ry = 380;

    ui_default_DynamicLeftGroup_ShootHeatArc->figure_type = 4;
    ui_default_DynamicLeftGroup_ShootHeatArc->operate_type = 1;
    ui_default_DynamicLeftGroup_ShootHeatArc->layer = 0;
    ui_default_DynamicLeftGroup_ShootHeatArc->color = 5;
    ui_default_DynamicLeftGroup_ShootHeatArc->start_x = 961;
    ui_default_DynamicLeftGroup_ShootHeatArc->start_y = 542;
    ui_default_DynamicLeftGroup_ShootHeatArc->width = 12;
    ui_default_DynamicLeftGroup_ShootHeatArc->start_angle = 250;
    ui_default_DynamicLeftGroup_ShootHeatArc->end_angle = 270;
    ui_default_DynamicLeftGroup_ShootHeatArc->rx = 450;
    ui_default_DynamicLeftGroup_ShootHeatArc->ry = 450;

    ui_default_DynamicRightGroup_ChassisArc->figure_type = 4;
    ui_default_DynamicRightGroup_ChassisArc->operate_type = 1;
    ui_default_DynamicRightGroup_ChassisArc->layer = 0;
    ui_default_DynamicRightGroup_ChassisArc->color = 6;
    ui_default_DynamicRightGroup_ChassisArc->start_x = 1630;
    ui_default_DynamicRightGroup_ChassisArc->start_y = 750;
    ui_default_DynamicRightGroup_ChassisArc->width = 10;
    ui_default_DynamicRightGroup_ChassisArc->start_angle = 200;
    ui_default_DynamicRightGroup_ChassisArc->end_angle = 160;
    ui_default_DynamicRightGroup_ChassisArc->rx = 50;
    ui_default_DynamicRightGroup_ChassisArc->ry = 50;

    ui_default_StaticGroup_CrossLine1->figure_type = 0;
    ui_default_StaticGroup_CrossLine1->operate_type = 1;
    ui_default_StaticGroup_CrossLine1->layer = 0;
    ui_default_StaticGroup_CrossLine1->color = 8;
    ui_default_StaticGroup_CrossLine1->start_x = 880;
    ui_default_StaticGroup_CrossLine1->start_y = 500;
    ui_default_StaticGroup_CrossLine1->width = 1;
    ui_default_StaticGroup_CrossLine1->end_x = 1040;
    ui_default_StaticGroup_CrossLine1->end_y = 500;

    ui_default_StaticGroup_CrossLine2->figure_type = 0;
    ui_default_StaticGroup_CrossLine2->operate_type = 1;
    ui_default_StaticGroup_CrossLine2->layer = 0;
    ui_default_StaticGroup_CrossLine2->color = 8;
    ui_default_StaticGroup_CrossLine2->start_x = 960;
    ui_default_StaticGroup_CrossLine2->start_y = 360;
    ui_default_StaticGroup_CrossLine2->width = 2;
    ui_default_StaticGroup_CrossLine2->end_x = 960;
    ui_default_StaticGroup_CrossLine2->end_y = 680;

    ui_default_DynamicBottomGroup_Round->figure_type = 2;
    ui_default_DynamicBottomGroup_Round->operate_type = 1;
    ui_default_DynamicBottomGroup_Round->layer = 0;
    ui_default_DynamicBottomGroup_Round->color = 8;
    ui_default_DynamicBottomGroup_Round->start_x = 1040;
    ui_default_DynamicBottomGroup_Round->start_y = 250;
    ui_default_DynamicBottomGroup_Round->width = 10;
    ui_default_DynamicBottomGroup_Round->r = 13;

    ui_default_StaticGroup_GroundLineLeft->figure_type = 0;
    ui_default_StaticGroup_GroundLineLeft->operate_type = 1;
    ui_default_StaticGroup_GroundLineLeft->layer = 0;
    ui_default_StaticGroup_GroundLineLeft->color = 1;
    ui_default_StaticGroup_GroundLineLeft->start_x = 552;
    ui_default_StaticGroup_GroundLineLeft->start_y = 82;
    ui_default_StaticGroup_GroundLineLeft->width = 3;
    ui_default_StaticGroup_GroundLineLeft->end_x = 666;
    ui_default_StaticGroup_GroundLineLeft->end_y = 238;

    ui_default_StaticGroup_GroundLineRight->figure_type = 0;
    ui_default_StaticGroup_GroundLineRight->operate_type = 1;
    ui_default_StaticGroup_GroundLineRight->layer = 0;
    ui_default_StaticGroup_GroundLineRight->color = 1;
    ui_default_StaticGroup_GroundLineRight->start_x = 1368;
    ui_default_StaticGroup_GroundLineRight->start_y = 82;
    ui_default_StaticGroup_GroundLineRight->width = 3;
    ui_default_StaticGroup_GroundLineRight->end_x = 1254;
    ui_default_StaticGroup_GroundLineRight->end_y = 238;

    ui_default_DynamicLeftGroup_PowerArc->figure_type = 4;
    ui_default_DynamicLeftGroup_PowerArc->operate_type = 1;
    ui_default_DynamicLeftGroup_PowerArc->layer = 0;
    ui_default_DynamicLeftGroup_PowerArc->color = 3;
    ui_default_DynamicLeftGroup_PowerArc->start_x = 923;
    ui_default_DynamicLeftGroup_PowerArc->start_y = 526;
    ui_default_DynamicLeftGroup_PowerArc->width = 12;
    ui_default_DynamicLeftGroup_PowerArc->start_angle = 225;
    ui_default_DynamicLeftGroup_PowerArc->end_angle = 270;
    ui_default_DynamicLeftGroup_PowerArc->rx = 376;
    ui_default_DynamicLeftGroup_PowerArc->ry = 420;

    ui_default_StaticGroup_PitchArc1->figure_type = 4;
    ui_default_StaticGroup_PitchArc1->operate_type = 1;
    ui_default_StaticGroup_PitchArc1->layer = 0;
    ui_default_StaticGroup_PitchArc1->color = 8;
    ui_default_StaticGroup_PitchArc1->start_x = 960;
    ui_default_StaticGroup_PitchArc1->start_y = 529;
    ui_default_StaticGroup_PitchArc1->width = 20;
    ui_default_StaticGroup_PitchArc1->start_angle = 90;
    ui_default_StaticGroup_PitchArc1->end_angle = 92;
    ui_default_StaticGroup_PitchArc1->rx = 400;
    ui_default_StaticGroup_PitchArc1->ry = 400;

    ui_default_StaticGroup_PitchArc2->figure_type = 4;
    ui_default_StaticGroup_PitchArc2->operate_type = 1;
    ui_default_StaticGroup_PitchArc2->layer = 0;
    ui_default_StaticGroup_PitchArc2->color = 8;
    ui_default_StaticGroup_PitchArc2->start_x = 960;
    ui_default_StaticGroup_PitchArc2->start_y = 529;
    ui_default_StaticGroup_PitchArc2->width = 20;
    ui_default_StaticGroup_PitchArc2->start_angle = 130;
    ui_default_StaticGroup_PitchArc2->end_angle = 132;
    ui_default_StaticGroup_PitchArc2->rx = 400;
    ui_default_StaticGroup_PitchArc2->ry = 400;

    ui_default_StaticGroup_PitchArc3->figure_type = 4;
    ui_default_StaticGroup_PitchArc3->operate_type = 1;
    ui_default_StaticGroup_PitchArc3->layer = 0;
    ui_default_StaticGroup_PitchArc3->color = 8;
    ui_default_StaticGroup_PitchArc3->start_x = 960;
    ui_default_StaticGroup_PitchArc3->start_y = 529;
    ui_default_StaticGroup_PitchArc3->width = 20;
    ui_default_StaticGroup_PitchArc3->start_angle = 50;
    ui_default_StaticGroup_PitchArc3->end_angle = 52;
    ui_default_StaticGroup_PitchArc3->rx = 400;
    ui_default_StaticGroup_PitchArc3->ry = 400;

    ui_default_StaticGroup_PitchArc4->figure_type = 4;
    ui_default_StaticGroup_PitchArc4->operate_type = 1;
    ui_default_StaticGroup_PitchArc4->layer = 0;
    ui_default_StaticGroup_PitchArc4->color = 8;
    ui_default_StaticGroup_PitchArc4->start_x = 960;
    ui_default_StaticGroup_PitchArc4->start_y = 529;
    ui_default_StaticGroup_PitchArc4->width = 15;
    ui_default_StaticGroup_PitchArc4->start_angle = 110;
    ui_default_StaticGroup_PitchArc4->end_angle = 112;
    ui_default_StaticGroup_PitchArc4->rx = 400;
    ui_default_StaticGroup_PitchArc4->ry = 400;

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_default_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_default_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_default_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_default_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_default_last_figures[i] = ui_default_now_figures[i];
#endif
        ui_default_dirty_figure[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_default_now_figures[i].operate_type = 2;
    }
}

void ui_update_default() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_default_now_figures[i], &ui_default_last_figures[i], sizeof(ui_default_now_figures[i])) != 0) {
            ui_default_dirty_figure[i] = 1;
            ui_default_last_figures[i] = ui_default_now_figures[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
