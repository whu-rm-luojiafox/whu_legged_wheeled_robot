//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_7_frame_t ui_g_DynamicGroup_0;

ui_interface_round_t *ui_g_DynamicGroup_FricRound = (ui_interface_round_t*)&(ui_g_DynamicGroup_0.data[0]);
ui_interface_round_t *ui_g_DynamicGroup_AutoRound = (ui_interface_round_t*)&(ui_g_DynamicGroup_0.data[1]);
ui_interface_arc_t *ui_g_DynamicGroup_DirectionArc = (ui_interface_arc_t*)&(ui_g_DynamicGroup_0.data[2]);
ui_interface_number_t *ui_g_DynamicGroup_HightNum = (ui_interface_number_t*)&(ui_g_DynamicGroup_0.data[3]);
ui_interface_line_t *ui_g_DynamicGroup_JumpLine = (ui_interface_line_t*)&(ui_g_DynamicGroup_0.data[4]);
ui_interface_number_t *ui_g_DynamicGroup_FricNum = (ui_interface_number_t*)&(ui_g_DynamicGroup_0.data[5]);
ui_interface_round_t *ui_g_DynamicGroup_ResvRound = (ui_interface_round_t*)&(ui_g_DynamicGroup_0.data[6]);

void _ui_init_g_DynamicGroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_DynamicGroup_0.data[i].figure_name[0] = 0;
        ui_g_DynamicGroup_0.data[i].figure_name[1] = 0;
        ui_g_DynamicGroup_0.data[i].figure_name[2] = i + 0;
        ui_g_DynamicGroup_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_DynamicGroup_0.data[i].operate_type = 0;
    }

    ui_g_DynamicGroup_FricRound->figure_type = 2;
    ui_g_DynamicGroup_FricRound->operate_type = 1;
    ui_g_DynamicGroup_FricRound->layer = 0;
    ui_g_DynamicGroup_FricRound->color = 8;
    ui_g_DynamicGroup_FricRound->start_x = 801;
    ui_g_DynamicGroup_FricRound->start_y = 783;
    ui_g_DynamicGroup_FricRound->width = 15;
    ui_g_DynamicGroup_FricRound->r = 18;

    ui_g_DynamicGroup_AutoRound->figure_type = 2;
    ui_g_DynamicGroup_AutoRound->operate_type = 1;
    ui_g_DynamicGroup_AutoRound->layer = 0;
    ui_g_DynamicGroup_AutoRound->color = 8;
    ui_g_DynamicGroup_AutoRound->start_x = 952;
    ui_g_DynamicGroup_AutoRound->start_y = 783;
    ui_g_DynamicGroup_AutoRound->width = 15;
    ui_g_DynamicGroup_AutoRound->r = 18;

    ui_g_DynamicGroup_DirectionArc->figure_type = 4;
    ui_g_DynamicGroup_DirectionArc->operate_type = 1;
    ui_g_DynamicGroup_DirectionArc->layer = 0;
    ui_g_DynamicGroup_DirectionArc->color = 2;
    ui_g_DynamicGroup_DirectionArc->start_x = 1526;
    ui_g_DynamicGroup_DirectionArc->start_y = 689;
    ui_g_DynamicGroup_DirectionArc->width = 10;
    ui_g_DynamicGroup_DirectionArc->start_angle = 30;
    ui_g_DynamicGroup_DirectionArc->end_angle = 330;
    ui_g_DynamicGroup_DirectionArc->rx = 50;
    ui_g_DynamicGroup_DirectionArc->ry = 50;

    ui_g_DynamicGroup_HightNum->figure_type = 6;
    ui_g_DynamicGroup_HightNum->operate_type = 1;
    ui_g_DynamicGroup_HightNum->layer = 0;
    ui_g_DynamicGroup_HightNum->color = 5;
    ui_g_DynamicGroup_HightNum->start_x = 213;
    ui_g_DynamicGroup_HightNum->start_y = 717;
    ui_g_DynamicGroup_HightNum->width = 2;
    ui_g_DynamicGroup_HightNum->font_size = 22;
    ui_g_DynamicGroup_HightNum->number = 0;

    ui_g_DynamicGroup_JumpLine->figure_type = 0;
    ui_g_DynamicGroup_JumpLine->operate_type = 1;
    ui_g_DynamicGroup_JumpLine->layer = 0;
    ui_g_DynamicGroup_JumpLine->color = 2;
    ui_g_DynamicGroup_JumpLine->start_x = 700;
    ui_g_DynamicGroup_JumpLine->start_y = 130;
    ui_g_DynamicGroup_JumpLine->width = 4;
    ui_g_DynamicGroup_JumpLine->end_x = 1220;
    ui_g_DynamicGroup_JumpLine->end_y = 130;

    ui_g_DynamicGroup_FricNum->figure_type = 6;
    ui_g_DynamicGroup_FricNum->operate_type = 1;
    ui_g_DynamicGroup_FricNum->layer = 0;
    ui_g_DynamicGroup_FricNum->color = 3;
    ui_g_DynamicGroup_FricNum->start_x = 373;
    ui_g_DynamicGroup_FricNum->start_y = 718;
    ui_g_DynamicGroup_FricNum->width = 2;
    ui_g_DynamicGroup_FricNum->font_size = 22;
    ui_g_DynamicGroup_FricNum->number = 0;

    ui_g_DynamicGroup_ResvRound->figure_type = 2;
    ui_g_DynamicGroup_ResvRound->operate_type = 1;
    ui_g_DynamicGroup_ResvRound->layer = 0;
    ui_g_DynamicGroup_ResvRound->color = 8;
    ui_g_DynamicGroup_ResvRound->start_x = 1106;
    ui_g_DynamicGroup_ResvRound->start_y = 783;
    ui_g_DynamicGroup_ResvRound->width = 15;
    ui_g_DynamicGroup_ResvRound->r = 18;


    ui_proc_7_frame(&ui_g_DynamicGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_DynamicGroup_0, sizeof(ui_g_DynamicGroup_0));
}

void _ui_update_g_DynamicGroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_DynamicGroup_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_DynamicGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_DynamicGroup_0, sizeof(ui_g_DynamicGroup_0));
}

void _ui_remove_g_DynamicGroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_DynamicGroup_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_DynamicGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_DynamicGroup_0, sizeof(ui_g_DynamicGroup_0));
}


void ui_init_g_DynamicGroup() {
    _ui_init_g_DynamicGroup_0();
}

void ui_update_g_DynamicGroup() {
    _ui_update_g_DynamicGroup_0();
}

void ui_remove_g_DynamicGroup() {
    _ui_remove_g_DynamicGroup_0();
}

ui_7_frame_t ui_g_StaticGraphicGroup_0;

ui_interface_rect_t *ui_g_StaticGraphicGroup_SelfaimRect = (ui_interface_rect_t*)&(ui_g_StaticGraphicGroup_0.data[0]);
ui_interface_rect_t *ui_g_StaticGraphicGroup_HeatRect = (ui_interface_rect_t*)&(ui_g_StaticGraphicGroup_0.data[1]);
ui_interface_line_t *ui_g_StaticGraphicGroup_CrosshairLine_1 = (ui_interface_line_t*)&(ui_g_StaticGraphicGroup_0.data[2]);
ui_interface_line_t *ui_g_StaticGraphicGroup_GuideLine_1 = (ui_interface_line_t*)&(ui_g_StaticGraphicGroup_0.data[3]);
ui_interface_line_t *ui_g_StaticGraphicGroup_CrosshairLine_2 = (ui_interface_line_t*)&(ui_g_StaticGraphicGroup_0.data[4]);
ui_interface_line_t *ui_g_StaticGraphicGroup_CrosshairLine_3 = (ui_interface_line_t*)&(ui_g_StaticGraphicGroup_0.data[5]);
ui_interface_line_t *ui_g_StaticGraphicGroup_GuideLine_2 = (ui_interface_line_t*)&(ui_g_StaticGraphicGroup_0.data[6]);

void _ui_init_g_StaticGraphicGroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_StaticGraphicGroup_0.data[i].figure_name[0] = 0;
        ui_g_StaticGraphicGroup_0.data[i].figure_name[1] = 1;
        ui_g_StaticGraphicGroup_0.data[i].figure_name[2] = i + 0;
        ui_g_StaticGraphicGroup_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_StaticGraphicGroup_0.data[i].operate_type = 0;
    }

    ui_g_StaticGraphicGroup_SelfaimRect->figure_type = 1;
    ui_g_StaticGraphicGroup_SelfaimRect->operate_type = 1;
    ui_g_StaticGraphicGroup_SelfaimRect->layer = 0;
    ui_g_StaticGraphicGroup_SelfaimRect->color = 6;
    ui_g_StaticGraphicGroup_SelfaimRect->start_x = 710;
    ui_g_StaticGraphicGroup_SelfaimRect->start_y = 320;
    ui_g_StaticGraphicGroup_SelfaimRect->width = 3;
    ui_g_StaticGraphicGroup_SelfaimRect->end_x = 1208;
    ui_g_StaticGraphicGroup_SelfaimRect->end_y = 750;

    ui_g_StaticGraphicGroup_HeatRect->figure_type = 1;
    ui_g_StaticGraphicGroup_HeatRect->operate_type = 1;
    ui_g_StaticGraphicGroup_HeatRect->layer = 0;
    ui_g_StaticGraphicGroup_HeatRect->color = 2;
    ui_g_StaticGraphicGroup_HeatRect->start_x = 194;
    ui_g_StaticGraphicGroup_HeatRect->start_y = 672;
    ui_g_StaticGraphicGroup_HeatRect->width = 3;
    ui_g_StaticGraphicGroup_HeatRect->end_x = 542;
    ui_g_StaticGraphicGroup_HeatRect->end_y = 782;

    ui_g_StaticGraphicGroup_CrosshairLine_1->figure_type = 0;
    ui_g_StaticGraphicGroup_CrosshairLine_1->operate_type = 1;
    ui_g_StaticGraphicGroup_CrosshairLine_1->layer = 0;
    ui_g_StaticGraphicGroup_CrosshairLine_1->color = 1;
    ui_g_StaticGraphicGroup_CrosshairLine_1->start_x = 760;
    ui_g_StaticGraphicGroup_CrosshairLine_1->start_y = 540;
    ui_g_StaticGraphicGroup_CrosshairLine_1->width = 3;
    ui_g_StaticGraphicGroup_CrosshairLine_1->end_x = 1163;
    ui_g_StaticGraphicGroup_CrosshairLine_1->end_y = 540;

    ui_g_StaticGraphicGroup_GuideLine_1->figure_type = 0;
    ui_g_StaticGraphicGroup_GuideLine_1->operate_type = 1;
    ui_g_StaticGraphicGroup_GuideLine_1->layer = 0;
    ui_g_StaticGraphicGroup_GuideLine_1->color = 3;
    ui_g_StaticGraphicGroup_GuideLine_1->start_x = 560;
    ui_g_StaticGraphicGroup_GuideLine_1->start_y = 60;
    ui_g_StaticGraphicGroup_GuideLine_1->width = 3;
    ui_g_StaticGraphicGroup_GuideLine_1->end_x = 659;
    ui_g_StaticGraphicGroup_GuideLine_1->end_y = 239;

    ui_g_StaticGraphicGroup_CrosshairLine_2->figure_type = 0;
    ui_g_StaticGraphicGroup_CrosshairLine_2->operate_type = 1;
    ui_g_StaticGraphicGroup_CrosshairLine_2->layer = 0;
    ui_g_StaticGraphicGroup_CrosshairLine_2->color = 1;
    ui_g_StaticGraphicGroup_CrosshairLine_2->start_x = 860;
    ui_g_StaticGraphicGroup_CrosshairLine_2->start_y = 417;
    ui_g_StaticGraphicGroup_CrosshairLine_2->width = 2;
    ui_g_StaticGraphicGroup_CrosshairLine_2->end_x = 1061;
    ui_g_StaticGraphicGroup_CrosshairLine_2->end_y = 417;

    ui_g_StaticGraphicGroup_CrosshairLine_3->figure_type = 0;
    ui_g_StaticGraphicGroup_CrosshairLine_3->operate_type = 1;
    ui_g_StaticGraphicGroup_CrosshairLine_3->layer = 0;
    ui_g_StaticGraphicGroup_CrosshairLine_3->color = 1;
    ui_g_StaticGraphicGroup_CrosshairLine_3->start_x = 960;
    ui_g_StaticGraphicGroup_CrosshairLine_3->start_y = 743;
    ui_g_StaticGraphicGroup_CrosshairLine_3->width = 3;
    ui_g_StaticGraphicGroup_CrosshairLine_3->end_x = 960;
    ui_g_StaticGraphicGroup_CrosshairLine_3->end_y = 340;

    ui_g_StaticGraphicGroup_GuideLine_2->figure_type = 0;
    ui_g_StaticGraphicGroup_GuideLine_2->operate_type = 1;
    ui_g_StaticGraphicGroup_GuideLine_2->layer = 0;
    ui_g_StaticGraphicGroup_GuideLine_2->color = 3;
    ui_g_StaticGraphicGroup_GuideLine_2->start_x = 1359;
    ui_g_StaticGraphicGroup_GuideLine_2->start_y = 60;
    ui_g_StaticGraphicGroup_GuideLine_2->width = 3;
    ui_g_StaticGraphicGroup_GuideLine_2->end_x = 1260;
    ui_g_StaticGraphicGroup_GuideLine_2->end_y = 239;


    ui_proc_7_frame(&ui_g_StaticGraphicGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticGraphicGroup_0, sizeof(ui_g_StaticGraphicGroup_0));
}

void _ui_update_g_StaticGraphicGroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_StaticGraphicGroup_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_StaticGraphicGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticGraphicGroup_0, sizeof(ui_g_StaticGraphicGroup_0));
}

void _ui_remove_g_StaticGraphicGroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_StaticGraphicGroup_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_StaticGraphicGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticGraphicGroup_0, sizeof(ui_g_StaticGraphicGroup_0));
}


void ui_init_g_StaticGraphicGroup() {
    _ui_init_g_StaticGraphicGroup_0();
}

void ui_update_g_StaticGraphicGroup() {
    _ui_update_g_StaticGraphicGroup_0();
}

void ui_remove_g_StaticGraphicGroup() {
    _ui_remove_g_StaticGraphicGroup_0();
}


ui_string_frame_t ui_g_StaticTextGroup_0;
ui_interface_string_t* ui_g_StaticTextGroup_FricText = &(ui_g_StaticTextGroup_0.option);

void _ui_init_g_StaticTextGroup_0() {
    ui_g_StaticTextGroup_0.option.figure_name[0] = 0;
    ui_g_StaticTextGroup_0.option.figure_name[1] = 2;
    ui_g_StaticTextGroup_0.option.figure_name[2] = 0;
    ui_g_StaticTextGroup_0.option.operate_type = 1;

    ui_g_StaticTextGroup_FricText->figure_type = 7;
    ui_g_StaticTextGroup_FricText->operate_type = 1;
    ui_g_StaticTextGroup_FricText->layer = 0;
    ui_g_StaticTextGroup_FricText->color = 0;
    ui_g_StaticTextGroup_FricText->start_x = 796;
    ui_g_StaticTextGroup_FricText->start_y = 862;
    ui_g_StaticTextGroup_FricText->width = 2;
    ui_g_StaticTextGroup_FricText->font_size = 20;
    ui_g_StaticTextGroup_FricText->str_length = 1;
    strcpy(ui_g_StaticTextGroup_FricText->string, "F");


    ui_proc_string_frame(&ui_g_StaticTextGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticTextGroup_0, sizeof(ui_g_StaticTextGroup_0));
}

void _ui_update_g_StaticTextGroup_0() {
    ui_g_StaticTextGroup_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_StaticTextGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticTextGroup_0, sizeof(ui_g_StaticTextGroup_0));
}

void _ui_remove_g_StaticTextGroup_0() {
    ui_g_StaticTextGroup_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_StaticTextGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticTextGroup_0, sizeof(ui_g_StaticTextGroup_0));
}
ui_string_frame_t ui_g_StaticTextGroup_1;
ui_interface_string_t* ui_g_StaticTextGroup_AutoText = &(ui_g_StaticTextGroup_1.option);

void _ui_init_g_StaticTextGroup_1() {
    ui_g_StaticTextGroup_1.option.figure_name[0] = 0;
    ui_g_StaticTextGroup_1.option.figure_name[1] = 2;
    ui_g_StaticTextGroup_1.option.figure_name[2] = 1;
    ui_g_StaticTextGroup_1.option.operate_type = 1;

    ui_g_StaticTextGroup_AutoText->figure_type = 7;
    ui_g_StaticTextGroup_AutoText->operate_type = 1;
    ui_g_StaticTextGroup_AutoText->layer = 0;
    ui_g_StaticTextGroup_AutoText->color = 0;
    ui_g_StaticTextGroup_AutoText->start_x = 946;
    ui_g_StaticTextGroup_AutoText->start_y = 862;
    ui_g_StaticTextGroup_AutoText->width = 2;
    ui_g_StaticTextGroup_AutoText->font_size = 20;
    ui_g_StaticTextGroup_AutoText->str_length = 1;
    strcpy(ui_g_StaticTextGroup_AutoText->string, "A");


    ui_proc_string_frame(&ui_g_StaticTextGroup_1);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticTextGroup_1, sizeof(ui_g_StaticTextGroup_1));
}

void _ui_update_g_StaticTextGroup_1() {
    ui_g_StaticTextGroup_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_StaticTextGroup_1);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticTextGroup_1, sizeof(ui_g_StaticTextGroup_1));
}

void _ui_remove_g_StaticTextGroup_1() {
    ui_g_StaticTextGroup_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_StaticTextGroup_1);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticTextGroup_1, sizeof(ui_g_StaticTextGroup_1));
}
ui_string_frame_t ui_g_StaticTextGroup_2;
ui_interface_string_t* ui_g_StaticTextGroup_FricSpeedText = &(ui_g_StaticTextGroup_2.option);

void _ui_init_g_StaticTextGroup_2() {
    ui_g_StaticTextGroup_2.option.figure_name[0] = 0;
    ui_g_StaticTextGroup_2.option.figure_name[1] = 2;
    ui_g_StaticTextGroup_2.option.figure_name[2] = 2;
    ui_g_StaticTextGroup_2.option.operate_type = 1;

    ui_g_StaticTextGroup_FricSpeedText->figure_type = 7;
    ui_g_StaticTextGroup_FricSpeedText->operate_type = 1;
    ui_g_StaticTextGroup_FricSpeedText->layer = 0;
    ui_g_StaticTextGroup_FricSpeedText->color = 3;
    ui_g_StaticTextGroup_FricSpeedText->start_x = 368;
    ui_g_StaticTextGroup_FricSpeedText->start_y = 772;
    ui_g_StaticTextGroup_FricSpeedText->width = 2;
    ui_g_StaticTextGroup_FricSpeedText->font_size = 20;
    ui_g_StaticTextGroup_FricSpeedText->str_length = 4;
    strcpy(ui_g_StaticTextGroup_FricSpeedText->string, "FRIC");


    ui_proc_string_frame(&ui_g_StaticTextGroup_2);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticTextGroup_2, sizeof(ui_g_StaticTextGroup_2));
}

void _ui_update_g_StaticTextGroup_2() {
    ui_g_StaticTextGroup_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_StaticTextGroup_2);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticTextGroup_2, sizeof(ui_g_StaticTextGroup_2));
}

void _ui_remove_g_StaticTextGroup_2() {
    ui_g_StaticTextGroup_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_StaticTextGroup_2);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticTextGroup_2, sizeof(ui_g_StaticTextGroup_2));
}
ui_string_frame_t ui_g_StaticTextGroup_3;
ui_interface_string_t* ui_g_StaticTextGroup_DirectText = &(ui_g_StaticTextGroup_3.option);

void _ui_init_g_StaticTextGroup_3() {
    ui_g_StaticTextGroup_3.option.figure_name[0] = 0;
    ui_g_StaticTextGroup_3.option.figure_name[1] = 2;
    ui_g_StaticTextGroup_3.option.figure_name[2] = 3;
    ui_g_StaticTextGroup_3.option.operate_type = 1;

    ui_g_StaticTextGroup_DirectText->figure_type = 7;
    ui_g_StaticTextGroup_DirectText->operate_type = 1;
    ui_g_StaticTextGroup_DirectText->layer = 0;
    ui_g_StaticTextGroup_DirectText->color = 2;
    ui_g_StaticTextGroup_DirectText->start_x = 1439;
    ui_g_StaticTextGroup_DirectText->start_y = 811;
    ui_g_StaticTextGroup_DirectText->width = 2;
    ui_g_StaticTextGroup_DirectText->font_size = 20;
    ui_g_StaticTextGroup_DirectText->str_length = 9;
    strcpy(ui_g_StaticTextGroup_DirectText->string, "direction");


    ui_proc_string_frame(&ui_g_StaticTextGroup_3);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticTextGroup_3, sizeof(ui_g_StaticTextGroup_3));
}

void _ui_update_g_StaticTextGroup_3() {
    ui_g_StaticTextGroup_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_StaticTextGroup_3);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticTextGroup_3, sizeof(ui_g_StaticTextGroup_3));
}

void _ui_remove_g_StaticTextGroup_3() {
    ui_g_StaticTextGroup_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_StaticTextGroup_3);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticTextGroup_3, sizeof(ui_g_StaticTextGroup_3));
}
ui_string_frame_t ui_g_StaticTextGroup_4;
ui_interface_string_t* ui_g_StaticTextGroup_HightText = &(ui_g_StaticTextGroup_4.option);

void _ui_init_g_StaticTextGroup_4() {
    ui_g_StaticTextGroup_4.option.figure_name[0] = 0;
    ui_g_StaticTextGroup_4.option.figure_name[1] = 2;
    ui_g_StaticTextGroup_4.option.figure_name[2] = 4;
    ui_g_StaticTextGroup_4.option.operate_type = 1;

    ui_g_StaticTextGroup_HightText->figure_type = 7;
    ui_g_StaticTextGroup_HightText->operate_type = 1;
    ui_g_StaticTextGroup_HightText->layer = 0;
    ui_g_StaticTextGroup_HightText->color = 5;
    ui_g_StaticTextGroup_HightText->start_x = 205;
    ui_g_StaticTextGroup_HightText->start_y = 772;
    ui_g_StaticTextGroup_HightText->width = 2;
    ui_g_StaticTextGroup_HightText->font_size = 20;
    ui_g_StaticTextGroup_HightText->str_length = 4;
    strcpy(ui_g_StaticTextGroup_HightText->string, "High");


    ui_proc_string_frame(&ui_g_StaticTextGroup_4);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticTextGroup_4, sizeof(ui_g_StaticTextGroup_4));
}

void _ui_update_g_StaticTextGroup_4() {
    ui_g_StaticTextGroup_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_StaticTextGroup_4);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticTextGroup_4, sizeof(ui_g_StaticTextGroup_4));
}

void _ui_remove_g_StaticTextGroup_4() {
    ui_g_StaticTextGroup_4.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_StaticTextGroup_4);
    SEND_MESSAGE((uint8_t *) &ui_g_StaticTextGroup_4, sizeof(ui_g_StaticTextGroup_4));
}

void ui_init_g_StaticTextGroup() {
    _ui_init_g_StaticTextGroup_0();
    _ui_init_g_StaticTextGroup_1();
    _ui_init_g_StaticTextGroup_2();
    _ui_init_g_StaticTextGroup_3();
    _ui_init_g_StaticTextGroup_4();
}

void ui_update_g_StaticTextGroup() {
    _ui_update_g_StaticTextGroup_0();
    _ui_update_g_StaticTextGroup_1();
    _ui_update_g_StaticTextGroup_2();
    _ui_update_g_StaticTextGroup_3();
    _ui_update_g_StaticTextGroup_4();
}

void ui_remove_g_StaticTextGroup() {
    _ui_remove_g_StaticTextGroup_0();
    _ui_remove_g_StaticTextGroup_1();
    _ui_remove_g_StaticTextGroup_2();
    _ui_remove_g_StaticTextGroup_3();
    _ui_remove_g_StaticTextGroup_4();
}

