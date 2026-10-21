//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_frame1.h"

#define TOTAL_FIGURE 5
#define TOTAL_STRING 4

ui_interface_figure_t ui_frame1_now_figures[TOTAL_FIGURE];
uint8_t ui_frame1_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_frame1_now_strings[TOTAL_STRING];
uint8_t ui_frame1_dirty_string[TOTAL_STRING];

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_frame1_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_frame1_last_strings[TOTAL_STRING];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_frame1() {
    ui_frame1_StaticNumberGroup_NewNumber1->figure_type = 6;
    ui_frame1_StaticNumberGroup_NewNumber1->operate_type = 1;
    ui_frame1_StaticNumberGroup_NewNumber1->layer = 0;
    ui_frame1_StaticNumberGroup_NewNumber1->color = 8;
    ui_frame1_StaticNumberGroup_NewNumber1->start_x = 1371;
    ui_frame1_StaticNumberGroup_NewNumber1->start_y = 569;
    ui_frame1_StaticNumberGroup_NewNumber1->width = 2;
    ui_frame1_StaticNumberGroup_NewNumber1->font_size = 15;
    ui_frame1_StaticNumberGroup_NewNumber1->number = 0;

    ui_frame1_StaticNumberGroup_NewNumber2->figure_type = 6;
    ui_frame1_StaticNumberGroup_NewNumber2->operate_type = 1;
    ui_frame1_StaticNumberGroup_NewNumber2->layer = 0;
    ui_frame1_StaticNumberGroup_NewNumber2->color = 7;
    ui_frame1_StaticNumberGroup_NewNumber2->start_x = 1356;
    ui_frame1_StaticNumberGroup_NewNumber2->start_y = 419;
    ui_frame1_StaticNumberGroup_NewNumber2->width = 2;
    ui_frame1_StaticNumberGroup_NewNumber2->font_size = 15;
    ui_frame1_StaticNumberGroup_NewNumber2->number = 15;

    ui_frame1_StaticNumberGroup_NewNumber3->figure_type = 6;
    ui_frame1_StaticNumberGroup_NewNumber3->operate_type = 1;
    ui_frame1_StaticNumberGroup_NewNumber3->layer = 0;
    ui_frame1_StaticNumberGroup_NewNumber3->color = 8;
    ui_frame1_StaticNumberGroup_NewNumber3->start_x = 1280;
    ui_frame1_StaticNumberGroup_NewNumber3->start_y = 826;
    ui_frame1_StaticNumberGroup_NewNumber3->width = 2;
    ui_frame1_StaticNumberGroup_NewNumber3->font_size = 15;
    ui_frame1_StaticNumberGroup_NewNumber3->number = 30;

    ui_frame1_StaticNumberGroup_NewNumber4->figure_type = 6;
    ui_frame1_StaticNumberGroup_NewNumber4->operate_type = 1;
    ui_frame1_StaticNumberGroup_NewNumber4->layer = 0;
    ui_frame1_StaticNumberGroup_NewNumber4->color = 7;
    ui_frame1_StaticNumberGroup_NewNumber4->start_x = 1304;
    ui_frame1_StaticNumberGroup_NewNumber4->start_y = 305;
    ui_frame1_StaticNumberGroup_NewNumber4->width = 2;
    ui_frame1_StaticNumberGroup_NewNumber4->font_size = 15;
    ui_frame1_StaticNumberGroup_NewNumber4->number = 30;

    ui_frame1_StaticNumberGroup_NewNumber5->figure_type = 6;
    ui_frame1_StaticNumberGroup_NewNumber5->operate_type = 1;
    ui_frame1_StaticNumberGroup_NewNumber5->layer = 0;
    ui_frame1_StaticNumberGroup_NewNumber5->color = 8;
    ui_frame1_StaticNumberGroup_NewNumber5->start_x = 1348;
    ui_frame1_StaticNumberGroup_NewNumber5->start_y = 721;
    ui_frame1_StaticNumberGroup_NewNumber5->width = 2;
    ui_frame1_StaticNumberGroup_NewNumber5->font_size = 15;
    ui_frame1_StaticNumberGroup_NewNumber5->number = 15;

    ui_frame1_StaticTextGroup_HeatText->figure_type = 7;
    ui_frame1_StaticTextGroup_HeatText->operate_type = 1;
    ui_frame1_StaticTextGroup_HeatText->layer = 0;
    ui_frame1_StaticTextGroup_HeatText->color = 5;
    ui_frame1_StaticTextGroup_HeatText->start_x = 470;
    ui_frame1_StaticTextGroup_HeatText->start_y = 565;
    ui_frame1_StaticTextGroup_HeatText->width = 2;
    ui_frame1_StaticTextGroup_HeatText->font_size = 15;
    ui_frame1_StaticTextGroup_HeatText->str_length = 1;
    strcpy(ui_frame1_StaticTextGroup_HeatText->string, "H");

    ui_frame1_StaticTextGroup_PowerText->figure_type = 7;
    ui_frame1_StaticTextGroup_PowerText->operate_type = 1;
    ui_frame1_StaticTextGroup_PowerText->layer = 0;
    ui_frame1_StaticTextGroup_PowerText->color = 3;
    ui_frame1_StaticTextGroup_PowerText->start_x = 582;
    ui_frame1_StaticTextGroup_PowerText->start_y = 565;
    ui_frame1_StaticTextGroup_PowerText->width = 2;
    ui_frame1_StaticTextGroup_PowerText->font_size = 15;
    ui_frame1_StaticTextGroup_PowerText->str_length = 1;
    strcpy(ui_frame1_StaticTextGroup_PowerText->string, "P");

    ui_frame1_Ungroup_FricText->figure_type = 7;
    ui_frame1_Ungroup_FricText->operate_type = 1;
    ui_frame1_Ungroup_FricText->layer = 0;
    ui_frame1_Ungroup_FricText->color = 5;
    ui_frame1_Ungroup_FricText->start_x = 880;
    ui_frame1_Ungroup_FricText->start_y = 210;
    ui_frame1_Ungroup_FricText->width = 2;
    ui_frame1_Ungroup_FricText->font_size = 15;
    ui_frame1_Ungroup_FricText->str_length = 1;
    strcpy(ui_frame1_Ungroup_FricText->string, "F");

    ui_frame1_Ungroup_AutoText->figure_type = 7;
    ui_frame1_Ungroup_AutoText->operate_type = 1;
    ui_frame1_Ungroup_AutoText->layer = 0;
    ui_frame1_Ungroup_AutoText->color = 5;
    ui_frame1_Ungroup_AutoText->start_x = 960;
    ui_frame1_Ungroup_AutoText->start_y = 210;
    ui_frame1_Ungroup_AutoText->width = 2;
    ui_frame1_Ungroup_AutoText->font_size = 15;
    ui_frame1_Ungroup_AutoText->str_length = 1;
    strcpy(ui_frame1_Ungroup_AutoText->string, "A");

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_frame1_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_frame1_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_frame1_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_frame1_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_frame1_last_figures[i] = ui_frame1_now_figures[i];
#endif
        ui_frame1_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_frame1_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_frame1_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_frame1_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_frame1_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_frame1_last_strings[i] = ui_frame1_now_strings[i];
#endif
        ui_frame1_dirty_string[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_frame1_now_figures[i].operate_type = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_frame1_now_strings[i].operate_type = 2;
    }
}

void ui_update_frame1() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_frame1_now_figures[i], &ui_frame1_last_figures[i], sizeof(ui_frame1_now_figures[i])) != 0) {
            ui_frame1_dirty_figure[i] = 1;
            ui_frame1_last_figures[i] = ui_frame1_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_frame1_now_strings[i], &ui_frame1_last_strings[i], sizeof(ui_frame1_now_strings[i])) != 0) {
            ui_frame1_dirty_string[i] = 1;
            ui_frame1_last_strings[i] = ui_frame1_now_strings[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
