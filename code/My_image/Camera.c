/*
 * Camera.c
 *
 * 只负责把 my_camera.c 的处理结果显示到 IPS200：
 *   - 二值图像
 *   - 蓝色左右边线
 *   - 红色中线
 *
 * 图像处理和 error 生成由 my_camera.c 完成。
 */

#include "Camera.h"
#include "my_camera.h"

extern volatile uint32 camera_fps;
extern volatile uint8 camera_fps_ready;

#define WHITE (255)
#define BLACK (0)
#define CAMERA_DISPLAY_X    (26)//图像向col方向的偏移量
#define CAMERA_DISPLAY_Y    (18)//图像向row方向的偏移量
#define CAMERA_DISPLAY_DIV  (1)//50fps/CAMERA_DISPLAY_DIV=当前图像的帧率


void IPS200_Draw_X(uint16 x, uint16 y, uint16 color, uint16 size)
{
    for (int i = -size; i <= size; ++i)
    {
        // 判断点 (x-i, y-i) 是否在有效范围内
        if ((x - i) >= 0 && (x - i) < MT9V03X_W && (y - i) >= 0 && (y - i) < MT9V03X_H)
        {
            ips200_draw_point( x- i+CAMERA_DISPLAY_X,  y- i+CAMERA_DISPLAY_Y,  color);
//            ips200_draw_point_On_RGB_Image(debug_rgb_image[0], x - i, y - i, color, MT9V03X_W, MT9V03X_H);
        }

        // 判断点 (x-i, y+i) 是否在有效范围内并绘制
        if ((x - i) >= 0 && (x - i) <  MT9V03X_W && (y + i) >= 0 && (y + i) < MT9V03X_H)
        {

            ips200_draw_point( x - i+CAMERA_DISPLAY_X,  y + i+CAMERA_DISPLAY_Y,  color);
//            ips200_draw_point_On_RGB_Image(debug_rgb_image[0], x - i, y + i, color, MT9V03X_W, MT9V03X_H);
        }
    }
}

//void protect(void)
//{
//
//    uint8 i;
//    uint8 black_line_count=0;
//    for(i= MT9V03X_H-1; i>MT9V03X_H-1-10; i--)//选定区域全黑，认为丢图了
//    {
//        if(bin_image[i][4]==IMG_BLACK&&bin_image[i][5]==IMG_BLACK&&
//           bin_image[i][10]==IMG_BLACK&&bin_image[i][15]==IMG_BLACK&&
//           bin_image[i][20]==IMG_BLACK&&bin_image[i][25]==IMG_BLACK&&
//           bin_image[i][64]==IMG_BLACK&&bin_image[i][65]==IMG_BLACK&&
//           bin_image[i][66]==IMG_BLACK&&bin_image[i][67]==IMG_BLACK&&
//           bin_image[i][68]==IMG_BLACK&&bin_image[i][69]==IMG_BLACK&&
//           bin_image[i][70]==IMG_BLACK&&bin_image[i][75]==IMG_BLACK&&
//           bin_image[i][66]==IMG_BLACK&&bin_image[i][67]==IMG_BLACK&&
//           bin_image[i][80]==IMG_BLACK&&bin_image[i][90]==IMG_BLACK&&
//           bin_image[i][100]==IMG_BLACK&&bin_image[i][110]==IMG_BLACK&&
//           bin_image[i][115]==IMG_BLACK&&bin_image[i][120]==IMG_BLACK&&
//           bin_image[i][125]==IMG_BLACK&&bin_image[i][130]==IMG_BLACK&&
//           bin_image[i][135]==IMG_BLACK&&bin_image[i][136]==IMG_BLACK&&
//           bin_image[i][137]==IMG_BLACK&&bin_image[i][138]==IMG_BLACK
//           )
//        {
//             black_line_count++;
//        }
//     }
//     if(black_line_count>=5)
//     {
//         Start_all_Flag=Start_servo_Flag=Start_motor_Flag=Start_motor_test_Flag=Start_servo_test_Flag=close_status;//所有动力全部关掉
//         all_state=0;
//         black_line_count=0;
//         Menu_Page_Update();
//     }
//}



static void Camera_Draw_Outline(void)//该函数需要两个返回值，边界点个数和边界点数组的地址，下面几个函数同理
{
    uint16 count;
    uint16 i;
    const my_camera_point_t *points;

    points = Camera_Get_Left_Edge(&count);

    for(i = 0; i < count; i++)
    {
        if(points[i].flag)
        {
            ips200_draw_point(
                (uint16)(points[i].col + CAMERA_DISPLAY_X),
                (uint16)(points[i].row + CAMERA_DISPLAY_Y),//+CAMERA_DISPLAY_Y
                RGB565_BLUE
            );
        }
    }

    points = Camera_Get_Right_Edge(&count);

    for(i = 0; i < count; i++)
    {
        if(points[i].flag)
        {
            ips200_draw_point(
                (uint16)(points[i].col + CAMERA_DISPLAY_X),
                (uint16)(points[i].row + CAMERA_DISPLAY_Y),//+ CAMERA_DISPLAY_Y
                RGB565_BLUE
            );
        }
    }
}

static void Camera_Draw_Center_Line(void)
{
    uint16 row;
    const my_camera_point_t *points;
    points = Camera_Get_Center_Line(NULL);
//
    for(row = 0; row < MT9V03X_H; row++)
    {
        if(points[row].flag)
        {
            int16 current_row = points[row].row;
            int16 current_col = points[row].col;

            ips200_draw_point(
                (uint16)(current_col + CAMERA_DISPLAY_X),
                (uint16)(current_row + CAMERA_DISPLAY_Y),
                RGB565_RED
            );
        }
    }

}


void Display_Element_State(void)
{
    element_type element = Camera_Get_Element();
    ele_left left_stage = Camera_Get_Left_Stage();
    ele_right right_stage = Camera_Get_Right_Stage();

    const char *element_text;
    const char *left_text = "LF: INACTIVE";
    const char *right_text = "RT: INACTIVE";

    /* 总状态 */
    switch(element)
    {
        case ELEM_NORMAL:
            element_text = "ELEM: NORMAL";
            break;

        case ELEM_LEFT_ROUNDABOUT:
            element_text = "ELEM: LEFT ROUND";
            break;

        case ELEM_RIGHT_ROUNDABOUT:
            element_text = "ELEM: RIGHT ROUND";
            break;



        default:
            element_text = "ELEM: UNKNOWN";
            break;
    }

    /* 左环岛子状态 */
    if(element == ELEM_LEFT_ROUNDABOUT)
    {
        switch(left_stage)
        {
            case ELE_LEFT_ONE:   left_text = "LF: ONE";     break;
            case ELE_LEFT_TWO:   left_text = "LF: TWO";     break;
            case ELE_LEFT_THR:   left_text = "LF: THREE";   break;
            case ELE_LEFT_FOUR:  left_text = "LF: FOUR";    break;
            case ELE_LEFT_FIVE:  left_text = "LF: FIVE";    break;
            case ELE_LEFT_SIX:   left_text = "LF: SIX";     break;
            case ELE_LEFT_SEVEN: left_text = "LF: SEVEN";   break;
            case ELE_LEFT_ET:    left_text = "LF: EIGHT";   break;
            default:            left_text = "LF: UNKNOWN"; break;
        }
    }

    /* 右环岛子状态 */
    if(element == ELEM_RIGHT_ROUNDABOUT)
    {
        switch(right_stage)
        {
            case ELE_RIGHT_ONE:   right_text = "RT: ONE";     break;
            case ELE_RIGHT_TWO:   right_text = "RT: TWO";     break;
            case ELE_RIGHT_THR:   right_text = "RT: THREE";   break;
            case ELE_RIGHT_FOUR:  right_text = "RT: FOUR";    break;
            case ELE_RIGHT_FIVE:  right_text = "RT: FIVE";    break;
            case ELE_RIGHT_SIX:   right_text = "RT: SIX";     break;
            case ELE_RIGHT_SEVEN: right_text = "RT: SEVEN";   break;
            case ELE_RIGHT_ET:    right_text = "RT: EIGHT";   break;
            default:             right_text = "RT: UNKNOWN"; break;
        }
    }

    /* 清除旧文字 */
    ips200_show_string(
        0, MT9V03X_H + CAMERA_DISPLAY_Y + 20,
        "                    "
    );
    ips200_show_string(
        0, MT9V03X_H + CAMERA_DISPLAY_Y + 40,
        "                    "
    );
    ips200_show_string(
        0, MT9V03X_H + CAMERA_DISPLAY_Y + 60,
        "                    "
    );

    /* 显示总状态、左环子状态、右环子状态 */
    ips200_show_string(
        0, MT9V03X_H + CAMERA_DISPLAY_Y + 20,
        element_text
    );
    ips200_show_string(
        0, MT9V03X_H + CAMERA_DISPLAY_Y + 40,
        left_text
    );
    ips200_show_string(
        0, MT9V03X_H + CAMERA_DISPLAY_Y + 60,
        right_text
    );
}

static void Display_corner_State(void)
{
    const char *str = "no";
    if(corner_l_thick.flag==1&&corner_r_thick.flag==0){
        str="left.corner";
    }
    else if(corner_l_thick.flag==0&&corner_r_thick.flag==1){
        str="right.corner";
    }
    else if(corner_l_thick.flag==1&&corner_r_thick.flag==1){
        str="all.corner";
    }
    else if(corner_l_thick.flag==0&&corner_r_thick.flag==0){
        str="no";
    }
    ips200_show_string (0, MT9V03X_H+CAMERA_DISPLAY_Y + 2, "             "); // 11个空格
    ips200_show_string (0, MT9V03X_H+CAMERA_DISPLAY_Y + 2, (const char *)str);
}





void Camera_Display_Debug(void)
{
    static uint16 last_display_frame_id = 0;
    static uint8 display_divider = 0;
    const uint8 *display_image;

    if(camera_frame_id == last_display_frame_id)
    {
        return;
    }

    last_display_frame_id = camera_frame_id;
    display_divider++;

    if(display_divider < CAMERA_DISPLAY_DIV)
    {
        return;
    }

    display_divider = 0;
    display_image = Camera_Get_Binary_Image();

    ips200_show_gray_image(
        CAMERA_DISPLAY_X,
        CAMERA_DISPLAY_Y,
        display_image,
        MT9V03X_W,
        MT9V03X_H,
        MT9V03X_W,
        MT9V03X_H,
        0
    );
    if(corner_l_thick.flag==1)
    {
        IPS200_Draw_X(corner_l_thick.col,corner_l_thick.row,  RGB565_RED, 5);
    }
//    if(corner_r_thick_far.flag==1)
//    {
//        IPS200_Draw_X(corner_r_thick_far.col,corner_r_thick_far.row,  RGB565_PURPLE, 5);
//    }
//    if(pro_point.flag == 1)
//    {
//        IPS200_Draw_X(pro_point.col,pro_point.row,  RGB565_BLUE, 5);
//    }
    Camera_Draw_Outline();
    Camera_Draw_Center_Line();
    Display_Element_State();
    Display_corner_State();/* ← 加这一行，在图像下方画状态文字 */
//    ips200_show_int(0, MT9V03X_H+CAMERA_DISPLAY_Y + 20, Get_r(), 2);
//    ips200_show_int(0, MT9V03X_H+CAMERA_DISPLAY_Y + 40, Get_l(), 3);
//    ips200_show_float(0, MT9V03X_H+CAMERA_DISPLAY_Y + 80, fangcha_left, 3, 3);
//    ips200_show_float(0, MT9V03X_H+CAMERA_DISPLAY_Y + 100, fangcha_right, 3, 3);
//    ips200_show_float(0, MT9V03X_H+CAMERA_DISPLAY_Y + 100, left_eg_loss, 3, 3);
//    ips200_show_float(0, MT9V03X_H+CAMERA_DISPLAY_Y + 120, right_eg_loss, 3, 3);
//    if(camera_fps_ready)
//    {
//        camera_fps_ready = 0;
//
//        ips200_show_string(
//            0,
//            MT9V03X_H + CAMERA_DISPLAY_Y + 60,
//            "FPS:    "
//        );
//
//        ips200_show_int(
//            40,
//            MT9V03X_H + CAMERA_DISPLAY_Y + 60,
//            (int32)camera_fps,
//            3
//        );
//    }
}

/* 保留旧菜单入口名称，但现在只显示，不处理图像。 */
void Image_Show(void)
{
    Camera_Display_Debug();
}
