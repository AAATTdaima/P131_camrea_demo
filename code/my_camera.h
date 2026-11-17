/*
 * my_camera.h
 *
 * 图像处理模块接口：
 *   - Camera_Process_Frame() 处理一帧图像并发布中线误差
 *   - Camera_Get_*() 提供给 Camera.c 做调试显示
 */

#ifndef CODE_MY_CAMERA_H_
#define CODE_MY_CAMERA_H_
#include "zf_common_headfile.h"


extern volatile uint32 camera_frame_count;
extern volatile uint32 camera_fps;
extern volatile uint8 camera_fps_ready;
extern volatile float fangcha_left;
extern volatile float fangcha_right;
extern volatile uint8 zebra_stop_flag;
//extern volatile uint16 left_eg_loss;
//extern volatile uint16 right_eg_loss;


typedef struct
{
    int16 row;
    int16 col;
    uint8 flag;
    uint8 dire;
} my_camera_point_t;//边界点结构体

typedef struct
{
    uint16 row;
    uint16 col;
    uint8 flag;
    uint8 angle;   /* 拐点内角（度，0~180）：<90 锐角，>90 钝角，≈90 直角；0 表示无法计算 */
    uint8 idex;
} corner_t;//拐点结构体

typedef struct
{
     uint8 flag;
     uint16 row;
     uint16 col;
}hit_i;

typedef enum {
    ELEM_NORMAL = 0,      // 直道巡线
    ELEM_LEFT_ROUNDABOUT,      // 左环岛
    ELEM_RIGHT_ROUNDABOUT,      // 右环岛
} element_type;



typedef enum {
    ELE_LEFT_ONE = 0,
    ELE_LEFT_TWO,      /* 左拐点近端 → 环岛在左，左进 */
    ELE_LEFT_THR,      /* 右拐点近端 → 环岛在右，右进 */
    ELE_LEFT_THR_P,
    ELE_LEFT_FOUR,
    ELE_LEFT_FIVE,
    ELE_LEFT_SIX,
    ELE_LEFT_SEVEN,
    ELE_LEFT_SEVEN_P,
    ELE_LEFT_ET,
} ele_left;

typedef enum {
    ELE_RIGHT_ONE =0,
    ELE_RIGHT_TWO,      /* 左拐点近端 → 环岛在左，左进 */
    ELE_RIGHT_THR,       /* 右拐点近端 → 环岛在右，右进 */
    ELE_RIGHT_FOUR,
    ELE_RIGHT_FIVE,
    ELE_RIGHT_SIX,
    ELE_RIGHT_SEVEN,
    ELE_RIGHT_ET,
} ele_right;

extern volatile ele_left c_ele_lf ;
extern volatile ele_right c_ele_rt;
extern volatile element_type current_element;
extern volatile uint16 left_eg_loss;
extern volatile uint16 right_eg_loss;

extern volatile uint8 left_speed_flag;
extern volatile uint8 right_speed_flag;

extern volatile corner_t corner_l_thick_far;
extern volatile corner_t corner_r_thick_far;
extern volatile corner_t corner_r_thick;
extern volatile corner_t corner_l_thick;
extern hit_i pro_point;


/* CPU0 每获得一帧新的摄像头图像后调用一次。 */
void Camera_Process_Frame(void);
void Track_Main_StateMachine(void);
void camera_fr(void);

/* 处理结果只读访问接口，供 Camera.c 显示使用。 */
const uint8 *Camera_Get_Binary_Image(void);
const my_camera_point_t *Camera_Get_Left_Edge(uint16 *count);//该函数需要两个返回值，边界点个数和边界点数组的地址，下面几个函数同理
const my_camera_point_t *Camera_Get_Right_Edge(uint16 *count);
const my_camera_point_t *Camera_Get_Center_Line(uint16 *count);
const element_type Camera_Get_Element(void);
const ele_left Camera_Get_Left_Stage(void);
//const ele_right Camera_Get_Right_Stage(void);
//



/* 这一层加入get函数，供状态机文件调用               */
/* 状态机在调用过当前帧处理函数后即可通过get函数获取证据变量 */
const my_camera_point_t *Get_edge_L(void);
const my_camera_point_t *Get_edge_R(void);
//

/* CPU1 位置环可读取的共享变量。 */
extern volatile int16  camera_center_error;//底部第五行到第三十五行的中线col坐标与屏幕中线坐标之间的差值的平均值
extern volatile int16  camera_center_col;//屏幕中心线的col坐标
extern volatile uint8  camera_center_valid;//做丢线保护的标志位
extern volatile uint16 camera_frame_id;//帧号，用于判断当前图像是否为新的一帧，可放入vsync中断中

#endif /* CODE_MY_CAMERA_H_ */
