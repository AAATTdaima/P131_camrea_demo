/*
 * my_camera.c
 *
 * 摄像头图像处理模块：
 *   1. 对 mt9v03x_image 做二值化和滤波；
 *   2. 搜索左右边线并生成中线；
 *   3. 计算 camera_center_error，供 CPU1 使用。
 *
 * 本文件不调用 IPS200 显示函数。显示由 Camera.c 负责。
 */

#include "my_camera.h"

#define MY_CAMERA_EDGE_MAX    (300)//寻线的最大边线点个数
#define BLACK            (0)
#define WHITE            (255)

volatile uint32 camera_frame_count = 0;
volatile uint32 camera_fps = 0;
volatile uint8 camera_fps_ready=0;//计算帧率变量
volatile uint16 camera_frame_id=0;

volatile element_type current_element = ELEM_NORMAL;
volatile ele_left c_ele_lf=ELE_LEFT_ONE ;
volatile ele_right c_ele_rt=ELE_RIGHT_ONE ;

volatile float fangcha_left=0;
volatile float fangcha_right=0;


volatile uint16 left_eg_loss=0;
volatile uint16 right_eg_loss=0;

static uint8 left_roundabout_done = 0;
static uint8 right_roundabout_done = 0;
static uint8 zuocebiaozhi=0;

static uint8 image[MT9V03X_H][MT9V03X_W];//图像处理时操作的缓存数组

static my_camera_point_t left_edge[MY_CAMERA_EDGE_MAX];//左边界结构体
static my_camera_point_t right_edge[MY_CAMERA_EDGE_MAX];//右边界结构体
static my_camera_point_t center_line[MT9V03X_H];//中线结构体

hit_i pro_point={0};

volatile uint8 left_speed_flag=0;
volatile uint8 right_speed_flag=0;

static uint16 left_edge_count = 0;//左边界点计数变量
static uint16 right_edge_count = 0;//右边界点计数变量
static uint16 center_line_count = 0;//中线点计数变量

static uint8 dire_left = 0;//左侧生长方向记录变量
//{7,0,6}
//{4,中,5}
//{3,无,2}
//没有正南分支，因为赛道不会往回走
static uint8 dire_right = 0;//右侧生长方向记录变量
//{7,0,6}
//{4,中,5}
//{3,无,2}
//没有正南分支，因为赛道不会往回走
static uint16 left_search_amount = MY_CAMERA_EDGE_MAX;//左边界点计数上限
static uint16 right_search_amount = MY_CAMERA_EDGE_MAX;//右边界点计数上限

typedef struct
{
    uint16 row;
    uint16 col;
} camera_seed_t;//种子点结构体

static camera_seed_t left_seed = {0, 0};//左侧种子点
static camera_seed_t right_seed = {0, 0};//右侧种子点
static uint8 left_seed_valid = 0;//左侧找到种子标志位，在camera_find_seed和camera_search_neighborhood之间调用
static uint8 right_seed_valid = 0;//右侧找到种子标志位，在camera_find_seed和camera_search_neighborhood之间调用


/* CPU0 发布给 CPU1 的共享结果，输出参数接口。 */
volatile int16  camera_center_error = 0;//底部第五行到第三十五行的中线col坐标与屏幕中线坐标之间的差值的平均值
volatile int16  camera_center_col = MT9V03X_W / 2;//屏幕中心线的col坐标
volatile uint8  camera_center_valid = 0;//做丢线保护的标志位
//拐点相关变量




volatile corner_t corner_r_thick={0};
volatile corner_t corner_l_thick={0};
volatile corner_t corner_l_thick_far={0};
volatile corner_t corner_r_thick_far={0};

//大津法
static uint8 camera_otsu_threshold(const uint8 *src)
{
    uint8 pixel_max = 0;
    uint8 pixel_min = 255;
    uint16 pixel_count[256] = {0};
    float pixel_probability[256] = {0};
    uint32 pixel_sum = (uint32)MT9V03X_W * MT9V03X_H / 4;
    uint32 gray_sum = 0;
    uint8 threshold = 0;
    uint16 i;
    uint16 j;

    for(i = 0; i < MT9V03X_H; i += 2)
    {
        for(j = 0; j < MT9V03X_W; j += 2)
        {
            uint8 gray = src[(uint32)i * MT9V03X_W + j];

            pixel_count[gray]++;
            gray_sum += gray;

            if(gray > pixel_max)
            {
                pixel_max = gray;
            }

            if(gray < pixel_min)
            {
                pixel_min = gray;
            }
        }
    }

    if(pixel_min == pixel_max)
    {
        return pixel_min;
    }

    for(i = pixel_min; i <= pixel_max; i++)
    {
        pixel_probability[i] =
            (float)pixel_count[i] / (float)pixel_sum;
    }

    {
        float weight0 = 0.0f;
        float weight1;
        float mean0_temp = 0.0f;
        float mean1_temp;
        float mean0;
        float mean1;
        float delta;
        float delta_max = 0.0f;
        float global_mean = (float)gray_sum / (float)pixel_sum;

        threshold = pixel_min;

        for(j = pixel_min; j <= pixel_max; j++)
        {
            weight0 += pixel_probability[j];
            mean0_temp += (float)j * pixel_probability[j];
            weight1 = 1.0f - weight0;

            if(weight0 < 1e-6f || weight1 < 1e-6f)
            {
                continue;
            }

            mean1_temp = global_mean - mean0_temp;
            mean0 = mean0_temp / weight0;
            mean1 = mean1_temp / weight1;
            delta = weight0 * weight1 * (mean0 - mean1) * (mean0 - mean1);

            if(delta > delta_max)
            {
                delta_max = delta;
                threshold = (uint8)j;
            }
        }
    }

    return threshold;
}
//

////斑马线停车
//uint8 zebra_crossing_stop(void)
//{
//    uint8 count=0;
//    uint8 flag=0;
//    for(uint16 i=0;i<MT9V03X_W-2;i++)
//    {
//        if(image[110][i]==255&&image[110][i+1]==0&&image[110][i+2]==0)
//        {
//            count+=1;
//        }
//    }
//    return flag=(count>=7)?1:0;
//}
////
//
////斑马线停车函数
volatile uint8 zebra_stop_flag = 0;
//static void Zebra_Stop_Update(void)
//{
//    static uint8 zebra_count = 0;  /* 已经过的斑马线次数 */
//    static uint8 armed = 1;        /* 是否允许记录下一次 */
//    static uint8 clear_frames = 0; /* 连续未检测到的帧数 */
//    static uint8 delay_frames = 0; /* 第二次识别后的帧数 */
//
//    uint8 detected;
//
//    /* 停车后保持锁存 */
//    if(zebra_stop_flag)
//    {
//        return;
//    }
//
//    /* 第二次识别后，无论后续是否看见斑马线，都继续计帧 */
//    if(zebra_count >= 2)
//    {
//        delay_frames++;
//
//        if(delay_frames >= 45)
//        {
//            zebra_stop_flag = 1;
//        }
//
//        return;
//    }
//
//    detected = zebra_crossing_stop();
//
//    if(detected)
//    {
//        clear_frames = 0;
//
//        /* 同一条斑马线连续出现多帧，只记录一次 */
//        if(armed)
//        {
//            zebra_count++;
//            armed = 0;
//        }
//    }
//    else if(!armed)
//    {
//        clear_frames++;
//
//        /* 连续5帧未检测到，允许记录下一次 */
//        if(clear_frames >= 5)
//        {
//            armed = 1;
//            clear_frames = 0;
//        }
//    }
//}
////


//覆写数组
static void camera_binarize(uint8 threshold)
{
    uint16 row;
    uint16 col;

    for(row = 0; row < MT9V03X_H; row++)
    {
        for(col = 0; col < MT9V03X_W; col++)
        {
            image[row][col] =
                (mt9v03x_image[row][col] >= threshold) ? WHITE : BLACK;
        }
    }
//    Zebra_Stop_Update();
}
//


//丢线保护
void Camera_Check_Bottom_Black(void)
{
    uint16 black_count=0;
    for(uint8 row = MT9V03X_H-10; row < MT9V03X_H; row++)
       {
           for(uint8 col = 0; col < MT9V03X_W; col++)
           {
               if(image[row][col] == BLACK)
               {
                   black_count++;

                   if(black_count > 1600)
                   {
                       camera_center_valid=0;
                       return;
                   }
               }
           }
       }
}
//


//八邻域画黑框
static void camera_draw_border(void)
{
    uint16 i;

    for(i = 0; i < MT9V03X_H; i++)
    {
        image[i][0] = BLACK;
        image[i][1] = BLACK;
        image[i][MT9V03X_W - 1] = BLACK;
        image[i][MT9V03X_W - 2] = BLACK;
    }

    for(i = 0; i < MT9V03X_W; i++)
    {
        image[0][i] = BLACK;
        image[1][i] = BLACK;
    }
}
//

//找种子点
static void camera_find_seed(void)
{
    uint16 seed_col = 0;
    uint16 i;

    left_seed_valid = 0;
    right_seed_valid = 0;

    if(image[MT9V03X_H - 1][MT9V03X_W / 2] == WHITE &&
       image[MT9V03X_H - 1][MT9V03X_W / 2 + 1] == WHITE &&
       image[MT9V03X_H - 1][MT9V03X_W / 2 - 1] == WHITE)
    {
        seed_col = MT9V03X_W / 2;
    }
    else if(image[MT9V03X_H - 1][MT9V03X_W / 4] == WHITE &&
            image[MT9V03X_H - 1][MT9V03X_W / 4 + 1] == WHITE &&
            image[MT9V03X_H - 1][MT9V03X_W / 4 - 1] == WHITE)
    {
        seed_col = MT9V03X_W / 4;
    }
    else if(image[MT9V03X_H - 1][MT9V03X_W * 3 / 4] == WHITE &&
            image[MT9V03X_H - 1][MT9V03X_W * 3 / 4 + 1] == WHITE &&
            image[MT9V03X_H - 1][MT9V03X_W * 3 / 4 - 1] == WHITE)
    {
        seed_col = MT9V03X_W * 3 / 4;
    }
    else
    {
        return;
    }

    for(i = seed_col; i >= 2; i--)
    {
        if(image[MT9V03X_H - 1][i] == WHITE &&
           image[MT9V03X_H - 1][i - 1] == BLACK &&
           image[MT9V03X_H - 1][i - 2] == BLACK)
        {
            left_seed.col = i - 1;
            left_seed.row = MT9V03X_H - 1;
            left_seed_valid = 1;
            break;
        }
    }

    for(i = seed_col; i + 2 < MT9V03X_W; i++)
    {
        if(image[MT9V03X_H - 1][i] == WHITE &&
           image[MT9V03X_H - 1][i + 1] == BLACK &&
           image[MT9V03X_H - 1][i + 2] == BLACK)
        {
            right_seed.col = i + 1;
            right_seed.row = MT9V03X_H - 1;
            right_seed_valid = 1;
            break;
        }
    }
}
//====== 将找到的种子放进左右边线结构体中 ========
static void camera_store_left(uint16 index, int16 row, int16 col,uint8 dire)
{
    if(index < MY_CAMERA_EDGE_MAX)
    {
        left_edge[index].row = row;
        left_edge[index].col = col;
        left_edge[index].flag = 1;
        left_edge[index].dire =dire;
    }
}

static void camera_store_right(uint16 index, int16 row, int16 col,uint8 dire)
{
    if(index < MY_CAMERA_EDGE_MAX)
    {
        right_edge[index].row = row;
        right_edge[index].col = col;
        right_edge[index].flag = 1;
        right_edge[index].dire =dire;
    }
}
//

//八邻域爬线
static void camera_search_neighborhood(void)
{
    uint16 i;

    left_edge_count = 0;
    right_edge_count = 0;

    if(1)
    {
        int16 curr_row = (int16)left_seed.row;
        int16 curr_col = (int16)left_seed.col;

        camera_store_left(0, curr_row, curr_col,0);
        left_edge_count = 1;
        dire_left = 0;

        for(i = 1; i < left_search_amount; i++)
        {
            if(curr_row < 0 || curr_row >= MT9V03X_H ||
               curr_col < 0 || curr_col >= MT9V03X_W)
            {
                break;
            }

            if(curr_row > 0 && curr_col > 0 && dire_left != 2 &&
               image[curr_row - 1][curr_col - 1] == BLACK &&
               image[curr_row - 1][curr_col] == WHITE)
            {
                curr_row--;
                curr_col--;
                dire_left = 7;
            }
            else if(curr_row > 0 && curr_col + 1 < MT9V03X_W && dire_left != 3 &&
                    image[curr_row - 1][curr_col + 1] == BLACK &&
                    image[curr_row][curr_col + 1] == WHITE)
            {
                curr_row--;
                curr_col++;
                dire_left = 6;
            }
            else if(curr_row > 0 && curr_col + 1 < MT9V03X_W &&
                    image[curr_row - 1][curr_col] == BLACK &&
                    image[curr_row - 1][curr_col + 1] == WHITE)
            {
                curr_row--;
                dire_left = 0;
            }
            else if(curr_col > 0 && curr_row > 0 && dire_left != 5 &&
                    image[curr_row][curr_col - 1] == BLACK &&
                    image[curr_row - 1][curr_col - 1] == WHITE)
            {
                curr_col--;
                dire_left = 4;
            }
            else if(curr_col + 1 < MT9V03X_W && curr_row + 1 < MT9V03X_H && dire_left != 4 &&
                    image[curr_row][curr_col + 1] == BLACK &&
                    image[curr_row + 1][curr_col + 1] == WHITE)
            {
                curr_col++;
                dire_left = 5;
            }
            else if(curr_row + 1 < MT9V03X_H && curr_col > 0 && dire_left != 6 &&
                    image[curr_row + 1][curr_col - 1] == BLACK &&
                    image[curr_row][curr_col - 1] == WHITE)
            {
                curr_row++;
                curr_col--;
                dire_left = 3;
            }
            else if(curr_row + 1 < MT9V03X_H && curr_col + 1 < MT9V03X_W && dire_left != 7 &&
                    image[curr_row + 1][curr_col + 1] == BLACK &&
                    image[curr_row + 1][curr_col] == WHITE)
            {
                curr_row++;
                curr_col++;
                dire_left = 2;
            }
            else
            {
                break;
            }

            camera_store_left(i, curr_row, curr_col,dire_left);
            left_edge_count++;
        }
    }

    if(1)
    {
        int16 curr_row = (int16)right_seed.row;
        int16 curr_col = (int16)right_seed.col;

        camera_store_right(0, curr_row, curr_col,0);
        right_edge_count = 1;
        dire_right = 0;

        for(i = 1; i < right_search_amount; i++)
        {
            if(curr_row < 0 || curr_row >= MT9V03X_H ||
               curr_col < 0 || curr_col >= MT9V03X_W)
            {
                break;
            }

            if(curr_row > 0 && curr_col + 1 < MT9V03X_W && dire_right != 3 &&
               image[curr_row - 1][curr_col + 1] == BLACK &&
               image[curr_row - 1][curr_col] == WHITE)
            {
                curr_row--;
                curr_col++;
                dire_right = 6;
            }
            else if(curr_row > 0 && curr_col > 0 && dire_right != 2 &&
                    image[curr_row - 1][curr_col - 1] == BLACK &&
                    image[curr_row][curr_col - 1] == WHITE)
            {
                curr_row--;
                curr_col--;
                dire_right = 7;
            }
            else if(curr_row > 0 && curr_col > 0 &&
                    image[curr_row - 1][curr_col] == BLACK &&
                    image[curr_row - 1][curr_col - 1] == WHITE)
            {
                curr_row--;
                dire_right = 0;
            }
            else if(curr_col + 1 < MT9V03X_W && curr_row > 0 && dire_right != 4 &&
                    image[curr_row][curr_col + 1] == BLACK &&
                    image[curr_row - 1][curr_col + 1] == WHITE)
            {
                curr_col++;
                dire_right = 5;
            }
            else if(curr_col > 0 && curr_row + 1 < MT9V03X_H && dire_right != 5 &&
                    image[curr_row][curr_col - 1] == BLACK &&
                    image[curr_row + 1][curr_col - 1] == WHITE)
            {
                curr_col--;
                dire_right = 4;
            }
            else if(curr_row + 1 < MT9V03X_H && curr_col > 0 && dire_right != 6 &&
                    image[curr_row + 1][curr_col - 1] == BLACK &&
                    image[curr_row + 1][curr_col] == WHITE)
            {
                curr_row++;
                curr_col--;
                dire_right = 3;
            }
            else if(curr_row + 1 < MT9V03X_H && curr_col + 1 < MT9V03X_W && dire_right != 7 &&
                    image[curr_row + 1][curr_col + 1] == BLACK &&
                    image[curr_row][curr_col + 1] == WHITE)
            {
                curr_row++;
                curr_col++;
                dire_right = 2;
            }
            else
            {
                break;
            }

            camera_store_right(i, curr_row, curr_col,dire_right);
            right_edge_count++;
        }
    }
}

//得到边线数组后将左右各两列设置为不感兴趣，roi为取消搜索行数
static void simple_roi_filter(uint16 roi)
{
    for (uint16 i = 0; i < MY_CAMERA_EDGE_MAX; i++)
    {
        if (left_edge[i].row < roi)  left_edge[i].flag = 0;
        if (right_edge[i].row< roi)  right_edge[i].flag = 0;
    }
}
//

//计算中线点，逐行计算
static void camera_build_center_line(void)
{
    uint16 row;
    uint16 i;

    center_line_count = 0;

    for(row = 0; row < MT9V03X_H; row++)
    {
        uint16 left_num = 0;
        uint16 right_num = 0;
        int16 left_col = MT9V03X_W;
        int16 right_col = -1;

        center_line[row].row = (int16)row;
        center_line[row].col = -1;
        center_line[row].flag = 0;

        //当前行左边线取最左侧的有效点
        for(i = 0; i < left_edge_count; i++)
        {
            if(left_edge[i].flag && left_edge[i].row == (int16)row &&
               left_edge[i].col >= 0 && left_edge[i].col < MT9V03X_W)
            {
                if(left_edge[i].col < left_col)
                {
                    left_col = left_edge[i].col;
                }
                left_num++;
            }
        }

        //当前行右边线取最右侧的有效点
        for(i = 0; i < right_edge_count; i++)
        {
            if(right_edge[i].flag && right_edge[i].row == (int16)row &&
               right_edge[i].col >= 0 && right_edge[i].col < MT9V03X_W)
            {
                if(right_edge[i].col > right_col)
                {
                    right_col = right_edge[i].col;
                }
                right_num++;
            }
        }

        //当前行左右两侧都有有效点
        if(left_num > 0 && right_num > 0)
        {
            if(right_col > left_col)
            {
                center_line[row].col =
                    (int16)(((int32)left_col + right_col) / 2);
                center_line[row].flag = 1;
                center_line_count++;
            }
        }
    }
}

//计算28行中线平均坐标
static void camera_update_center_error(uint8 max,uint8 min)
{
    int16 row;
    int32 center_sum = 0;
    uint16 center_num = 0;

    camera_center_valid = 0;//此变量可以用来写丢线保护，丢线时，让电机不再输出pwm
    for(row = max; row >= min; row--)//70  50
    {
        if(center_line[row].flag)
        {
            center_sum += center_line[row].col;
            center_num++;
        }
    }
    if(center_num > 0)
    {
        camera_center_col = (int16)(center_sum / center_num);
        camera_center_error =
        camera_center_col - (int16)(MT9V03X_W / 2);
        camera_center_valid = 1;
    }
    else
    {
        camera_center_col = MT9V03X_W / 2;
        camera_center_error = 0;
    }
}
//



/* 普通巡线：根据累计里程切换前瞻 */
static void camera_update_center_error_by_mileage(void)
{
    /* 读取一次，避免判断过程中被编码器中断更新 */
    int32 mileage = (int32)left_count_f;

    if (mileage <= 130000)
    {
        /* 起步高速段：远前瞻 */
        camera_update_center_error(46, 42);
    }
    else if (mileage <= 270000)
    {
        /* 中间低速段：近前瞻 */
        camera_update_center_error(70, 50);
    }
    else
    {
        /* 后程高速段：远前瞻 */
        camera_update_center_error(46, 42);
    }
}

// 两点确定直线，向两端延伸到图像边缘，直接替代左边线
static uint16 bresenham_left_line(
    int16 x0, int16 y0,
    int16 x1, int16 y1)
{
    int32 vx, vy;
    int32 x, y, end_x, end_y;
    int32 dx, dy, sx, sy;
    int32 err, e2;
    uint16 i, count = 0;

    float t_min = -1.0e9f;
    float t_max =  1.0e9f;
    float t1, t2, temp;
    float fx, fy;

    const int32 max_x = MT9V03X_W - 1;
    const int32 max_y = MT9V03X_H - 1;

    /* 清除旧边线有效标志 */
    left_edge_count = 0;

    for (i = 0; i < MY_CAMERA_EDGE_MAX; i++)
    {
        left_edge[i].flag = 0;
    }

    vx = (int32)x1 - (int32)x0;
    vy = (int32)y1 - (int32)y0;

    /* 两点重合，无法确定延长方向 */
    if (vx == 0 && vy == 0)
    {
        return 0;
    }

    /*
     * 直线参数方程：
     * x = x0 + t * vx
     * y = y0 + t * vy
     *
     * 不限制t在[0,1]内，实现两端延长。
     */

    /* 根据左右边界，限制参数t的范围 */
    if (vx == 0)
    {
        if (x0 < 0 || x0 > max_x)
        {
            return 0;
        }
    }
    else
    {
        t1 = (0.0f - (float)x0) / (float)vx;
        t2 = ((float)max_x - (float)x0) / (float)vx;

        if (t1 > t2)
        {
            temp = t1;
            t1 = t2;
            t2 = temp;
        }

        if (t1 > t_min)
            t_min = t1;

        if (t2 < t_max)
            t_max = t2;
    }

    /* 根据上下边界，进一步限制参数t的范围 */
    if (vy == 0)
    {
        if (y0 < 0 || y0 > max_y)
        {
            return 0;
        }
    }
    else
    {
        t1 = (0.0f - (float)y0) / (float)vy;
        t2 = ((float)max_y - (float)y0) / (float)vy;

        if (t1 > t2)
        {
            temp = t1;
            t1 = t2;
            t2 = temp;
        }

        if (t1 > t_min)
            t_min = t1;

        if (t2 < t_max)
            t_max = t2;
    }

    /* 直线与图像无交点 */
    if (t_min > t_max)
    {
        return 0;
    }

    /* 计算延长后的起点 */
    fx = (float)x0 + t_min * (float)vx;
    fy = (float)y0 + t_min * (float)vy;

    if (fx < 0.0f) fx = 0.0f;
    if (fx > (float)max_x) fx = (float)max_x;
    if (fy < 0.0f) fy = 0.0f;
    if (fy > (float)max_y) fy = (float)max_y;

    x = (int32)(fx + 0.5f);
    y = (int32)(fy + 0.5f);

    /* 计算延长后的终点 */
    fx = (float)x0 + t_max * (float)vx;
    fy = (float)y0 + t_max * (float)vy;

    if (fx < 0.0f) fx = 0.0f;
    if (fx > (float)max_x) fx = (float)max_x;
    if (fy < 0.0f) fy = 0.0f;
    if (fy > (float)max_y) fy = (float)max_y;

    end_x = (int32)(fx + 0.5f);
    end_y = (int32)(fy + 0.5f);

    /* Bresenham整数画线 */
    dx = (end_x >= x) ? (end_x - x) : (x - end_x);
    dy = (end_y >= y) ? (end_y - y) : (y - end_y);

    sx = (x < end_x) ? 1 : -1;
    sy = (y < end_y) ? 1 : -1;

    err = dx - dy;

    while (count < MY_CAMERA_EDGE_MAX)
    {
        if (x < 0 || x > max_x ||
            y < 0 || y > max_y)
        {
            break;
        }

        left_edge[count].col  = (int16)x;
        left_edge[count].row  = (int16)y;
        left_edge[count].flag = 1;
        left_edge[count].dire = 0;
        count++;

        if (x == end_x && y == end_y)
        {
            break;
        }

        e2 = 2 * err;

        if (e2 > -dy)
        {
            err -= dy;
            x += sx;
        }

        if (e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }

    left_edge_count = count;
    return count;
}

// 两点确定直线，向两端延伸到图像边缘，直接替代右边线
static uint16 bresenham_right_line(int16 x0, int16 y0,int16 x1, int16 y1)
{
    int32 vx, vy;
    int32 x, y, end_x, end_y;
    int32 dx, dy, sx, sy;
    int32 err, e2;
    uint16 i, count = 0;

    float t_min = -1.0e9f;
    float t_max =  1.0e9f;
    float t1, t2, temp;
    float fx, fy;

    const int32 max_x = MT9V03X_W - 1;
    const int32 max_y = MT9V03X_H - 1;

    /* 清除旧边线，避免剩余点仍被当成有效点 */
    right_edge_count = 0;

    for (i = 0; i < MY_CAMERA_EDGE_MAX; i++)
    {
        right_edge[i].flag = 0;
    }

    /* 使用32位运算，避免坐标相减时溢出 */
    vx = (int32)x1 - (int32)x0;
    vy = (int32)y1 - (int32)y0;

    /* 两点重合，无法确定延长方向 */
    if (vx == 0 && vy == 0)
    {
        return 0;
    }

    /*
     * 直线参数方程：
     * x = x0 + t * vx
     * y = y0 + t * vy
     *
     * 原始两点分别对应 t=0 和 t=1。
     * 不限制 t 在[0,1]内，从而实现两端延长。
     */

    /* 根据左右边界，限制参数t的范围 */
    if (vx == 0)
    {
        /* 竖直线位于图像外，无交点 */
        if (x0 < 0 || x0 > max_x)
        {
            return 0;
        }
    }
    else
    {
        t1 = (0.0f - (float)x0) / (float)vx;
        t2 = ((float)max_x - (float)x0) / (float)vx;

        if (t1 > t2)
        {
            temp = t1;
            t1 = t2;
            t2 = temp;
        }

        if (t1 > t_min)
            t_min = t1;

        if (t2 < t_max)
            t_max = t2;
    }

    /* 根据上下边界，进一步限制参数t的范围 */
    if (vy == 0)
    {
        /* 水平线位于图像外，无交点 */
        if (y0 < 0 || y0 > max_y)
        {
            return 0;
        }
    }
    else
    {
        t1 = (0.0f - (float)y0) / (float)vy;
        t2 = ((float)max_y - (float)y0) / (float)vy;

        if (t1 > t2)
        {
            temp = t1;
            t1 = t2;
            t2 = temp;
        }

        if (t1 > t_min)
            t_min = t1;

        if (t2 < t_max)
            t_max = t2;
    }

    /* 直线不经过图像 */
    if (t_min > t_max)
    {
        return 0;
    }

    /* 求延长后起点，并限制浮点误差 */
    fx = (float)x0 + t_min * (float)vx;
    fy = (float)y0 + t_min * (float)vy;

    if (fx < 0.0f) fx = 0.0f;
    if (fx > (float)max_x) fx = (float)max_x;
    if (fy < 0.0f) fy = 0.0f;
    if (fy > (float)max_y) fy = (float)max_y;

    x = (int32)(fx + 0.5f);
    y = (int32)(fy + 0.5f);

    /* 求延长后终点 */
    fx = (float)x0 + t_max * (float)vx;
    fy = (float)y0 + t_max * (float)vy;

    if (fx < 0.0f) fx = 0.0f;
    if (fx > (float)max_x) fx = (float)max_x;
    if (fy < 0.0f) fy = 0.0f;
    if (fy > (float)max_y) fy = (float)max_y;

    end_x = (int32)(fx + 0.5f);
    end_y = (int32)(fy + 0.5f);

    /* Bresenham整数画线 */
    dx = (end_x >= x) ? (end_x - x) : (x - end_x);
    dy = (end_y >= y) ? (end_y - y) : (y - end_y);

    sx = (x < end_x) ? 1 : -1;
    sy = (y < end_y) ? 1 : -1;

    err = dx - dy;

    while (count < MY_CAMERA_EDGE_MAX)
    {
        /* 写入前检查坐标，防止越界 */
        if (x < 0 || x > max_x ||
            y < 0 || y > max_y)
        {
            break;
        }

        right_edge[count].col  = (int16)x;
        right_edge[count].row  = (int16)y;
        right_edge[count].flag = 1;
        right_edge[count].dire = 0;
        count++;

        if (x == end_x && y == end_y)
        {
            break;
        }

        e2 = 2 * err;

        if (e2 > -dy)
        {
            err -= dy;
            x += sx;
        }

        if (e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }

    right_edge_count = count;
    return count;
}


//找锐角拐点函数
static corner_t last_right_corner = {0};
static corner_t last_left_corner = {0};
//拐点初判，判断一定行内近处锐角拐点稳定性高
static void get_turning_point(void)
{

    /* ---- 左拐点 ---- */
    corner_l_thick.flag = 0; corner_l_thick.row = 0; corner_l_thick.col = 0;
    if (  left_edge_count > 9)
    {
        for (int i = 0; i < left_edge_count - 9; i++)
        {
            if (left_edge[i].flag==1&&left_edge[i + 8].row > 5)
            {
                /* 向量点积粗筛：B 处确实拐折 */
                if ((left_edge[i].col   - left_edge[i + 4].col) * (left_edge[i + 8].col - left_edge[i + 4].col) +
                    (left_edge[i].row   - left_edge[i + 4].row) * (left_edge[i + 8].row - left_edge[i + 4].row) >= 0)
                {
                    /* 方向判据：左边界向左撑开（近端 col > 远端 col） */
                    if (left_edge[i + 4].col > left_edge[i + 8].col)
                    {
                        corner_l_thick.flag = 1;
                        corner_l_thick.row  = left_edge[i + 4].row;
                        corner_l_thick.col  = left_edge[i + 4].col;
                        break;
                    }
                }
            }
        }
    }

    /* ---- 右拐点 ---- */
    corner_r_thick.flag = 0; corner_r_thick.row = 0; corner_r_thick.col = 0;
    if ( right_edge_count > 9)
    {
        for (int i = 0; i < right_edge_count - 9; i++)
        {
            if (right_edge[i].flag==1&&right_edge[i + 8].row > 5)
            {
                if ((right_edge[i].col   - right_edge[i + 4].col) * (right_edge[i + 8].col - right_edge[i + 4].col) +
                    (right_edge[i].row   - right_edge[i + 4].row) * (right_edge[i + 8].row - right_edge[i + 4].row) >= 0)
                {
                    /* 方向判据：右边界向右撑开（远端 col > 近端 col） */
                    if (right_edge[i + 8].col > right_edge[i + 4].col)
                    {
                        corner_r_thick.flag = 1;
                        corner_r_thick.row  = left_edge[i + 4].row;   // ✅ 已改回 right_edge
                        corner_r_thick.col  = left_edge[i + 4].col;
                        break;
                    }
                }
            }
        }
    }
}
//


static corner_t last_right_corner_far = {0};
static corner_t last_left_corner_far = {0};
/* 从上到下寻找远处钝角拐点 */
static void get_far_turning_point(void)
{
    uint16 row, i, j;
    uint16 far_col, far_row;
    float ux, uy, vx, vy;
    float len_u2, len_v2, cos_angle;

    const uint8 min_row = 20;
    const uint8 edge_margin = 5;

    /* 夹角范围：100°～160° */
    const float cos_160 = -0.93969262f;
    const float cos_100 = -0.17364818f;

    /* ---- 左侧远钝角拐点 ---- */
    corner_l_thick_far.flag = 0;
    corner_l_thick_far.row  = 0;
    corner_l_thick_far.col  = 0;

    if (left_edge_count >= 9)
    {
        /* 行号递增，从远处向近处搜索 */
        for (row = min_row; row < MT9V03X_H; row++)
        {
            for (i = 4; i < left_edge_count - 4; i++)
            {
                if (left_edge[i].row != row)
                    continue;

                /* 九点窗口有效性检查 */
                for (j = i - 4; j <= i + 4; j++)
                {
                    if (left_edge[j].flag != 1 ||
                        left_edge[j].row < 0 ||
                        left_edge[j].row >= MT9V03X_H ||
                        left_edge[j].col < 0 ||
                        left_edge[j].col >= MT9V03X_W)
                    {
                        break;
                    }
                }

                if (j <= i + 4)
                    continue;

                /*
                 * A = left_edge[i - 4]
                 * B = left_edge[i]，候选拐点
                 * C = left_edge[i + 4]
                 *
                 * A、B、C都不能靠近左右图像边缘。
                 */
                if (left_edge[i - 4].col < edge_margin ||
                    left_edge[i - 4].col >= MT9V03X_W - edge_margin ||
                    left_edge[i].col < edge_margin ||
                    left_edge[i].col >= MT9V03X_W - edge_margin ||
                    left_edge[i + 4].col < edge_margin ||
                    left_edge[i + 4].col >= MT9V03X_W - edge_margin)
                {
                    continue;
                }

                /* A、C中行号较小的点作为远端点 */
                if (left_edge[i - 4].row < left_edge[i + 4].row)
                {
                    far_col = left_edge[i - 4].col;
                    far_row = left_edge[i - 4].row;
                }
                else
                {
                    far_col = left_edge[i + 4].col;
                    far_row = left_edge[i + 4].row;
                }

                /* 远端点必须位于候选点上方 */
                if (far_row >= left_edge[i].row)
                    continue;

                /* 左侧：候选点列号必须大于远端点列号 */
                if (left_edge[i].col >= far_col)
                    continue;

                /* 以B为起点，计算BA、BC */
                ux = (float)left_edge[i - 4].col
                   - (float)left_edge[i].col;
                uy = (float)left_edge[i - 4].row
                   - (float)left_edge[i].row;

                vx = (float)left_edge[i + 4].col
                   - (float)left_edge[i].col;
                vy = (float)left_edge[i + 4].row
                   - (float)left_edge[i].row;

                len_u2 = ux * ux + uy * uy;
                len_v2 = vx * vx + vy * vy;

                if (len_u2 < 0.001f || len_v2 < 0.001f)
                    continue;

                cos_angle = (ux * vx + uy * vy)
                          / sqrtf(len_u2 * len_v2);

                if (cos_angle >= cos_160 &&
                    cos_angle <= cos_100)
                {
                    corner_l_thick_far.row  = left_edge[i].row;
                    corner_l_thick_far.col  = left_edge[i].col;
                    corner_l_thick_far.flag = 1;
                    break;
                }
            }

            if (corner_l_thick_far.flag == 1)
                break;
        }
    }

    /* ---- 右侧远钝角拐点 ---- */
    corner_r_thick_far.flag = 0;
    corner_r_thick_far.row  = 0;
    corner_r_thick_far.col  = 0;

    if (right_edge_count >= 9)
    {
        for (row = min_row; row < MT9V03X_H; row++)
        {
            for (i = 4; i < right_edge_count - 4; i++)
            {
                if (right_edge[i].row != row)
                    continue;

                /* 九点窗口有效性检查 */
                for (j = i - 4; j <= i + 4; j++)
                {
                    if (right_edge[j].flag != 1 ||
                        right_edge[j].row < 0 ||
                        right_edge[j].row >= MT9V03X_H ||
                        right_edge[j].col < 0 ||
                        right_edge[j].col >= MT9V03X_W)
                    {
                        break;
                    }
                }

                if (j <= i + 4)
                    continue;

                /* A、B、C都不能靠近左右图像边缘 */
                if (right_edge[i - 4].col < edge_margin ||
                    right_edge[i - 4].col >= MT9V03X_W - edge_margin ||
                    right_edge[i].col < edge_margin ||
                    right_edge[i].col >= MT9V03X_W - edge_margin ||
                    right_edge[i + 4].col < edge_margin ||
                    right_edge[i + 4].col >= MT9V03X_W - edge_margin)
                {
                    continue;
                }

                /* A、C中行号较小的点作为远端点 */
                if (right_edge[i - 4].row < right_edge[i + 4].row)
                {
                    far_col = right_edge[i - 4].col;
                    far_row = right_edge[i - 4].row;
                }
                else
                {
                    far_col = right_edge[i + 4].col;
                    far_row = right_edge[i + 4].row;
                }

                /* 远端点必须位于候选点上方 */
                if (far_row >= right_edge[i].row)
                    continue;

                /* 右侧：候选点列号必须小于远端点列号 */
                if (right_edge[i].col <= far_col)
                    continue;

                /* 以B为起点，计算BA、BC */
                ux = (float)right_edge[i - 4].col
                   - (float)right_edge[i].col;
                uy = (float)right_edge[i - 4].row
                   - (float)right_edge[i].row;

                vx = (float)right_edge[i + 4].col
                   - (float)right_edge[i].col;
                vy = (float)right_edge[i + 4].row
                   - (float)right_edge[i].row;

                len_u2 = ux * ux + uy * uy;
                len_v2 = vx * vx + vy * vy;

                if (len_u2 < 0.001f || len_v2 < 0.001f)
                    continue;

                cos_angle = (ux * vx + uy * vy)
                          / sqrtf(len_u2 * len_v2);

                if (cos_angle >= cos_160 &&
                    cos_angle <= cos_100)
                {
                    corner_r_thick_far.row  = right_edge[i].row;
                    corner_r_thick_far.col  = right_edge[i].col;
                    corner_r_thick_far.flag = 1;
                    break;
                }
            }

            if (corner_r_thick_far.flag == 1)
                break;
        }
    }
}

//最小二乘法计算方差，左零右一，有待测试，暂时认为可用于左右环岛判断的辅助条件
float Camera_Edge_Fit_Variance(uint8 side)
{
    const my_camera_point_t *edge;
    uint16 count;
    uint16 i;
    uint16 n = 0;

    float sum_x = 0.0;
    float sum_y = 0.0;
    float sum_xx = 0.0;
    float sum_xy = 0.0;
    float sum_yy = 0.0;
    float x, y;
    float sxx, sxy, syy;
    float variance;

    if(side == 0)
    {
        edge = left_edge;
        count = left_edge_count;
    }
    else if(side == 1)
    {
        edge = right_edge;
        count = right_edge_count;
    }
    else
    {
        return -1.0f;
    }

    if(count > MY_CAMERA_EDGE_MAX)
    {
        count = MY_CAMERA_EDGE_MAX;
    }

    /* 只遍历一次，直接读取原数组 */
    for(i = 0; i < count; i++)
    {
        if(edge[i].flag != 1)
        {
            continue;
        }

        x = (double)edge[i].row;
        y = (double)edge[i].col;

        sum_x  += x;
        sum_y  += y;
        sum_xx += x * x;
        sum_xy += x * y;
        sum_yy += y * y;
        n++;
    }

    if(n < 2)
    {
        return -1.0f;
    }

    sxx = sum_xx - sum_x * sum_x / n;
    sxy = sum_xy - sum_x * sum_y / n;
    syy = sum_yy - sum_y * sum_y / n;

    /* 防止所有有效点行坐标相同时除零 */
    if(sxx <= 0.0)
    {
        return -1.0f;
    }

    variance = (syy - sxy * sxy / sxx) / n;

    /* 浮点舍入可能使理论上的 0 出现微小负值 */
    if(variance < 0.0)
    {
        variance = 0.0;
    }

    return (float)variance;
}


static hit_i last_hit = {0};
#define VALLEY_STEP_MIN   2U   /* 至少两次同向变化 */
#define VALLEY_FLAT_MAX   2U   /* 最多允许连续两次持平 */
//谷底检查函数，dire左0右1
static uint8 gudi_hanshu(uint8 dire, uint8 row_min, uint8 row_max)
{
    const my_camera_point_t *edge;
    uint16 count;
    uint16 i;

    int16 curr_row;
    int16 curr_col;
    int16 prev_col = 0;
    int16 diff;

    uint8 have_prev = 0U;
    uint8 state = 0U;          /* 0：正向尚未确认，1：正向已确认 */
    uint8 forward_count = 0U;
    uint8 reverse_count = 0U;
    uint8 flat_count = 0U;

    if ((dire != 0U && dire != 1U)|| row_min > row_max|| row_max >= MT9V03X_H)
    {
        return 0U;
    }

    /* 直接引用工程边线数组，不复制、不另建数组。 */
    if (dire == 0U)
    {
        edge = left_edge;
        count = left_edge_count;
    }
    else
    {
        edge = right_edge;
        count = right_edge_count;
    }

    if (count > MY_CAMERA_EDGE_MAX)
    {
        count = MY_CAMERA_EDGE_MAX;
    }

    for (i = 0U; i < count; i++)
    {
        curr_row = edge[i].row;

        if (edge[i].flag != 1U || curr_row < (int16)row_min || curr_row > (int16)row_max)
        {
            /* 禁止跨无效点或范围外的点累计趋势。 */
            have_prev = 0U;
            state = 0U;
            forward_count = 0U;
            reverse_count = 0U;
            flat_count = 0U;
            continue;
        }

        curr_col = edge[i].col;

        if (!have_prev)
        {
            prev_col = curr_col;
            have_prev = 1U;
            continue;
        }

        /*
         * 统一变化方向：
         * diff > 0：正向变化。
         * diff < 0：反向变化。
         */
        diff = (dire == 0U)
             ? ((int)curr_col - (int)prev_col)
             : ((int)prev_col - (int)curr_col);

        prev_col = curr_col;

        if (diff == 0)
        {
            flat_count++;

            if (flat_count > VALLEY_FLAT_MAX)
            {
                if (state == 0U)
                {
                    /* 正向尚未确认，清空正向累计。 */
                    forward_count = 0U;
                }
                else
                {
                    /*
                     * 保留已经确认的正向趋势，
                     * 只清空尚未确认的反向累计。
                     */
                    reverse_count = 0U;
                }

                flat_count = 0U;
            }

            continue;
        }

        flat_count = 0U;

        if (state == 0U)
        {
            if (diff > 0)
            {
                forward_count++;

                if (forward_count >= VALLEY_STEP_MIN)
                {
                    state = 1U;
                    reverse_count = 0U;
                }
            }
            else
            {
                /* 正向尚未确认，反向变化打断累计。 */
                forward_count = 0U;
            }
        }
        else
        {
            if (diff < 0)
            {
                reverse_count++;

                if (reverse_count >= VALLEY_STEP_MIN)
                {
                    /* 保存结构体中的实际坐标，不保存数组下标。 */
                    pro_point.row = (uint16)curr_row;
                    pro_point.col = (uint16)curr_col;
                    pro_point.flag = 1U;
                    return 1U;
                }
            }
            else
            {
                /* 保留正向确认状态，重新累计反向变化。 */
                reverse_count = 0U;
            }
        }
    }

    return 0U;
}


//统计贴边的点的个数，左0右1，返回值为个数
uint16 Camera_Edge_Border_Count(uint8 side)
{
    const my_camera_point_t *edge;
    uint16 count;
    uint16 i;
    uint16 result = 0;
    int16 min_col;
    int16 max_col;

    if(side == 0)
    {
        edge = left_edge;
        count = left_edge_count;
        min_col = 0;
        max_col = 2;
    }
    else if(side == 1)
    {
        edge = right_edge;
        count = right_edge_count;
        min_col = (int16)(MT9V03X_W - 3);
        max_col = (int16)(MT9V03X_W - 1);
    }
    else
    {
        return 0;
    }

    if(count > MY_CAMERA_EDGE_MAX)
    {
        count = MY_CAMERA_EDGE_MAX;
    }

    for(i = 0; i < count; i++)
    {
        if(edge[i].flag == 1 &&
           edge[i].col >= min_col &&
           edge[i].col <= max_col)
        {
            result++;
        }
    }

    return result;
}
//

/*
 * 判断单侧贴边线段是否连续。
 * side：0 左侧，1 右侧
 * 返回：1 连续，0 不连续（或没有合格线段、参数错误）
 */
/* 定义在函数外；其他文件需要读取时，在头文件添加 extern 声明。 */
uint8 left_border_continuous = 0;
uint8 right_border_continuous = 0;
uint8 Camera_Edge_Border_Continuous(uint8 side)
{
    const my_camera_point_t *edge;
    uint16 count;
    uint16 i;
    int16 point_row;
    int16 min_col;
    int16 max_col;

    uint8 border_row[MT9V03X_H] = {0};

    uint16 row;
    uint16 run_len = 0;            /* 当前连续贴边行数 */
    uint16 segment_count = 0;      /* 合格贴边线段数量 */
    uint16 previous_end = 0;       /* 上一条合格线段的结束行 */
    uint16 current_start;
    uint16 gap;
    uint16 min_gap = MT9V03X_H;    /* 合格线段之间的最小间隔 */

    if(side == 0)
    {
        edge = left_edge;
        count = left_edge_count;
        min_col = 0;
        max_col = 2;
    }
    else if(side == 1)
    {
        edge = right_edge;
        count = right_edge_count;
        min_col = (int16)(MT9V03X_W - 3);
        max_col = (int16)(MT9V03X_W - 1);
    }
    else
    {
        return 0;
    }

    if(count > MY_CAMERA_EDGE_MAX)
    {
        count = MY_CAMERA_EDGE_MAX;
    }

    /* 按行标记贴边点：去重，并消除爬线顺序的影响。 */
    for(i = 0; i < count; i++)
    {
        point_row = edge[i].row;

        if(edge[i].flag == 1 &&
           point_row >= 0 &&
           point_row < MT9V03X_H &&
           edge[i].col >= min_col &&
           edge[i].col <= max_col)
        {
            border_row[point_row] = 1;
        }
    }

    /*
     * 多扫描一次，用于结算一直延伸到最后一行的线段。
     * && 短路判断保证 border_row 不会越界访问。
     */
    for(row = 0; row <= MT9V03X_H; row++)
    {
        if(row < MT9V03X_H && border_row[row] == 1)
        {
            run_len++;
        }
        else
        {
            if(run_len >= 5)
            {
                current_start = row - run_len;

                if(segment_count > 0)
                {
                    gap = current_start - previous_end - 1;

                    if(gap < min_gap)
                    {
                        min_gap = gap;
                    }
                }

                previous_end = row - 1;
                segment_count++;
            }

            run_len = 0;
        }
    }

    if(segment_count == 0)
    {
        return 0;
    }

    if(segment_count == 1)
    {
        return 1;
    }

    return (min_gap <= 5) ? 1 : 0;
}


//测试最小图像处理
void camera_fr(void)
{
    uint8 threshold;
    threshold = camera_otsu_threshold(mt9v03x_image[0]);//大津法得到阈值
    camera_binarize(threshold);//将阈值修改入image数组
    camera_draw_border();//为image图像画黑框
    camera_find_seed();//找到爬线种子，存入camera_seed_t左右结构体中
    camera_search_neighborhood();//八邻域爬边线，爬出的边线，放入数组left_edge和right_edge中
    simple_roi_filter(20);
    get_turning_point();
    get_far_turning_point();
    gudi_hanshu(0, 20, 100);
    fangcha_left=Camera_Edge_Fit_Variance(0);
    fangcha_right=Camera_Edge_Fit_Variance(1);
    left_eg_loss=Camera_Edge_Border_Count(0);
    right_eg_loss=Camera_Edge_Border_Count(1);
    camera_build_center_line();
    camera_update_center_error_by_mileage();
    camera_frame_count++;
    camera_frame_id++;
}
//


//图像处理流程测试版
void Camera_Process_Frame(void)
{
    switch(current_element)
    {
        case ELEM_NORMAL:
        {
            uint8 threshold;
            threshold = camera_otsu_threshold(mt9v03x_image[0]);//大津法得到阈值
            camera_binarize(threshold);//将阈值修改入image数组
            camera_draw_border();//为image图像画黑框
            camera_find_seed();//找到爬线种子，存入camera_seed_t左右结构体中
            camera_search_neighborhood();//八邻域爬边线，爬出的边线，放入数组left_edge和right_edge中
            simple_roi_filter(30);
            get_turning_point();
            camera_build_center_line();
            camera_update_center_error_by_mileage();
            camera_frame_count++;
            camera_frame_id++;
            break;
        }
        case ELEM_RIGHT_ROUNDABOUT:
                {
                    switch(c_ele_rt)
                    {
                        case ELE_RIGHT_ONE:
                        {
                            uint8 threshold;
                            threshold = camera_otsu_threshold(mt9v03x_image[0]);
                            camera_binarize(threshold);
                            camera_draw_border();
                            camera_find_seed();
                            camera_search_neighborhood();
                            simple_roi_filter(60);
                            fangcha_left=Camera_Edge_Fit_Variance(0);//方差检测
                            fangcha_right=Camera_Edge_Fit_Variance(1);
                            left_eg_loss=Camera_Edge_Border_Count(0);//左丢线检测确认
                            right_eg_loss=Camera_Edge_Border_Count(1);//右丢线检测确认
                            camera_build_center_line();
                            camera_update_center_error(70,50);
                            camera_frame_count++;
                            camera_frame_id++;
                            break;
                        }
                        case ELE_RIGHT_TWO:
                        {
                            uint8 threshold;
                            threshold = camera_otsu_threshold(mt9v03x_image[0]);
                            camera_binarize(threshold);
                            camera_draw_border();
                            camera_find_seed();
                            camera_search_neighborhood();
                            simple_roi_filter(40);//改变搜索前瞻
                            get_turning_point();//拐点消失确认
                            pro_point.flag=gudi_hanshu(1,40,100);//谷底点搜索
                            bresenham_right_line(144,0,187,119);//固定补线
                            camera_build_center_line();
                            camera_update_center_error(70,50);
                            camera_frame_count++;
                            camera_frame_id++;
                            break;
                        }
                        case ELE_RIGHT_THR:
                        {
                            uint8 threshold;
                            threshold = camera_otsu_threshold(mt9v03x_image[0]);
                            camera_binarize(threshold);
                            camera_draw_border();
                            camera_find_seed();
                            camera_search_neighborhood();
                            simple_roi_filter(20);
                            left_eg_loss=Camera_Edge_Border_Count(0);//入环左丢线确认
                            right_eg_loss=Camera_Edge_Border_Count(1);//入环右丢线确认
                            camera_build_center_line();
                            camera_update_center_error(70,50);
                            camera_frame_count++;
                            camera_frame_id++;
                            break;
                        }
                        case ELE_RIGHT_FOUR:
                        {
                            uint8 threshold;
                            threshold = camera_otsu_threshold(mt9v03x_image[0]);
                            camera_binarize(threshold);
                            camera_draw_border();
                            camera_find_seed();
                            camera_search_neighborhood();
                            simple_roi_filter(20);
                            bresenham_left_line(MT9V03X_W-41,0,0,119);//固定补线
                            camera_build_center_line();
                            camera_update_center_error(70,50);
                            camera_frame_count++;
                            camera_frame_id++;
                            break;
                        }
                        case ELE_RIGHT_FIVE:
                        {
                            uint8 threshold;
                            threshold = camera_otsu_threshold(mt9v03x_image[0]);
                            camera_binarize(threshold);
                            camera_draw_border();
                            camera_find_seed();
                            camera_search_neighborhood();
                            simple_roi_filter(20);
                            get_turning_point();//出环拐点检测
                            camera_build_center_line();
                            camera_update_center_error(70,50);
                            camera_frame_count++;
                            camera_frame_id++;
                            break;
                        }
                        case ELE_RIGHT_SIX:
                        {
                            uint8 threshold;
                            threshold = camera_otsu_threshold(mt9v03x_image[0]);
                            camera_binarize(threshold);
                            camera_draw_border();
                            camera_find_seed();
                            camera_search_neighborhood();
                            simple_roi_filter(20);
                            get_turning_point();//出环拐点消失
                            camera_build_center_line();
                            camera_update_center_error(70,50);
                            camera_frame_count++;
                            camera_frame_id++;
                            break;
                        }
                        case ELE_RIGHT_SEVEN:
                        {
                            uint8 threshold;
                            threshold = camera_otsu_threshold(mt9v03x_image[0]);
                            camera_binarize(threshold);
                            camera_draw_border();
                            camera_find_seed();
                            camera_search_neighborhood();
                            simple_roi_filter(20);
                            bresenham_left_line(177,0,0,119);//出环固定补线
                            camera_build_center_line();
                            camera_update_center_error(70,50);
                            camera_frame_count++;
                            camera_frame_id++;
                            break;
                        }
                        case ELE_RIGHT_ET:
                        {
                            uint8 threshold;
                            threshold = camera_otsu_threshold(mt9v03x_image[0]);
                            camera_binarize(threshold);
                            camera_draw_border();
                            camera_find_seed();
                            camera_search_neighborhood();
                            simple_roi_filter(20);
                            bresenham_right_line(100,0,187,119);//引导直道补线
                            camera_build_center_line();
                            camera_update_center_error(70,50);
                            camera_frame_count++;
                            camera_frame_id++;
                            break;
                        }
                        default:
                            current_element=ELEM_NORMAL;
                            c_ele_rt=ELE_RIGHT_ONE;
                            break;
                    }
                    break;
                }
            case ELEM_LEFT_ROUNDABOUT:
            {
                switch(c_ele_lf)
                {
                    case ELE_LEFT_ONE:
                    {
                        uint8 threshold;
                        threshold = camera_otsu_threshold(mt9v03x_image[0]);//大津法得到阈值
                        camera_binarize(threshold);//将阈值修改入image数组
                        camera_draw_border();//为image图像画黑框
                        camera_find_seed();//找到爬线种子，存入camera_seed_t左右结构体中
                        camera_search_neighborhood();//八邻域爬边线，爬出的边线，放入数组left_edge和right_edge中
                        simple_roi_filter(20);
                        get_turning_point();
                        fangcha_left=Camera_Edge_Fit_Variance(0);
                        fangcha_right=Camera_Edge_Fit_Variance(1);
                        left_eg_loss=Camera_Edge_Border_Count(0);
                        right_eg_loss=Camera_Edge_Border_Count(1);
                        camera_build_center_line();//根据边线数组计算出中线坐标，放入数组center_line中
                        camera_update_center_error(70,68);
                        camera_frame_count++;
                        camera_frame_id++;
                        break;
                    }
                    case ELE_LEFT_TWO:
                    {
                        uint8 threshold;
                        threshold = camera_otsu_threshold(mt9v03x_image[0]);//大津法得到阈值
                        camera_binarize(threshold);//将阈值修改入image数组
                        camera_draw_border();//为image图像画黑框
                        camera_find_seed();//找到爬线种子，存入camera_seed_t左右结构体中
                        camera_search_neighborhood();//八邻域爬边线，爬出的边线，放入数组left_edge和right_edge中
                        simple_roi_filter(10);
                        get_turning_point();
//                        pro_point.flag = gudi_hanshu(0, 30, 119);
                        left_border_continuous = Camera_Edge_Border_Continuous(0);
                        camera_build_center_line();//根据边线数组计算出中线坐标，放入数组center_line中
                        camera_update_center_error(70,68);
                        camera_frame_count++;
                        camera_frame_id++;
                        break;
                    }
                    case ELE_LEFT_THR:
                    {
                        uint8 threshold;
                        threshold = camera_otsu_threshold(mt9v03x_image[0]);//大津法得到阈值
                        camera_binarize(threshold);//将阈值修改入image数组
                        camera_draw_border();//为image图像画黑框
                        camera_find_seed();//找到爬线种子，存入camera_seed_t左右结构体中
                        camera_search_neighborhood();//八邻域爬边线，爬出的边线，放入数组left_edge和right_edge中
                        simple_roi_filter(20);
                        pro_point.flag=gudi_hanshu(0,20,100);
                        camera_build_center_line();//根据边线数组计算出中线坐标，放入数组center_line中
                        camera_update_center_error(70,68);
                        camera_frame_count++;
                        camera_frame_id++;
                        break;
                    }
                    case ELE_LEFT_THR_P:
                    {
                        uint8 threshold;
                        threshold = camera_otsu_threshold(mt9v03x_image[0]);//大津法得到阈值
                        camera_binarize(threshold);//将阈值修改入image数组
                        camera_draw_border();//为image图像画黑框
                        camera_find_seed();//找到爬线种子，存入camera_seed_t左右结构体中
                        camera_search_neighborhood();//八邻域爬边线，爬出的边线，放入数组left_edge和right_edge中
                        simple_roi_filter(20);
                        pro_point.flag=gudi_hanshu(0,20,100);
                        if(pro_point.flag== 1)
                         {
                            last_hit = pro_point;
                         }
                         if(last_hit.flag == 1)
                         {
                             bresenham_left_line(last_hit.col,last_hit.row,0,119);
                         }
                        camera_build_center_line();//根据边线数组计算出中线坐标，放入数组center_line中
                        camera_update_center_error(70,68);
                        camera_frame_count++;
                        camera_frame_id++;
                        break;
                    }
                    case ELE_LEFT_FOUR:
                    {
                        uint8 threshold;
                        threshold = camera_otsu_threshold(mt9v03x_image[0]);//大津法得到阈值
                        camera_binarize(threshold);//将阈值修改入image数组
                        camera_draw_border();//为image图像画黑框
                        camera_find_seed();//找到爬线种子，存入camera_seed_t左右结构体中
                        camera_search_neighborhood();//八邻域爬边线，爬出的边线，放入数组left_edge和right_edge中
                        simple_roi_filter(20);
                        get_far_turning_point();
                        camera_build_center_line();//根据边线数组计算出中线坐标，放入数组center_line中
                        camera_update_center_error(70,68);
                        camera_frame_count++;
                        camera_frame_id++;
                        break;
                    }
                    case ELE_LEFT_FIVE:
                    {
                        uint8 threshold;
                        threshold = camera_otsu_threshold(mt9v03x_image[0]);//大津法得到阈值
                        camera_binarize(threshold);//将阈值修改入image数组
                        camera_draw_border();//为image图像画黑框
                        camera_find_seed();//找到爬线种子，存入camera_seed_t左右结构体中
                        camera_search_neighborhood();//八邻域爬边线，爬出的边线，放入数组left_edge和right_edge中
                        simple_roi_filter(20);
                        get_far_turning_point();
                        if(corner_l_thick_far.flag == 1)
                        {
                            last_left_corner_far = corner_l_thick_far;
                        }
                        if(last_left_corner_far.flag == 1)
                        {
                            bresenham_right_line(last_left_corner_far.col,last_left_corner_far.row,187, 119);
                        }
                        camera_build_center_line();//根据边线数组计算出中线坐标，放入数组center_line中
                        camera_update_center_error(45,40);
                        camera_frame_count++;
                        camera_frame_id++;
                        break;
                    }
                    case ELE_LEFT_SIX:
                    {
                        uint8 threshold;
                        threshold = camera_otsu_threshold(mt9v03x_image[0]);//大津法得到阈值
                        camera_binarize(threshold);//将阈值修改入image数组
                        camera_draw_border();//为image图像画黑框
                        camera_find_seed();//找到爬线种子，存入camera_seed_t左右结构体中
                        camera_search_neighborhood();//八邻域爬边线，爬出的边线，放入数组left_edge和right_edge中
                        simple_roi_filter(20);
                        get_turning_point();
                        camera_build_center_line();//根据边线数组计算出中线坐标，放入数组center_line中
                        camera_update_center_error(45,40);
                        camera_frame_count++;
                        camera_frame_id++;
                        break;
                    }
                    case ELE_LEFT_SEVEN:
                    {
                        uint8 threshold;
                        threshold = camera_otsu_threshold(mt9v03x_image[0]);//大津法得到阈值
                        camera_binarize(threshold);//将阈值修改入image数组
                        camera_draw_border();//为image图像画黑框
                        camera_find_seed();//找到爬线种子，存入camera_seed_t左右结构体中
                        camera_search_neighborhood();//八邻域爬边线，爬出的边线，放入数组left_edge和right_edge中
                        simple_roi_filter(20);
                        get_turning_point();
                        if(corner_r_thick.flag == 1)
                        {
                            last_right_corner = corner_r_thick;
                        }
                        if(last_right_corner.flag == 1)
                        {
                            bresenham_right_line(last_right_corner.col,last_right_corner.row,187, 119);
                        }
                        camera_build_center_line();//根据边线数组计算出中线坐标，放入数组center_line中
                        camera_update_center_error(45,40);
                        camera_frame_count++;
                        camera_frame_id++;
                        break;
                    }
                    case ELE_LEFT_SEVEN_P:
                    {
                        uint8 threshold;
                        threshold = camera_otsu_threshold(mt9v03x_image[0]);//大津法得到阈值
                        camera_binarize(threshold);//将阈值修改入image数组
                        camera_draw_border();//为image图像画黑框
                        camera_find_seed();//找到爬线种子，存入camera_seed_t左右结构体中
                        camera_search_neighborhood();//八邻域爬边线，爬出的边线，放入数组left_edge和right_edge中
                        simple_roi_filter(20);
                        bresenham_right_line(10,0,187,119);
                        camera_build_center_line();//根据边线数组计算出中线坐标，放入数组center_line中
                        camera_update_center_error(40,35);
                        camera_frame_count++;
                        camera_frame_id++;
                        break;
                    }
                    case ELE_LEFT_ET:
                    {
                        uint8 threshold;
                        threshold = camera_otsu_threshold(mt9v03x_image[0]);//大津法得到阈值
                        camera_binarize(threshold);//将阈值修改入image数组
                        camera_draw_border();//为image图像画黑框
                        camera_find_seed();//找到爬线种子，存入camera_seed_t左右结构体中
                        camera_search_neighborhood();//八邻域爬边线，爬出的边线，放入数组left_edge和right_edge中
                        simple_roi_filter(20);
                        bresenham_left_line(90,0,0,119);
                        camera_build_center_line();//根据边线数组计算出中线坐标，放入数组center_line中
                        camera_update_center_error(45,40);
                        camera_frame_count++;
                        camera_frame_id++;
                        break;
                    }
                    default:
                        current_element=ELEM_NORMAL;
                        c_ele_lf=ELE_LEFT_ONE;
                        break;
                }
                break;
        }
        default:
            current_element=ELEM_NORMAL;
            break;
    }
}
//


//左状态机
static void Left_Roundabout_StateMachine(void)
{
    static uint8 satisfy_count = 0;  // 连续满足条件的帧数
    static uint8 fail_count = 0;     // 连续不满足条件的帧数
    switch(c_ele_lf)
    {
        case ELE_LEFT_ONE:
        {
            if((left_count_f>=100000) && (fangcha_left - fangcha_right) >= 50.0f && fangcha_right<=70.0f && fangcha_right>=0&& right_eg_loss<=60)//第一次50,70在弯道出现误判,(left_count_f>=117398) && (fangcha_left - fangcha_right) >= 50.0f && fangcha_right<=70.0f && fangcha_right>=0&& right_eg_loss<=60
            {
                fail_count = 0;
                satisfy_count+=1;
                if(satisfy_count >= 3)
                {
                    last_left_corner.flag = 0;
                    c_ele_lf = ELE_LEFT_TWO;
                    Beep_ShortRing();
                    satisfy_count = 0;
                    fail_count = 0;
                    left_speed_flag=1;
                }
            }
            else
            {
                satisfy_count = 0;
                fail_count+=1;
                if(fail_count >= 5)
                {
                    c_ele_lf = ELE_LEFT_ONE;
                    current_element=ELEM_NORMAL;
                    satisfy_count = 0;
                    fail_count = 0;
                }
            }
            break;
        }
//        case ELE_LEFT_TWO:
//        {
//            uint8 corner_is_valley = 0;
//            uint8 blocking_corner;
//            int16 row_diff;
//            int16 col_diff;
//
//            if(corner_l_thick.flag == 1 && pro_point.flag == 1)
//            {
//                row_diff = (int16)corner_l_thick.row
//                         - (int16)pro_point.row;
//                col_diff = (int16)corner_l_thick.col
//                         - (int16)pro_point.col;
//
//                /*
//                 * 初始匹配范围：行差±6、列差±8。
//                 * 需要根据实际图像调整。
//                 */
//                if(row_diff >= -6 && row_diff <= 6 &&
//                   col_diff >= -8 && col_diff <= 8)
//                {
//                    corner_is_valley = 1;
//                }
//            }
//
//            /* 只有35行以下、且未被判为谷底的拐点才阻止跳转。 */
//            blocking_corner =
//                (corner_l_thick.flag == 1 &&
//                 corner_l_thick.row > 35 &&
//                 corner_is_valley == 0);
//
//            if(blocking_corner == 0 && left_border_continuous == 1)
//            {
//                satisfy_count++;
//
//                if(satisfy_count >= 3)
//                {
//                    c_ele_lf = ELE_LEFT_THR;
//                    satisfy_count = 0;
//                }
//            }
//            else
//            {
//                satisfy_count = 0;
//            }
//
//            break;
//        }
        case ELE_LEFT_TWO:
        {
            if((corner_l_thick.flag == 0 || corner_l_thick.row<=35)&& left_border_continuous==1)//第一次行号为20，过于容易进入状态，暂改为40  pro_point.flag == 1 &&
            {
                satisfy_count+=1;
                if(satisfy_count>=3)
                {
                    c_ele_lf=ELE_LEFT_THR;
                    satisfy_count=0;
                }
            }
            else
            {
                satisfy_count=0;
            }
            break;
        }

        case ELE_LEFT_THR:
        {
            if(pro_point.flag == 1)
            {
                satisfy_count++;
                if(satisfy_count >= 3)
                {
                    //保存切换前最后一帧的有效谷底点
                    last_hit = pro_point;

                    c_ele_lf = ELE_LEFT_THR_P;
                    satisfy_count = 0;
                }
            }
            else
            {
                satisfy_count = 0;
            }
            break;
        }
        case ELE_LEFT_THR_P:
        {

            if (pro_point.flag == 0)
            {
                satisfy_count++;
                if (satisfy_count >= 3)
                {
                    c_ele_lf = ELE_LEFT_FOUR;
                    satisfy_count = 0;
                }
            }
            else
            {
                satisfy_count = 0;
            }
            break;
        }
        case ELE_LEFT_FOUR:
        {
            if(corner_l_thick_far.flag == 1)
            {
                satisfy_count++;
                if(satisfy_count >= 3)
                {
                    //保存切换前最后一帧的有效左远拐点
                    last_left_corner_far = corner_l_thick_far;

                    c_ele_lf = ELE_LEFT_FIVE;
                    satisfy_count = 0;
                }
            }
            else
            {
                satisfy_count = 0;
            }
            break;
        }
        case ELE_LEFT_FIVE:
        {
            if(corner_l_thick_far.flag==0)
            {
                satisfy_count+=1;
                if(satisfy_count>=3)
                {
                    c_ele_lf=ELE_LEFT_SIX;
                    satisfy_count=0;
                }
            }
            else
            {
                satisfy_count=0;
            }
            break;
        }
        case ELE_LEFT_SIX:
        {
            if(corner_r_thick.flag == 1&&corner_r_thick.col>=94&&corner_r_thick.row>=40)
            {
                satisfy_count++;
                if(satisfy_count >= 3)
                {
                    //保存切换前最后一帧的有效右近拐点
                    last_right_corner = corner_r_thick;
                    c_ele_lf = ELE_LEFT_SEVEN;
                    Beep_ShortRing();
                    satisfy_count = 0;
                }
            }
            else
            {
                satisfy_count = 0;
            }
            break;
        }
        case ELE_LEFT_SEVEN:
        {

            if(corner_r_thick.flag==0)
            {
                satisfy_count+=1;
                if(satisfy_count>=3)
                {
                    c_ele_lf=ELE_LEFT_SEVEN_P;
                    satisfy_count=0;
                }
            }
            else
            {
                satisfy_count=0;
            }
             break;
        }
        case ELE_LEFT_SEVEN_P:
        {
             satisfy_count+=1;
             if(satisfy_count>=50)
             {
                 c_ele_lf=ELE_LEFT_ET;
                 satisfy_count=0;
             }
             break;
        }
        case ELE_LEFT_ET:
        {
             satisfy_count+=1;
             if(satisfy_count>=60)
             {
                 left_roundabout_done=1;
                 zuocebiaozhi=1;
                 c_ele_lf=ELE_LEFT_ONE;
                 current_element=ELEM_NORMAL;
                 satisfy_count=0;
                 left_speed_flag=0;
             }
             break;
        }
        default:
        {
            satisfy_count = 0;
            fail_count = 0;
            c_ele_lf=ELE_LEFT_ONE;
            break;
        }
    }
}
//


//右状态机
static void Right_Roundabout_StateMachine(void)
{
    static uint8 satisfy_count = 0;  // 连续满足条件的帧数
    static uint8 fail_count = 0;     // 连续不满足条件的帧数
    switch(c_ele_rt)
    {
        case ELE_RIGHT_ONE:
        {
            if((right_eg_loss>=50 && left_eg_loss<=45 && fangcha_left<=100.0f)||(left_count_f>=220000))//第一次50,70在弯道出现误判,可以确认是左线方差问题导致圆环的识别问题right_eg_loss>=50 && left_eg_loss<=45 && fangcha_left<=100.0f
            {
                fail_count = 0;
                satisfy_count+=1;
                if(satisfy_count >= 3)
                {
                    c_ele_rt = ELE_RIGHT_TWO;
                    Beep_ShortRing();
                    satisfy_count = 0;
                    fail_count = 0;
                    right_speed_flag=1;
                }
            }
            else
            {
                satisfy_count = 0;
                fail_count+=1;
                if(fail_count >= 20)//先改为20帧，看看效果，无法满足则将右侧圆环的判定改为丢线判定
                {
                    c_ele_rt = ELE_RIGHT_ONE;
                    current_element=ELEM_NORMAL;
                    satisfy_count = 0;
                    fail_count = 0;
                }
            }
            break;
        }

        case ELE_RIGHT_TWO:
        {
            if(pro_point.flag == 1 && corner_r_thick.flag == 0)//第一次行号为20，过于容易进入状态，暂改为40
            {
                satisfy_count+=1;
                if(satisfy_count>=3)
                {
                    c_ele_rt=ELE_RIGHT_THR;
                    satisfy_count=0;

                }
            }
            else
            {
                satisfy_count=0;
            }
            break;
        }

        case ELE_RIGHT_THR:
        {

            if((left_eg_loss<=50 && right_eg_loss>=75) )//丢线统计暂时没有具体数据(left_eg_loss<=50 && right_eg_loss>=85)
            {
                satisfy_count+=1;
                if(satisfy_count>=3)
                {
                    c_ele_rt=ELE_RIGHT_FOUR;
                    satisfy_count=0;
                }
            }
            else
            {
                satisfy_count=0;
            }
            break;
        }
        case ELE_RIGHT_FOUR:
        {
            satisfy_count+=1;
            if(satisfy_count>=30)//180过大
            {
                c_ele_rt=ELE_RIGHT_FIVE;
                satisfy_count=0;
            }
            break;
        }
        case ELE_RIGHT_FIVE:
        {
            if(corner_l_thick.flag==1&&corner_l_thick.col>=94&&corner_l_thick.row>=40)
            {
                satisfy_count+=1;
                if(satisfy_count>=3)
                {
                    c_ele_rt=ELE_RIGHT_SIX;
                    satisfy_count=0;
                }
            }
            else
            {
                satisfy_count=0;
            }
            break;
        }
        case ELE_RIGHT_SIX:
        {
            if(corner_l_thick.flag==0)
            {
                satisfy_count+=1;
                if(satisfy_count>=3)
                {
                    c_ele_rt=ELE_RIGHT_SEVEN;
                    satisfy_count=0;
                }
            }
            else
            {
                satisfy_count=0;
            }
            break;
        }
        case ELE_RIGHT_SEVEN:
        {
             satisfy_count+=1;
             if(satisfy_count>=60)
             {
                 c_ele_rt=ELE_RIGHT_ET;
                 satisfy_count=0;
             }
             break;
        }
        case ELE_RIGHT_ET:
        {
             satisfy_count+=1;
             if(satisfy_count>=200)
             {
                 right_roundabout_done = 1;
                 c_ele_rt=ELE_RIGHT_ONE;
                 current_element=ELEM_NORMAL;
                 satisfy_count=0;
                 right_speed_flag=0;
             }
             break;
        }
        default:
        {
            satisfy_count = 0;
            fail_count = 0;
            c_ele_rt=ELE_RIGHT_ONE;
            break;
        }
    }
}
//



//主状态机
void Track_Main_StateMachine(void)
{
    static uint8 left_enter_count = 0;
    static uint8 right_enter_count = 0;

    switch(current_element)
    {
        case ELEM_NORMAL:
        {
            if(left_roundabout_done == 0 && corner_l_thick.flag == 1 && corner_r_thick.flag == 0)//&& corner_l_thick.row>=55  (left_count_f>=100000) &&
            {
                right_enter_count = 0;
                left_enter_count++;

                if(left_enter_count >= 3)
                {
                    current_element = ELEM_LEFT_ROUNDABOUT;
                    c_ele_lf = ELE_LEFT_ONE;
                    left_enter_count = 0;
                }
            }
            else if(zuocebiaozhi==1  && right_roundabout_done == 0 && corner_r_thick.flag == 1 && corner_l_thick.flag == 0)//zuocebiaozhi==1&&right_roundabout_done == 0
            {
                left_enter_count = 0;
                right_enter_count++;

                if(right_enter_count >= 3)
                {
                    current_element = ELEM_RIGHT_ROUNDABOUT;
                    c_ele_rt = ELE_RIGHT_ONE;
                    right_enter_count = 0;
                }
            }

            else
            {
                left_enter_count = 0;
                right_enter_count = 0;
            }

            break;
        }

        case ELEM_LEFT_ROUNDABOUT:
        {
            Left_Roundabout_StateMachine();
            break;
        }

        case ELEM_RIGHT_ROUNDABOUT:
        {
            Right_Roundabout_StateMachine();
            break;
        }

        default:
        {
            current_element = ELEM_NORMAL;
            left_enter_count = 0;
            right_enter_count = 0;
            break;
        }
    }
}
//









const element_type Camera_Get_Element(void){return current_element;}
const ele_left Camera_Get_Left_Stage(void){return c_ele_lf;}
const ele_right Camera_Get_Right_Stage(void){return c_ele_rt;}


const uint8 *Camera_Get_Binary_Image(void)
{
    return image[0];
}

const my_camera_point_t *Camera_Get_Left_Edge(uint16 *count)//该函数需要两个返回值，边界点个数和边界点数组的地址，下面几个函数同理
{
    if(count != NULL)//防止放入的是空指针
    {
        *count = left_edge_count;
    }

    return left_edge;
}

const my_camera_point_t *Camera_Get_Right_Edge(uint16 *count)
{
    if(count != NULL)//防止放入的是空指针
    {
        *count = right_edge_count;
    }

    return right_edge;
}

const my_camera_point_t *Camera_Get_Center_Line(uint16 *count)
{
    if(count != NULL)//防止放入的是空指针
    {
        *count = center_line_count;
    }

    return center_line;
}
