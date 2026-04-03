  #include "zf_common_headfile.h"

#define White 255
#define Black 0

#define Left 1
#define Right 2
int hengduan_flag=0;
double zhonxianpiancha;
int you_err = 25;//右出环偏差
int image_close=0;
extern uint16 dl1b_distance_mm;
int x,y;
int er;
int flag1;//坡道标志
int right_flage;
int left_flage;
//extern int flag11;//执行完坡道一次标志位

extern float gyro_lpf;//角速度
extern float gyro_lpf_z;//角速度
int danbianqiaowupan=0;
extern uint8 zebra_stop_flag;
float parameterB1,parameterA1;   //y=parameterB1*x+parameterA1
float k;
float b;


//zx[h]=left_border;//左边界数组
//yx[h]=right_border;//右边界数组
uint8 left[image_h]={2};
uint8 right[image_h]={185};
uint8 middle[image_h]={94};
uint8 Endline=1;
uint8 WhiteNum=0;
uint8 X1,Y1;//左下补线点（圆环）
uint8 right_lost_num=0;
uint8 left_lost_num=0;
uint8 imag[image_h][image_w];
uint8 Threshold1_value=225;
uint32 image_process_time=0;   //图像处理时间

uint8 Right_straight_flag=0; //右直线
uint8 Left_straight_flag=0; //左直线

uint8 annulus_L_Flag=0;       //左圆环
uint8 annulus_R_Flag=0;       //右圆环
uint8 annulus_L_memory=0;     //左圆环计步
uint8 annulus_R_memory=0;     //右圆环计步
int annulus_out =0;

uint8 zebra_crossing_flag=0;//斑马线
uint8 zebra_flag_end=0;
uint8 Left_garage_flag=0; //左车库
uint8 Left_garage_memory=0;//左车库计步

//圆环凸起点
uint8 roundabout_X=0;
uint8 roundabout_Y=0;
uint8 roundabout_Flag=0;

//出环识别点
uint8 Exit_loop_X=0;
uint8 Exit_loop_Y=0;
uint8 Exit_loop_Flag=0;

uint8 Crossroad_Flag=0;      //十字
uint8 Crossroad_memory=0;     //十字计步
uint8 Finish_Flag=0; //处理完成标识位
//丢线
uint8 Lost_left_Flag=0;
uint8 Lost_right_Flag=0;
uint8 Lost_point_L_scan_line=0;
uint8 Lost_point_R_scan_line=0;

//左下拐点
uint8 Lower_left_inflection_X =0;
uint8 Lower_left_inflection_Y =0;
uint8 Lower_left_inflection_Flag=0;

//右下拐点
uint8 Lower_right_inflection_X =0;
uint8 Lower_right_inflection_Y =0;
uint8 Lower_right_inflection_Flag=0;

//左上拐点
uint8 Upper_left_inflection_X =0;
uint8 Upper_left_inflection_Y =0;
uint8 Upper_left_inflection_Flag=0;

//右上拐点
uint8 Upper_right_inflection_X =0;
uint8 Upper_right_inflection_Y =0;
uint8 Upper_right_inflection_Flag=0;

//斑马线停车
uint8 Encoder_adding = 0;

int start_time_flag=0,start_time1=0;                             //起步加速标志位
int mid_road_time = 0,mid_road_time_stop=0,mid_road_flag=0;      //中途加速标志位
int over_road_time= 0,over_road_time_stop=0,over_road_flag = 0;  //结尾加速标志位
int stop_flag_time;              //停车延时时间
int stop_time =0,stop_time_flag = 0;

float Desired_speed_change = 0;          //一直为0，不能更改，消除积分累计
int start_time=0;
int delete_in=0;
int turn_on_camera =0 ;
int interrupt_flag = 0,mode_interrupt=0;
int  Start_Flag = 0;
int Camera_display_flag = 0;
int flag_parameter = 0;
int camera_start = 0;

//检测弯道
uint8 Check_left_flag=0;
uint8 Check_right_flag=0;
uint8 Check_straight_flag=0;
uint8 Check_obstacle_flag=0;//小障碍标志位
int turnleft_flag =0;
int turnright_flag =0;
uint8 Check_obstacle_L_flag = 0;
uint8 Check_obstacle_R_flag = 0;

int16 L_corner_flag1 = 0;//左拐点存在标志
int16 L_corner_row1 = 0;//左拐点所在行
int16 L_corner_col1 = 0;//左拐点所在列
extern float L_corner_angle1 = 0;//左拐点角度
int L_corner_angle1_1 = 0;//左拐点角度
int16 R_corner_flag1 = 0;//右拐点存在标志
int16 R_corner_row1 = 0;//右拐点所在行
int16 R_corner_col1 = 0;//右拐点所在列
extern float R_corner_angle1 = 0;//右拐点角度
int C;

float offset_quanz112hong [15] = {0.96, 0.92, 0.88, 0.83, 0.77,
                               0.71, 0.65, 0.59, 0.53, 0.47,
                               0.47, 0.47, 0.47, 0.47, 0.47,};  //偏差权重

struct ROAD_TYPE road_type = {
        .straight      = 0,
        .bend          = 0,
        .Ramp          = 0,
        .Cross         = 0,
        .L_Cross       = 0,
        .R_Cross       = 0,
        .LeftCirque    = 0,
        .RightCirque   = 0,
        .Fork          = 0,
        .Barn_l_out    = 0,
        .Barn_r_out    = 0,
        .Barn_l_in     = 0,
        .Barn_r_in     = 0,

};
struct SPEED_TYPE speed_type = {
        .straight    = 0,
        .bend        = 0,
        .difficut_bend =0,
        .cross       = 0,
        .cirque      = 0,
        .fork        = 0,
        .lean_cross  = 0,
        .ramp        = 0,
        .stop        = 0,
        .barn        = 0
};

void add_speed(float ActualValue,float SetValue,float sum)
{
    if(ActualValue < SetValue)          //缓慢加速到设定值
        ActualValue += sum;
    if(ActualValue >= SetValue)
        ActualValue = SetValue;
}


void minus_speed(float ActualValue,float SetValue,float sum)
{
    if(ActualValue > SetValue)          //缓慢减速到设定值
        ActualValue -= sum;
    if(ActualValue <= SetValue)
        ActualValue = SetValue;
}


float absolute(float z)
{
    z = z< 0 ? (-z) : z;
    return z;
}

int16 limit_a_b(int16 x, int a, int b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}


int16 limit1(int16 x, int16 y)
{
    if (x > y)             return y;
    else if (x < -y)       return -y;
    else                return x;
}




uint8 original_image[image_h][image_w];
void Get_image(uint8(*mt9v03x_image)[MT9V03X_W])
{
#define use_num     1   //1 不压缩，2 压缩一倍
    uint8 i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < 80; i += use_num)          //
    {
        for (j = 0; j <MT9V03X_W; j += use_num)     //
        {
            original_image[row][line] = mt9v03x_image[i][j];//这里的参数填写你的摄像头采集到的图像
            line++;
        }
        line = 0;
        row++;
    }
}






uint8 OtsuThreshold1(uint8 *image, uint16 col, uint16 row)
{

#define GrayScale 256
    uint16 Image_Width  = col;
    uint16 Image_Height = row;
    int X; uint16 Y;
    uint8* data = image;
    int HistGram[GrayScale] = {0};

    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 类间方差;
    uint8 MinValue=0, MaxValue=0;
    uint8 Threshold1 = 0;


    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        HistGram[(int)data[Y*Image_Width + X]]++; //统计每个灰度值的个数信息
        }
    }




    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //前景像素点数
          PixelFore = Amount - PixelBack;         //背景像素点数
          OmegaBack = (double)PixelBack / Amount;//前景像素百分比
          OmegaFore = (double)PixelFore / Amount;//背景像素百分比
          PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
          MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
          MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//遍历最大的类间方差g
          {
              SigmaB = Sigma;
              Threshold1 = (uint8)Y;
          }
    }

   return Threshold1;
}


//二值化
uint8 Threshold1;  //阈值
uint8 Threshold1_static = 120;   //阈值静态下限
uint16 Threshold1_detach = 300;  //阳光算法分割阈值(光强越强,该值越大)
void Get01change_Dajin(void)
{
    Threshold1 = Threshold1_Deal(original_image[0], image_w, image_h, Threshold1_detach);

    if (Threshold1 < Threshold1_static)
    {
        Threshold1 = Threshold1_static;
    }

    uint8 thre;
    for(uint8 y = 0; y < image_h; y++)
    {
        for(uint8 x = 0; x < image_w; x++)
        {
            if (x <= 15)
                thre = Threshold1 - 10;
            else if (x >= image_w-15)
                thre = Threshold1 - 10;
            else
                thre = Threshold1;

            if (original_image[y][x] >thre)         //数值越大，显示的内容越多，较浅的图像也能显示出来
                imag[y][x] = 255;  //白    //255
            else
                imag[y][x] = 0;  //黑
        }
    }
}

uint8 Threshold1_Deal(uint8* image, uint16 col, uint16 row, uint32 pixel_Threshold1)
{
#define GrayScale 256
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height;
    uint8 Threshold1 = 0;
    uint8* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    uint32 gray_sum = 0;
    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i += 1)
    {
        for (j = 0; j < width; j += 1)
        {
            // if((sun_mode&&data[i*width+j]<pixel_Threshold1)||(!sun_mode))
            //{
            pixelCount[(
                    int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum += (int)data[i * width + j];  //灰度值总和
            //}
        }
    }

    //计算每个像素值的点在整幅图像中的比例
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }

    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < pixel_Threshold1; j++)
    {
        w0 +=
                pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和 即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值

        w1 = 1 - w0;
        u1tmp = gray_sum / pixelSum - u0tmp;

        u0 = u0tmp / w0;    //背景平均灰度
        u1 = u1tmp / w1;    //前景平均灰度
        u = u0tmp + u1tmp;  //全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            Threshold1 = (uint8)j;     //本来这里没有强制类型转换的,我自己加的
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }
    return Threshold1;
}
void binaryzation(void)
{
  uint8 i,j;

Threshold1_value = OtsuThreshold1(original_image[0], image_w, image_h);
  for(i = 0;i<image_h;i++)
  {

      for(j = 0;j<image_w;j++)
      {
          if(original_image[i][j]>Threshold1_value)imag[i][j] = white_pixel;
          else imag[i][j] = black_pixel;

      }

  }

}


uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0,l_found = 0,r_found = 0;
    uint8 j = 0;
    //清零
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y

    //从中间往左边，先找起点
    for (i = 94; i > border_min; i--)
    {
        start_point_l[0] = i;//x
        start_point_l[1] = start_row;//y
        if (imag[start_row][i] == 255 && imag[start_row][i - 1] == 0)
        {
            j=i;
            l_found = 1;
            break;
        }
    }
    //避免赛道贴太近的巡线干扰
    if((j==0)||(j!=0&&j>94))j=image_w / 2;
    else j=(j+image_w / 2)/2;

    for (i = j; i < border_max; i++)
    {
        start_point_r[0] = i;//x
        start_point_r[1] = start_row;//y
        if (imag[start_row][i] == 255 && imag[start_row][i + 1] == 0)
        {
            r_found = 1;
            break;
        }
    }

    if(l_found||r_found)
        return 1;
    else
        return 0;

}

#define USE_num image_h*3
 //存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*Endline)
{

    uint8 i = 0, j = 0;
    //左边变量
    uint8 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };
    uint16 l_data_statics;//统计左边
    //定义八个邻域
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是顺时针

    //右边变量
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//中心坐标点
    uint8 index_r = 0;//索引下标
    uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics;//统计右边
    //定义八个邻域
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是逆时针

    l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

    //第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x;//x
    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y

        //开启邻域循环
    while (break_flag--)
    {

        //左边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;//索引加一

        //右边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y

        index_l = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//先清零，后使用
            temp_l[i][1] = 0;//先清零，后使用
        }

        //左边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i);//记录生长方向
            }

            if (index_l)
            {
                //更新坐标点
                center_point_l[0] = temp_l[0][0];//x
                center_point_l[1] = temp_l[0][1];//y
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }

        }
        if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
            //printf("三次进入同一个点，退出\n");
            break;
        }
        if (absolute(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && absolute(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
            )
        {
            //printf("\n左右相遇退出\n");
            *Endline = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
            //printf("\n在y=%d处退出\n",*Endline);
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
           // printf("\n如果左边比右边高了，左边等待右边\n");
            continue;//如果左边比右边高了，左边等待右边
        }
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
        {
            //printf("\n左边开始向下了，等待右边，等待中... \n");
            center_point_l[0] = points_l[l_data_statics - 1][0];//x
            center_point_l[1] = points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//索引加一

        index_r = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//先清零，后使用
            temp_r[i][1] = 0;//先清零，后使用
        }

        //右边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;//索引加一
                dir_r[r_data_statics - 1] = (i);//记录生长方向
                //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            if (index_r)
            {

                //更新坐标点
                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }

            }
        }


    }

    //取出循环次数
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;

}

void get_left(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;

    //初始化
    for (i = 0;i<image_h;i++)
    {
        left[i] = border_min;

    }
    h = image_h - 2;
    //左边
    for (j = 0; j < total_L; j++)
    {
        if (points_l[j][1] == h)
        {
            left[h] = points_l[j][0]+1;

        }
        else continue; //每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)
        {
            break;//到最后一行退出
        }
    }
}

void lost_left(void){
    uint8 i=0;
    left_lost_num=0;
    Lost_left_Flag=0;
    for(i=image_h -5;i>2;i--){
        if(imag[i][2]==White){
            left_lost_num++;
            Lost_point_L_scan_line=i+4;
        }
        if(left_lost_num>20){
            Lost_left_Flag=1; //判断左边下方是否丢线
            return;
        }
    }
}

void get_right(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;

    for (i = 0; i < image_h; i++)
    {
        right[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据

    }
    h = image_h - 2;
    //右边
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            right[h] = points_r[j][0] - 1;

        }
        else continue;//每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)break;//到最后一行退出
    }
}

void lost_right(void){
    uint8 i=0;
    right_lost_num=0;
    Lost_right_Flag=0;
    for(i=image_h-5;i>2;i--){
        if(imag[i][image_w-3]==White){
            right_lost_num++;
            Lost_point_R_scan_line=i+4;
        }
        if(right_lost_num>20){
            Lost_right_Flag=1;  //判断右边下方是否丢线
            return;
        }
    }
}


//寻找中线
void middle_line(void)
{
    if(annulus_L_memory==3)
    {
        for(y=image_h-1;y>Endline;y--)
        {
            middle[y]=(left[y]+130+left[y])/2;
        }
    }
    else if(annulus_R_memory==3)//
    {
        for(y=image_h-1;y>Endline;y--)
        {
            middle[y]=(right[y]-130+right[y])/2;
        }
    }

    else
    {
        for(y=image_h-1;y>Endline;y--)
        {
            middle[y]=(right[y]+left[y])/2;
        }
    }
}


void left_dandiao()
{
    int i,q;
    for(i=70;i>=15;i--)
    {
        if(left[i-1]-left[i]<=3&&(left[i-1]-left[i])>=0)
        {
            q++;
        }
        else
        {
            left_flage=0;
            return;
        }
    }
    if(q>=40)
        left_flage=1;
}


void right_dandiao()
{
    int i,j;
    for(i=70;i>=15;i--)
    {
        if((right[i]-right[i-1])<=3&&(right[i]-right[i-1])>=0)
        {
            j++;
        }
        else
        {
            right_flage=0;
           return;
        }

    }
    if(j>=40)
        right_flage=1;
}







//定义膨胀和腐蚀的阈值区间
#define Threshold1_max   255*5
#define Threshold1_min   255*2
void image_filter(uint8(*imag)[image_w])//形态学滤波，膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;


    for (i = 1; i < image_h - 1; i++)
    {
        for (j = 1; j < (image_w - 1); j++)
        {
            //统计八个方向的像素值
            num =
                imag[i - 1][j - 1] + imag[i - 1][j] + imag[i - 1][j + 1]
                + imag[i][j - 1] + imag[i][j + 1]
                + imag[i + 1][j - 1] + imag[i + 1][j] + imag[i + 1][j + 1];


            if (num >= Threshold1_max && imag[i][j] == 0)
            {

                imag[i][j] = 255;//白  可以搞成宏定义，方便更改

            }
            if (num <= Threshold1_min && imag[i][j] == 255)
            {

                imag[i][j] = 0;//黑

            }

        }
    }

}
//边界画黑框
void image_draw_rectan1(uint8(*image)[image_w])
{

    uint8 i = 0;
    for (i = 0; i < image_h; i++)
    {
        image[i][0] = 0;
        image[i][1] = 0;
        image[i][image_w - 1] = 0;
        image[i][image_w - 2] = 0;

    }
    for (i = 0; i < image_w; i++)
    {
        image[0][i] = 0;
        image[1][i] = 0;
        //image[image_h-1][i] = 0;

    }
}


//==========================================================拐点识别====================================================

//---------------------------------左下拐点--------------------------------
//-----------------------------第二版：用断点判断------------------------
void Lower_left(void){
    Lower_left_inflection_Flag=0;
    Lower_left_inflection_X =0;
    Lower_left_inflection_Y =0;
        for(y=image_h-3;y>(Endline+10);y--){
            if(y>10){
                if((left[y]-left[y-4])>15&&left[y-4]<=3&&(left[y]-left[y+2])<5&&left[y]>10){
                    Lower_left_inflection_Flag=1;
                    Lower_left_inflection_X =left[y];
                    Lower_left_inflection_Y =y;
                    return;
                }
            }

         }

    //----------------------------第一版：用扫线方法判断--------------------------------
    //    for(y=110;y>(Endline+10);y--){
    //        for(x=left[y+1];x<186&&x>1;x++){
    //              if(imag[y][x-1]==Black&&imag[y][x]==Black&&imag[y][x+1]==White){
    //                 Lower_left_inflection_X =x;
    //                 Lower_left_inflection_Y =y;
    //              //  ips200_draw_point(Lower_left_inflection_X, Lower_left_inflection_Y , RGB565_RED);
    //                 break;
    //              }
    //              if(x>(left[y-1]+3)&&imag[y][x]==White){
    //                  Lower_left_inflection_Flag=1;
    //                  X1=Lower_left_inflection_X;
    //                  Y1=Lower_left_inflection_Y;
    //                  break;
    //                  }
    //            }
    //
    //
    //        }
}

//-------------------------------------右下拐点---------------------------------
//-----------------------------第二版：用断点判断------------------------
void Lower_right(void)

{
    Lower_right_inflection_Flag=0;
    Lower_right_inflection_X =0;
    Lower_right_inflection_Y =0;
    for(y=image_h-3;y>(Endline+10);y--){
        if(y>10){
            if((right[y-4]-right[y])>15&&right[y-4]>=image_w-3&&(right[y+2]-right[y])<5&&left[y]<image_w-10){
                Lower_right_inflection_Flag=1;
                Lower_right_inflection_X =right[y];
                Lower_right_inflection_Y =y;
                return;
            }
        }

     }
    //----------------------------第一版：用扫线方法判断--------------------------------
    //（可对右下拐点识别，但是右转弯时的拐角也会被判定）
    //        for(y=115;y>(Endline+15);y--){
    //            for(x=right[y+1];x<186&&x>1;x--){
    //                  if(imag[y][x-1]==White&&imag[y][x]==Black&&imag[y][x+1]==Black){
    //                     Lower_right_inflection_X =x;
    //                     Lower_right_inflection_Y =y;
    //                    // ips200_draw_point(Lower_right_inflection_X, Lower_right_inflection_Y , RGB565_RED);
    //                     break;
    //                  }
    //                  if(x<(right[y-1]-3)&&imag[y][x]==White){
    //                      Lower_right_inflection_Flag=1;
    //                      break;
    //                      }
    //                }
    //
    //            }

}

uint8 Find_Flag;
//-------------------------------------左上拐点------------------------------------
//-----------------------------第二版：用断点判断------------------------
void Upper_left(void){
    uint8 h=image_h-3;
    Upper_left_inflection_Flag=0;
    Upper_left_inflection_X =0;
    Upper_left_inflection_Y =0;
    if(Lost_left_Flag==1){
        //针对圆环写的找点方式
          if(annulus_L_Flag==1)
          {
              for(h=Lost_point_L_scan_line+5;h>(Endline+5);h--)
              {
                if((left[h]-left[h+4])>5&&left[h+4]==2&&left[h]!=2&&(left[h-4]-left[h])<5)
                {
                   if(left[h]>93) {            //针对圆环状态4补线出现的断层问题进行的尝试优化
                       Upper_left_inflection_Flag=1;
                       Upper_left_inflection_X =left[h];
                       Upper_left_inflection_Y =h;
                       //-----------第一版：用扫线方法判断-----------
                       Find_Flag=0;
                       for(y=h;y<image_h-10;y++){
                           Find_Flag=0;
                           for(x=left[h]+5;x>70;x--){
                                 if(imag[y][x-1]==Black&&imag[y][x]==Black&&imag[y][x+1]==White){
                                     Find_Flag=1;
                                    break;
                                 }

                               }
                           if(Find_Flag==0){
                               Upper_left_inflection_Y =y-1;
                               return;
                               }

                           }

                   }
                   else{
                       Upper_left_inflection_Flag=1;
                       Upper_left_inflection_X =left[h];
                       Upper_left_inflection_Y =h;
                       return;
                   }
                }
             }
          }
          else{
              for(h=Lost_point_L_scan_line+5;h>(Endline+5);h--){
                if((left[h]-left[h+4])>4&&left[h+4]==2&&left[h]!=2&&(left[h-1]-left[h])<5){

                       Upper_left_inflection_Flag=1;
                       Upper_left_inflection_X =left[h];
                       Upper_left_inflection_Y =h;
                       return;

                }
             }
          }

    }
    //----------------------------第一版：用扫线方法判断--------------------------------
    //        for(y=Endline+15;y<110;y++){
    //            for(x=left[y-1];x<186&&x>1;x--){
    //                  if(imag[y][x-1]==Black&&imag[y][x]==Black&&imag[y][x+1]==White){
    //                     Upper_left_inflection_X =x;
    //                     Upper_left_inflection_Y =y;
    //                     //ips200_draw_point(Upper_left_inflection_X, Upper_left_inflection_Y , RGB565_BLUE );
    //                     break;
    //                  }
    //                  if(x<(left[y-1]-3)){
    //                      Upper_left_inflection_Flag=1;
    //                      break;
    //                      }
    //                }
    //
    //
    //            }

}

//-----------------------------------右上拐点-----------------------------------
void Upper_right(void){
    uint8 h=image_h-3;
    Upper_right_inflection_Flag=0;
    Upper_right_inflection_X =0;
    Upper_right_inflection_Y =0;
    if(Lost_right_Flag==1){
        //针对圆环写的找点方式
          if(annulus_R_Flag==1)
          {
              for(h=Lost_point_R_scan_line+5;h>(Endline+5);h--)
              {
                if((right[h+4]-right[h])>5&&(right[h]-right[h-4])<5&&right[h+4]==image_w -3&&right[h]!=image_w-3)
                {
                   if(right[h]<93)
                   {
                       Upper_right_inflection_Flag=1;
                       Upper_right_inflection_X =right[h];
                       Upper_right_inflection_Y =h;


                       Find_Flag=0;
                       for(y=h;y<image_h-10;y++)
                       {
                           Find_Flag=0;
                           for(x=right[h]+5;x>70;x--)
                           {
                                 if(imag[y][x-1]==White&&imag[y][x]==Black&&imag[y][x+1]==Black){
                                     Find_Flag=1;
                                    break;
                                 }
                               }
                           if(Find_Flag==0){
                               Upper_right_inflection_Y =y-1;
                               return;
                               }
                           }
                   }
                   else{
                       Upper_right_inflection_Flag=1;
                       Upper_right_inflection_X =right[h];
                       Upper_right_inflection_Y =h;
                       return;
                   }
                }
             }
          }
          else{
              for(h=Lost_point_R_scan_line+5;h>(Endline+5);h--){
                      if((right[h+4]-right[h])>4&&right[h+4]==image_w-3&&right[h]!=image_w -3&&(right[h]-right[h-1])<5){
                          Upper_right_inflection_Flag=1;
                          Upper_right_inflection_X =right[h];
                          Upper_right_inflection_Y =h;
                          return;
                  }

              }
          }

    }
    //----------------------------第一版：用扫线方法判断--------------------------------
    //        for(y=Endline+15;y<110;y++){
    //            for(x=right[y-1];x<186&&x>1;x++){
    //                  if(imag[y][x-1]==White&&imag[y][x]==Black&&imag[y][x+1]==Black){
    //                     Upper_right_inflection_X =x;
    //                     Upper_right_inflection_Y =y;
    //                    // ips200_draw_point(Upper_right_inflection_X, Upper_right_inflection_Y , RGB565_BLUE );
    //                     break;
    //                  }
    //                  if(x>(right[y-1]+3)){
    //                      Upper_right_inflection_Flag=1;
    //                      break;
    //                      }
    //                }
    //            }
}

//--------拐点总判断-----------
void inflection_point(void){

    Upper_left();
    Upper_right();
    Lower_left();
    Lower_right();
}


float k1_r,k2_r,k3_r;
//==================================================右直线识别===========================================================
void right_straight(void){
    Right_straight_flag=0;
    k1_r=((float)right[70]-(float)right[40])/30;
    k2_r=((float)right[50]-(float)right[35])/15;
    k3_r=((float)right[40]-(float)right[5])/35;

    if(absolute(k1_r-k2_r)<0.9&&absolute(k2_r-k3_r)<0.9&&absolute(k3_r-k1_r)<0.9&&k1_r!=0&&k2_r!=0&&k3_r!=0&&Lost_right_Flag==0&&Lower_right_inflection_Flag==0){
        Right_straight_flag=1;
    }

}
float k1_l,k2_l,k3_l;
//==================================================左直线识别===========================================================
void left_straight(void){
    Left_straight_flag=0;
    k1_l=((float)left[70]-(float)left[40])/30;
    k2_l=((float)left[50]-(float)left[35])/15;
    k3_l=((float)left[40]-(float)left[5])/35;

    if(absolute(k1_l-k2_l)<0.9&&absolute(k2_l-k3_l)<0.9&&absolute(k3_l-k1_l)<0.9&&k1_l!=0&&k2_l!=0&&k3_l!=0&&Lost_left_Flag==0&&Lower_left_inflection_Flag==0){
        Left_straight_flag=1;
    }

}


//=================================================十字识别=======================================================
void crossroad(void){
    //提前十字处理
    if(annulus_L_Flag==0&&Lost_left_Flag==1&&Lost_right_Flag==1&&Lower_left_inflection_Flag==1&&Lower_right_inflection_Flag==1&&Crossroad_Flag==0&&Crossroad_memory==0&&annulus_R_Flag==0&&Endline<5&&length>78){
        road_type.Cross = 1;
        Crossroad_Flag=1;
        Crossroad_memory=1;
        annulus_L_memory =0;
        annulus_L_Flag=0;
        annulus_R_memory =0;
        annulus_R_Flag=0;
    }
    if(Crossroad_Flag==1){
        //状态1 看到十字，和左右下面两个拐点
       if(Crossroad_memory==1){

           if(Lower_left_inflection_Flag==1){
               Addingline1(1,Lower_left_inflection_X,Lower_left_inflection_Y);
           }
           else{
               Addingline( 1, 60,35, 30, image_h-1);
           }

           if(Lower_right_inflection_Flag==1){
               Addingline1(2,Lower_right_inflection_X,Lower_right_inflection_Y);
           }
           else{
               Addingline( 2, 119,35,image_w-30, image_h-1);
           }

        if ( Upper_left_inflection_Flag == 1 || Upper_right_inflection_Flag == 1)
            {
            Crossroad_memory=2;
            }
         }
       //状态2 看到左上拐点和右上拐点，向下拉线
       if(Crossroad_memory==2){
           if(Lower_left_inflection_Flag==1&&Upper_left_inflection_Flag==1)
           {
               Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y,Lower_left_inflection_X,Lower_left_inflection_Y);
           }
           else if(Upper_left_inflection_Flag==1)
           {
               Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, 10, image_h-1);
           }
           else{
               Addingline( 1, 60,35, 30, image_h-2);
           }
           if(Lower_right_inflection_Flag==1&&Upper_right_inflection_Flag==1)
           {
               Addingline( 2,Upper_right_inflection_X, Upper_right_inflection_Y,Lower_right_inflection_X,Lower_right_inflection_Y);
           }
           else if(Upper_right_inflection_Flag==1){
               Addingline( 2,Upper_right_inflection_X, Upper_right_inflection_Y,image_w-10, image_h-1);
           }
           else{
               Addingline( 2, 150,35, image_w-30, image_h-2);
           }
           if(Lower_left_inflection_Flag==0&&Lower_right_inflection_Flag==0){
//               if(Lower_left_inflection_Flag==0&&Lower_right_inflection_Flag==0&&Lost_left_Flag==1&&Lost_right_Flag==1){ //智能制造赛道
               Crossroad_memory=4;
           }
           else return;
       }
       //状态3 在十字中行驶,出环遇拐点补线
       if(Crossroad_memory==3){
           if(Lower_left_inflection_Flag==1){
               Addingline1(1,Lower_left_inflection_X,Lower_left_inflection_Y);
           }
           if(Lower_right_inflection_Flag==1){
               Addingline1(2,Lower_right_inflection_X,Lower_right_inflection_Y);
           }
           if(Upper_left_inflection_Flag==1&&Upper_right_inflection_Flag==1&&Lost_left_Flag==0&&Lost_right_Flag==0){
               Crossroad_memory=4;
           }
       }
       //状态4 补线出十字
       if(Crossroad_memory==4){
           if(Upper_left_inflection_Flag==1)
           {
               Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y,10, image_h-1);
           }
           else{
               Addingline( 1, 50,35, 30, image_h-1);
           }
           if(Upper_right_inflection_Flag==1)
           {
               Addingline( 2,Upper_right_inflection_X, Upper_right_inflection_Y, image_w-10, image_h-1);
           }
           else{
               Addingline( 2, 135,35, image_w-30, image_h-1);
           }
           //退出
           if(Upper_left_inflection_Flag==0&&Upper_right_inflection_Flag==0){
               road_type.Cross = 0;
               Crossroad_Flag=0;
               Crossroad_memory=0;
               return;
           }
       }

  }
}


//=============================================圆环===================================================
//================环岛识别=====================

void roundabout_L(void){

    roundabout_X=0;
    roundabout_Y=0;
    roundabout_Flag=0;
    for(y=image_h-3;y>10;y--){
        if((left[y]-left[y-8])>5&&left[y-8]<=3&&(left[y]-left[y+2])<5&&Lost_left_Flag==1){
            y+=4;
            roundabout_Flag=1;
            roundabout_X =left[y];
            roundabout_Y =y;
            return;
        }
     }

}

void roundabout_R(void){
    roundabout_X=0;
    roundabout_Y=0;
    roundabout_Flag=0;
    for(y=image_h-3;y>10;y--){
        if((right[y-8]-right[y])>5&&right[y-8]>=image_w -3&&(right[y+2]-right[y])<5&&Lost_right_Flag==1){
            y+=4;
            roundabout_Flag=1;
            roundabout_X =right[y];
            roundabout_Y =y;
            return;
        }
     }

}
//================出环拐点识别=====================
void Exit_loop_L_inflection(void)
{
   uint8 i;
   Exit_loop_Flag=0;
   Exit_loop_X=0;
   Exit_loop_Y=0;
  //左圆环，识别右边拐点
    for(i=image_h-5;i>Endline+5;i--)
    {
        if(right[i]<right[i+4]&&right[i]<right[i-4]&&right[i]<right[i+3]&&right[i]<right[i-3]){
            Exit_loop_Flag=1;
            Exit_loop_X=right[i];
            Exit_loop_Y=i;
            return;
        }
    }
}

void Exit_loop_R_inflection(void)
{
   uint8 i;
   Exit_loop_Flag=0;
   Exit_loop_X=0;
   Exit_loop_Y=0;
  //右圆环，识别右边拐点
    for(i=image_h-5;i>Endline+5;i--)
    {
        if(left[i+4]<left[i]&&left[i-4]<left[i]&&left[i+3]<left[i]&&left[i-3]<left[i]){
            Exit_loop_Flag=1;
            Exit_loop_X=left[i];
            Exit_loop_Y=i;
            return;
        }
    }
}


//=========避免十字误判圆环===========
int  danbianqiao_wupan()
{
    int i=0,j=0,k=0,l=0;
        for(i=75;i>=2;i--)
        {
            for(j=2;j<=185;j++)
            {
                if(imag[i][j])
                {
                    l++;
                }

            }
            if(l>=160)
            {
                k++;
            }
            l=0;

        }
        if(k>=5)
            return 1 ;
        else
            return 0 ;
}




//=========横断判断===========
void  hengduan()
{

        int i=0,j=0,k=0,l=0;
            for(i=50;i>=30;i--)
            {
                for(j=0;j<=185;j++)
                {
                    if(imag[i][j]==0)
                    {
                        l++;
                    }

                }
                if(l>=183)
                {
                    k++;
                }
                l=0;

            }
            if(k>=10)//&&anu_falg==0&&dl1b_distance_mm<=350
            {
                 now_yaw = QEKF_INS.YawTotalAngle ;
                 hengduan_flag=1;  //横断标志位打开！！！跳过后自己处理归零
                 buzzer();
            }


}
//============= 检查拐点附近5行是否全白=============//
uint8 yuanhuanwupan = 0;
// 全白行检测函数
void CheckAllWhite(uint8 inflectionY) //checkAllWhiteRows(Lower_left_inflection_Y);左环-------------checkAllWhiteRows(Lower_right_inflection_Y);右环
{
    yuanhuanwupan = 0;
    for (uint8 y = inflectionY; y > inflectionY - 5 && y > 10; y--) {
        uint8 allWhite = 1;
        for (uint8 x = 0; x < image_w; x++) {
            if (imag[y][x] != White)
            {
                allWhite = 0;
                break;
            }
        }
        if (allWhite) {
            yuanhuanwupan = 1;
            return;
        }
    }
}

//================左圆环识别=====================
void annulus_L(void){
    //识别圆环
    if(!danbianqiaowupan&&Upper_left_inflection_Flag==0&&Lost_left_Flag==1&&Lower_left_inflection_Flag==1&&Right_straight_flag==1
            &&Lower_right_inflection_Flag==0&&annulus_L_memory==0&&annulus_L_Flag==0&&imag[Lower_left_inflection_Y][2]==White&&length>=78
            &&zebra_crossing_flag==0&&Lower_left_inflection_Y>=30&&!Crossroad_Flag)
    {
          road_type.LeftCirque=1;
          annulus_L_Flag=1;
          annulus_L_memory =1;
          Crossroad_Flag=0;
          Crossroad_memory=0;
      }

     if(annulus_L_Flag==1){

         //状态 1  识别到圆环，未识别到环岛（可强行补线，或不补线，待测试）
         if (annulus_L_memory == 1)
         {
             if(Lower_left_inflection_Flag==1)
             {
                 if(Lower_left_inflection_Y>=73)Lower_left_inflection_Y=73;
                 Addingline1( 1, Lower_left_inflection_X, Lower_left_inflection_Y);
             }
             else
              if(Lower_left_inflection_Flag==0){
                 annulus_L_memory = 2;
             }
         }
         //状态2 识别到圆环，环岛，并对左边进行补线
          if (annulus_L_memory == 2 )
         {
              roundabout_L();          //环岛
            if(roundabout_Flag==1){
                Addingline( 1, roundabout_X, roundabout_Y,image_w-170, image_h -1 );
            }
            else {
                Addingline( 1, 10, 25,image_w-170 , image_h -1 );
            }
            if(Upper_left_inflection_Flag==1&&Upper_left_inflection_Y>4){   //改变Upper_left_inflection_Y限制间接改变入环位置，待验证
                annulus_L_memory = 3;
            }
            else return;
         }
         //状态3 到达圆环入口，封住前路，补线入环

          if (annulus_L_memory == 3 )
          {

              if(Upper_left_inflection_Flag==1){
                  if(Upper_left_inflection_X>20){
                      Addingline( 2, Upper_left_inflection_X-20,Upper_left_inflection_Y,right[image_h-1] , image_h-1);
                  }
                  else{
                      Addingline( 2, Upper_left_inflection_X,Upper_left_inflection_Y,right[image_h-1] , image_h-1);
                  }
                  return;
              }
              else if(Upper_left_inflection_Flag==0&&Lost_left_Flag==1&&Endline>2){//
                  annulus_L_memory = 4;
              }
              else return;

          }

      //    if (annulus_L_memory == 4 )sum=sum+4;
         //状态4 在圆环中行驶，当看到右下拐点时进入下一状态
         if (annulus_L_memory == 4)
         {
            Exit_loop_L_inflection();
            if(Exit_loop_Flag==0){
                return;
            }
            if(Exit_loop_Flag==1&&Lost_left_Flag==1){
                annulus_L_memory = 5;
            }

         }

         //状态5 出环时看到右下拐点,对其补线处理
         if (annulus_L_memory == 5 )
          {
             Exit_loop_L_inflection();
             if(Exit_loop_Flag==1){
                 if(Exit_loop_Y>32){
                     Addingline( 2, 3, Exit_loop_Y-30, Exit_loop_X, Exit_loop_Y);//此处补线结束点待测试
                     return;
                 }
                 else {
                     Addingline( 2, 3, 15, Exit_loop_X, Exit_loop_Y);//此处补线结束点待测试
                     return;
                 }
             }
             else if(Lost_left_Flag==1&&Lost_right_Flag==1&&Exit_loop_Flag==0){
                 annulus_L_memory = 6;
             }
             else return;

          }

         //状态6 出环时右下拐点消失，但是车还没完全出环，此时还需要补线处理zhonxianpiancha=-20;//左转

         if (annulus_L_memory == 6 )
          {

             if(Lost_left_Flag==1&&Lost_right_Flag==1){
                 Addingline( 2,80, Endline+3, image_w, image_h-1);
                 return;
             }
             else if((Lost_left_Flag==1&&Lost_right_Flag==0)){
                 annulus_L_memory = 7;
             }
             else return;
          }
         // 状态7 出环补线
         if (annulus_L_memory == 7)
           {
              if(Upper_left_inflection_Flag==1){
                  Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, 2, image_h-1);
                  return;
              }
              else{
                  Addingline( 1, 32, 20, image_w -150, image_h-1);
              }
              if(Lost_right_Flag==0&&Lost_left_Flag==0){
                  road_type.LeftCirque=0;
                  annulus_L_memory =0;
                  annulus_L_Flag=0;
                  return;
              }

           }

     }
}


//================右圆环识别=====================
void annulus_R(void){
    //识别圆环
      if(!danbianqiaowupan&&Upper_right_inflection_Flag==0&&Lost_right_Flag==1&&Lower_right_inflection_Flag==1&&Lower_left_inflection_Flag==0
              &&Left_straight_flag==1&&annulus_R_memory==0&&annulus_R_Flag==0&&imag[Lower_right_inflection_Y][image_h-3]==White&&length>=78
              &&zebra_crossing_flag==0&&Lower_right_inflection_Y>=30&&!Crossroad_Flag)
      {
          road_type.RightCirque=1;
          annulus_R_Flag=1;
          annulus_R_memory =1;
          Crossroad_Flag=0;
          Crossroad_memory=0;
      }
     if(annulus_R_Flag==1){
         //状态 1  识别到圆环，未识别到环岛（可强行补线，或不补线，待测试）
         if (annulus_R_memory == 1)
         {
             if(Lower_right_inflection_Flag==1){
                 if(Lower_right_inflection_Y>=73)Lower_right_inflection_Y=73;
                 Addingline1( 2, Lower_right_inflection_X, Lower_right_inflection_Y);
             }
         else if(Lower_right_inflection_Flag==0){
                 annulus_R_memory = 2;
             }
         }
         //状态2 识别到圆环，环岛，并对左边进行补线
          if (annulus_R_memory == 2 )
         {
              roundabout_R();          //环岛
            if(roundabout_Flag==1){
                Addingline( 2, roundabout_X, roundabout_Y,image_w-10 , image_h-3 );
             }
            else {
                Addingline( 2, 170, 25,image_w-10 , image_h-3 );  //要改
            }
            if(Upper_right_inflection_Flag==1&&Upper_right_inflection_Y>6){   //改变Upper_right_inflection_Y限制间接改变入环位置，待验证
                annulus_R_memory = 3;
            }
            else return;
         }
         //状态3 到达圆环入口，封住前路，补线入环
          if (annulus_R_memory == 3 )
          {
              if(Upper_right_inflection_Flag==1)
              {
                  if(Upper_right_inflection_X<168)
                  {
                      Addingline( 1, Upper_right_inflection_X+25,Upper_right_inflection_Y+5,left[image_h-3]+20 , image_h-3);
                  }
                  else
                  {
                      Addingline( 1, Upper_right_inflection_X,Upper_right_inflection_Y,left[image_h-3]+20 , image_h-3);
                  }
                   return;
              }
              else if(Upper_right_inflection_Flag==0&&Lost_right_Flag==1&&Endline>2)//&&Lost_right_Flag==1
              {
                  annulus_R_memory = 4;
              }
              else return;

          }
         //状态4 在圆环中行驶，当看到左下拐点时进入下一状态
         if (annulus_R_memory == 4)
         {
            Exit_loop_R_inflection();
            if(Exit_loop_Flag==0){
                return;
            }
            if(Exit_loop_Flag==1&&Lost_right_Flag==1){
                annulus_R_memory = 5;
            }
         }

         //状态5 出环时看到右下拐点,对其补线处理
         if (annulus_R_memory == 5 )
          {
             Exit_loop_R_inflection();
             if(Exit_loop_Flag==1){
                 if(Exit_loop_Y>33){
                     Addingline( 1, image_w-3, Exit_loop_Y-30, Exit_loop_X, Exit_loop_Y);//此处补线结束点待测试
                     return;
                 }
                 else {
                     Addingline( 1, image_w-3, 15, Exit_loop_X, Exit_loop_Y);//此处补线结束点待测试
                     return;
                 }
             }
             else if(Lost_right_Flag==1&&Lost_left_Flag==1&&Exit_loop_Flag==0){
                 annulus_R_memory = 6;
             }
             else return;

          }

         // 状态6 出环时右下拐点消失，但车还没完全出环
         if (annulus_R_memory == 6 )
          {
             if(Lost_left_Flag==1&&Lost_right_Flag==1)
             {
                 Addingline( 1, image_w-80, Endline+3, 3, image_h-3);
                 return;
             }
             else if(Lost_right_Flag==1&&Lost_left_Flag==0)
             {
                 annulus_R_memory = 7;
             }
             else return;
          }

        //  状态7：出环补线（右环岛）
         if (annulus_R_memory == 7)
          {
              if(Upper_right_inflection_Flag==1)
              {
                  Addingline( 2, Upper_right_inflection_X, Upper_right_inflection_Y, image_w - 40, image_h-3);
                  return;
              }
              else
              {
                  Addingline(2, 115, 32,image_w-40 , image_h-3);
              }
              if(Lost_left_Flag==0&&Lost_right_Flag==0)
              {
                  road_type.RightCirque=0;
                  annulus_R_memory =0;
                  annulus_R_Flag=0;
              }
           }




/*         if (annulus_R_memory == 7)
         {
             if(Upper_right_inflection_Flag==1)
             {
                 Addingline( 2, Upper_right_inflection_X, Upper_right_inflection_Y, image_w - 40, image_h-3);
                 return;
             }
             else
             {
                 Addingline(2, 115, 32,image_w-40 , image_h-3);
             }
             if(Lost_left_Flag==0&&Lost_right_Flag==0)
             {
                 road_type.RightCirque=0;
                 annulus_R_memory =0;
                 annulus_R_Flag=0;
             }
          }

     }*/

//         if (annulus_R_memory == 7)
//         {
//             if (Upper_right_inflection_Flag == 1)
//             {
//                 // 优化拐点补线：起点向右下偏移，终点延长出环距离
//                 Addingline(2,
//                     Upper_right_inflection_X + 8,   // X向右偏移8像素，避开内圆干扰
//                     Upper_right_inflection_Y + 5,    // Y向下偏移5像素，贴近实际车位置
//                     image_w - 25,                    // 终点X更靠右（原image_w-40）
//                     image_h - 20                     // 终点Y抬高，延长水平出环引导
//                 );
//                 return;
//             }
//             else
//             {
//                 // 无拐点时：起点更贴近右侧，终点延长水平线
//                 Addingline(2,
//                     image_w - 50,    // 起点X从115改为右侧（原115偏左）
//                     image_h - 10,     // 起点Y从32改为底部（原32过高）
//                     image_w - 25,     // 终点X右移
//                     image_h - 20      // 终点Y抬高，强化水平延伸
//                 );
//             }
//
//             // 状态重置逻辑保持不变
//             if (Lost_left_Flag == 0 && Lost_right_Flag == 0)
//             {
//                 road_type.RightCirque = 0;
//                 annulus_R_memory = 0;
//                 annulus_R_Flag = 0;
//             }
//         }

         // 状态7 出环补线（左边界主导）
/*         if (annulus_R_memory == 7)
         {
             // 1. 动态定位出环点（优先左边界底部特征点）
             uint8 exit_x = 0;
             uint8 exit_y = image_h - 5;  // 底部区域

             // 左边界稳定性检测（底部10行滑动滤波）
             static uint8 stable_left = 0;
             uint8 valid_count = 0;
             for (int y = image_h-1; y >= image_h-10; y--) {
                 if (left[y] != 0 && left[y] < image_w-10) {
                     stable_left = (stable_left * valid_count + left[y]) / (valid_count + 1);
                     valid_count++;
                 }
             }

             // 2. 计算补线起终点
             if (valid_count >= 3) {  // 左边界稳定时
                 exit_x = stable_left + 15;  // 向右偏移15像素，避开内圆干扰[1,7](@ref)
             }
             else if (Upper_right_inflection_Flag == 1) {  // 次选右上拐点
                 exit_x = Upper_right_inflection_X - 8;  // 向左偏移避开内圆
                 exit_y = Upper_right_inflection_Y;
             }
             else {  // 保底逻辑
                 exit_x = image_w - 35;
             }

             // 3. 水平直线延伸（关键！强制X不变）
             uint8 target_x = exit_x;        // X坐标严格不变
             uint8 target_y = image_h / 4;   // 终点提升至图像1/4高度，延长直行引导

             // 4. 执行补线
             Addingline(2, exit_x, exit_y, target_x, target_y);

             // 5. 延迟状态重置（增加环岛特征消失判断）
             static uint8 reset_count = 0;
             if (Lost_left_Flag == 0 && Lost_right_Flag == 0) {
                 // 新增：左边界连续递增（表明已出环）[7](@ref)
                 if (left[image_h-5] < left[image_h-15]) {
                     if (++reset_count >= 5) {  // 连续5帧稳定
                         road_type.RightCirque = 0;
                         annulus_R_memory = 0;
                         reset_count = 0;
                     }
                 } else {
                     reset_count = 0;
                 }
             } else {
                 reset_count = 0;
             }
         }*/


     }
}



//---------------------------------------线性拟合---------------------------------
void advanced_regression1(int type, int startline1, int endline1, int startline2, int endline2)
 {
    int i = 0;
    int sumlines1 = endline1 - startline1;
    int sumlines2 = endline2 - startline2;
    int sumX = 0;
    int sumY = 0;
    float averageX = 0;
    float averageY = 0;
    float sumUp = 0;
    float sumDown = 0;
    if (type == 0)  //拟合中线
    {
        /**计算sumX sumY**/
        for (i = startline1; i <=endline1; i++)
        {
            sumX += i;
            sumY += middle[i];
        }
        for (i = startline2; i <=endline2; i++)
        {
            sumX += i;
            sumY += middle[i];
        }
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x的平均值
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y的平均值
        for (i = startline1; i <= endline1; i++)
        {
            sumUp += (middle[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i <= endline2; i++)
        {
            sumUp += (middle[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB1 = 0;
        else parameterB1 = sumUp / sumDown;
        parameterA1 = averageY - parameterB1 * averageX;

    }
    else if (type == 1)     //拟合左线
    {
        /**计算sumX sumY**/
        for (i = startline1; i <= endline1; i++)
        {
            sumX += i;
            sumY += left[i];
        }
        for (i = startline2; i <= endline2; i++)
        {
            sumX += i;
            sumY += left[i];
        }
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x的平均值
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y的平均值
        for (i = startline1; i <= endline1; i++)
        {
            sumUp += (left[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i <= endline2; i++)
        {
            sumUp += (left[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB1 = 0;
        else parameterB1 = sumUp / sumDown;
        parameterA1 = averageY - parameterB1 * averageX;
    }
    else if (type == 2)         //拟合右线
    {
        /**计算sumX sumY**/
        for (i = startline1; i <= endline1; i++)
        {
            sumX += i;
            sumY += right[i];
        }
        for (i = startline2; i <= endline2; i++)
        {
            sumX += i;
            sumY += right[i];
        }
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x的平均值
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y的平均值
        for (i = startline1; i <= endline1; i++)
        {
            sumUp += (right[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i <= endline2; i++)
        {
            sumUp += (right[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB1 = 0;
        else parameterB1 = sumUp / sumDown;
        parameterA1 = averageY - parameterB1 * averageX;
    }
}
//------------------------------判断直线斜率是否相同------------------------------
int Judgment_symbol(float x, float y)
{
    int a;
    a = 0;
    if (x < 0 && y < 0) a = 1;
    if (x >= 0 && y >= 0) a = 1;
    return a;
}
//-----------------------------------------补线-----------------------------------

//有起点和终点的补线函数
void Addingline( uint8 choice, uint8 startX, uint8 startY, uint8 endX, uint8 endY)
{
    y = 0;

    // 直线 x = ky + b
     k = 0;
     b = 0;
    switch(choice)
    {
      case 1://左补线
        {
            k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
            b = (float)startX - (float)startY * k;

            for(y = startY; y < endY; y++)
            {
                left[y] = (uint8)(k * y + b);
            }
            break;
        }

      case 2://右补线
        {
            k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
            b = (float)startX - (float)startY * k;

            for(y = startY; y < endY; y++)
            {
                right[y]= (uint8)(k * y + b);

            }
            break;
        }

    }
}


void Addingline1( uint8 choice, uint8 startX, uint8 startY)    //看到拐点延斜率向上延长
{

    // 直线 x = ky + b
     k = 0;
     b = 0;
    int temp=0;
    switch(choice)
    {
      case 1://左补线
        {

            k = (float)(((float)left[Lower_left_inflection_Y+1] - (float)left[Lower_left_inflection_Y+5]) /(-4));
            b = (float)((float)left[Lower_left_inflection_Y+5]- (float)(Lower_left_inflection_Y+5) * k);

            for(y = startY; y >(Endline+20); y--)
            {

             temp = (int)(k* y + b);
             if(temp<image_w-1&&temp>2){
                 left[y]=temp;
             }
            }
            break;
        }

      case 2://右补线  待测试
      {

           k = (float)(((float)right[Lower_right_inflection_Y+1] - (float)right[Lower_right_inflection_Y+5]) /(-4));
           b = (float)((float)right[Lower_right_inflection_Y+5]- (float)(Lower_right_inflection_Y+5) * k);

           for(y = startY; y >(Endline+20); y--)
           {

            temp = (int)(k* y + b);
            if(temp<image_w-1&&temp>2){
                right[y]=temp;
            }
           }
           break;
       }

    }
}
void Addingline2( uint8 choice, uint8 startX, uint8 startY)   //找到上拐点延斜率向下拉线
{

    // 直线 x = k*y + b
     k = 0;
     b = 0;
    int temp=0;
    switch(choice)
    {
      case 1://左补线
        {

            k = (float)(((float)left[Upper_left_inflection_Y-2] - (float)left[Upper_left_inflection_Y-5]) /3);
            b = (float)((float)left[Upper_left_inflection_Y-5]- (float)(Upper_left_inflection_Y-5) * k);

            for(y = startY; y<(image_h-3); y++)
            {

             temp = (int)(k* y + b);
             if(temp<image_w-1&&temp>2){
                 left[y]=temp;
             }

            }
            break;
        }

     case 2://右补线  待测试
        {

            k = (float)(((float)right[Upper_right_inflection_Y-2] - (float)right[Upper_right_inflection_Y-5]) /3);
            b = (float)((float)right[Upper_right_inflection_Y-5]- (float)(Upper_right_inflection_Y-5) * k);
            for(y = startY; y<(image_h-3); y++)
            {

             temp = (int)(k* y + b);
             if(temp<image_w-1&&temp>2){
                 right[y]=temp;
             }

            }
            break;
        }

    }
}





int16 Sum1;
void HDPJ_lvbo(uint8 data[], uint8 N, uint8 size)
{
    Sum1 = 0;
    for(uint8 j =0; j <size; j++)
    {
        if(j <N /2)
        {
            for(uint8 k =0; k <N; k++)
            {
                Sum1 +=data[j +k];
            }
            data[j] =Sum1 /N;
        }
        else
            if(j <size -N /2)
            {
                for(uint8 k =0; k <N /2; k++)
                {
                    Sum1 +=(data[j +k] +data[j -k]);
                }
                data[j] = Sum1 /N;
            }
            else
            {
                for(uint8 k =0; k <size -j; k++)
                {
                    Sum1 +=data[j +k];
                }
                for(uint8 k =0; k <(N -size +j); k++)
                {
                    Sum1 +=data[j -k];
                }
                data[j] = Sum1 /N;
            }
        Sum1 = 0;
    }
}


float error_all_turn,error_all = 0;
void Calculate_Offset_1()
{
    int16 offset;                    //摄像头处理得到的偏差;

    for(uint8 y =image_h -2; y >=10; y--)    //利用近大远小的权重计算偏差(调试完成)
    {
        middle[y] =1.0*middle[y];    //这句代码并不是没有用,千万不可以删
    }

        HDPJ_lvbo(middle, 20, image_h -1);   //平均滑动滤波

    for(uint8 y =image_h -2; y >=image_h -10; y--)    //利用近大远小的权重计算偏差(调试完成)
    {
        //offset +=offset_quanzhong[image_h-30 -y] *(middle[y] -MT9V03X_W/2);
        offset += (middle[y] -image_w/2);
    }

        offset =offset/20.0;
     error_all = -offset;
     error_all_turn = error_all;    //车身靠左
}







/**
* @brief 最小二乘法
* @param uint8 begin                输入起点
* @param uint8 end                  输入终点
* @param uint8 *border              输入需要计算斜率的边界首地址
*  @see CTest       Slope_Calculate(start, end, border);//斜率
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
float Slope_Calculate(uint8 begin, uint8 end, uint8 *border)
{
    float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
    int16 i = 0;
    float result = 0;
    static float resultlast;

    for (i = begin; i < end; i++)
    {
        xsum += i;
        ysum += border[i];
        xysum += i * (border[i]);
        x2sum += i * i;

    }
    if ((end - begin)*x2sum - xsum * xsum) //判断除数是否为零
    {
        result = ((end - begin)*xysum - xsum * ysum) / ((end - begin)*x2sum - xsum * xsum);
        resultlast = result;
    }
    else
    {
        result = resultlast;
    }
    return result;
}

/**
* @brief 计算斜率截距
* @param uint8 start                输入起点
* @param uint8 end                  输入终点
* @param uint8 *border              输入需要计算斜率的边界
* @param float *slope_rate          输入斜率地址
* @param float *intercept           输入截距地址
*  @see CTest       calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept)
{
    uint16 i, num = 0;
    uint16 xsum = 0, ysum = 0;
    float y_average, x_average;

    num = 0;
    xsum = 0;
    ysum = 0;
    y_average = 0;
    x_average = 0;
    for (i = start; i < end; i++)
    {
        xsum += i;
        ysum += border[i];
        num++;
    }

    //计算各个平均数
    if (num)
    {
        x_average = (float)(xsum / num);
        y_average = (float)(ysum / num);

    }

    /*计算斜率*/
    *slope_rate = Slope_Calculate(start, end, border);//斜率
    *intercept = y_average - (*slope_rate)*x_average;//截距
}





void Get_Bin_Image (unsigned char mode)
{
    unsigned short i = 0, j = 0;
    unsigned short Threshold1 = 0;

    if (mode == 0)
    {
        Threshold1 = GetOTSU(original_image);  //大津法阈值  灰度图像
      //  Threshold1=50;
            /* 二值化 */
    for (i = 0; i < image_h; i++)
    {
        for (j = 0; j < image_w; j++)
        {
            if (original_image[i][j] > Threshold1) //数值越大，显示的内容越多，较浅的图像也能显示出来
                imag[i][j] = 255;   //白
            else
                imag[i][j] = 0;    //黑
        }
    }

    }
    if (mode == 1)
    {

    }
    else if (mode == 2)
    {
            Threshold1 = GetOTSU(original_image);
            Sobel (original_image, imag , Threshold1);//边缘检测
       return;
    }
    else if (mode == 3)
    {

        return;
    }


}



uint8 length=0;
void Mid_Col(void)
{
    int i;
    for(i=image_h;i>1;i--)
    {
        if(imag[i][94]==0 && imag[i-1][94]==0 && imag[i-2][94]==0)
        {
            break;
        }
    }
    length=image_h-i;
    //        return length;
}





void Sobel (uint8 imageIn[image_h][image_w], uint8 imageOut[image_h][image_w], uint8 Threshold1)
{
    /** 卷积核大小 */
    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = image_w - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = image_h - KERNEL_SIZE / 2;
    short i, j;
    short temp[2];
    for (i = yStart; i < yEnd; i++)
    {
        for (j = xStart; j < xEnd; j++)
        {
            /* 计算不同方向梯度幅值  */
            temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
            - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]                      // {-1, 0, 1},
            - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];             // {-1, 0, 1}};

            temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -1, -1},
            - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]                      // { 0,  0,  0},
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];             // { 1,  1,  1}};

            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);

            /* 找出梯度幅值最大值  */
            if (temp[0] < temp[1])
                temp[0] = temp[1];

            if (temp[0] > Threshold1) imageOut[i][j] = 0;
            else    imageOut[i][j] = 255;
        }
    }
}





//===================================================阈值求取===================================================
short GetOTSU (unsigned char MT_Img[image_h][image_w])
{
    signed short i, j;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelshortegralBack = 0;
    unsigned long Pixelshortegral = 0;
    signed long PixelshortegralFore = 0;
    signed long PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    signed short MinValue, MaxValue;
    signed short Threshold1 = 0;
    unsigned char HistoGram[256];
    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //初始化灰度直方图

    for (j = 0; j < image_h; j++)
    {
        for (i = 0; i < image_w; i++)
        {
            HistoGram[MT_Img[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值

    if (MaxValue == MinValue)
        return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)
        return MinValue;        // 图像中只有二个颜色

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  像素总数

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //灰度值总数
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //前景像素点数
        PixelFore = Amount - PixelBack;           //背景像素点数
        OmegaBack = (float) PixelBack / Amount;   //前景像素百分比
        OmegaFore = (float) PixelFore / Amount;   //背景像素百分比
        PixelshortegralBack += HistoGram[j] * j;  //前景灰度值
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //背景灰度值
        MicroBack = (float) PixelshortegralBack / PixelBack;   //前景灰度百分比
        MicroFore = (float) PixelshortegralFore / PixelFore;   //背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差
        if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold1 = j;
        }
    }
    return Threshold1;                        //返回最佳阈值;
}


void Element_Test(void)
{
    if(!road_type.Fork && !road_type.LeftCirque && !road_type.RightCirque && !road_type.L_Cross
            && !road_type.R_Cross && !road_type.Cross && !road_type.Barn_l_in && !road_type.Barn_r_in)
    {

    }
}

uint8 sudu_yingzi=0;    //速度因子

void Check_Zhidao(void)
{//这里要考虑搜线终止行
    uint8 inc =0, dec =0;
    sudu_yingzi =0;

    for(uint8 y =image_h-2; y >(Endline+5); y--)
    {
        if((left[y] <=left[y-1]) && left[y] !=2)   //两边不丢线才计算直道长度
        {
            inc++;
        }

        if((right[y] >=left[y-1]) && right[y] !=image_w -3)
        {
            dec++;
        }
    }
    if(inc>=dec)
    {
        sudu_yingzi =dec;
    }
    else
    {
        sudu_yingzi =inc;
    }
  if(!road_type.Fork && !road_type.LeftCirque && !road_type.RightCirque && !road_type.L_Cross
                    && !road_type.R_Cross && !road_type.Cross && !road_type.Barn_l_in && !road_type.Barn_r_in)
  {
    if(sudu_yingzi >=69 &&Lost_left_Flag==0&&Lost_right_Flag==0&&Left_straight_flag==1&&Right_straight_flag==1&&Left_straight_flag==1)
    {
        Check_straight_flag = 1;
    }
    else
    {
        Check_straight_flag  = 0;
    }
  }
}
void check_bend_dir(void)
{
    if(!road_type.Fork && !road_type.LeftCirque && !road_type.RightCirque && !road_type.L_Cross
                    && !road_type.R_Cross && !road_type.Cross && !road_type.Barn_l_in && !road_type.Barn_r_in)
    {
        if(Lost_left_Flag==1&&sudu_yingzi<50&&annulus_L_Flag==0&&annulus_L_memory==0&&Crossroad_Flag==0&&Crossroad_memory==0)
        {
            Check_left_flag=1;
            Check_right_flag=0;
            Check_straight_flag=0;
        }else{
            Check_left_flag=0;
        }
        if(Check_left_flag==1)
        {
            error_all_turn=error_all_turn-2;
        }
        if(Lost_right_Flag==1&&sudu_yingzi<40&&annulus_R_Flag==0&&annulus_R_memory==0&&Crossroad_Flag==0&&Crossroad_memory==0)
        {
            Check_left_flag=0;
            Check_right_flag=1;
            Check_straight_flag=0;
        }else{
            Check_right_flag=0;
        }
        if(Check_right_flag==1)
        {
            error_all_turn=error_all_turn+2;
        }
        if(Check_left_flag==0&&Check_right_flag==0&&sudu_yingzi>69&&annulus_R_Flag==0&&annulus_R_memory==0&&Crossroad_Flag==0&&Crossroad_memory==0)
        {
            Check_left_flag=0;
            Check_right_flag=0;
            Check_straight_flag=1;
        }
    }
    else{
        Check_left_flag=0;
        Check_right_flag=0;
        Check_straight_flag=0;
    }
}



uint8 Center = image_w / 2;
int ImageA_MidLine_Get_F(void)
{
   int32_t MidLine_F = 0;
   MidLine_F = (0.4f*middle[32]  +
              0.6f*middle[34]  +
              1.0f*middle[36]  +
              1.0f*middle[38]  +
              1.0f*middle[40]  +
              1.0f*middle[42]  +
              1.1f*middle[44]  +
              1.2f*middle[46]  +
              1.3f*middle[48]  +
              1.4f*middle[50]) / 10;
   return MidLine_F;
}

int ImageA_MidLine_Get_N(void)
{
   int32_t MidLine_N = 0;
   MidLine_N = (0.4f*middle[68]  +
              0.6f*middle[69]  +
              1.0f*middle[70]  +
              1.0f*middle[71]  +
              1.0f*middle[72]  +
              1.0f*middle[73]  +
              1.1f*middle[74]  +
              1.2f*middle[75]  +
              1.3f*middle[76]  +
              1.4f*middle[77]) / 10;
   return MidLine_N;
}





//=========斑马线===========
void  zebra_crossing(uint8(*bin_image)[image_w])
{
   int i = 0, j = 0;
   int count = 0;
   int ew=0;
   for(i = 65;i < 75; i++)
   {
       for(j = 10; j < 180; j++)
       {
           if(bin_image[i][j] == 255 && bin_image[i][j+1] == 0)
           {
               count ++;
           }
           if(count > 9)
           {
               ew++;
               count=0;
               break;
           }

       }
       if(ew > 3&&hengduan_flag==0)
       {
           zebra_crossing_flag=1;//斑马线标志位
           ew=0;
           break;
       }

   }
}



int podao_time=0;
//=========坡道===========PID_HH[3]
void  podao(void)
{
    if(dl1b_distance_mm<=300&&HALF_WIDTH>158&&!hengduan_flag&&!SingleBridge_mode&&!Check_obstacle_flag&&!annulus_L_Flag&&!annulus_R_Flag)//&&Extrawide_counting1(image_h-3,Endline)>=76dl1b_distance_mm<=300
       {
        flag1 = 1;

       }

    if(flag1 == 1&&dl1b_distance_mm>=8000&&HALF_WIDTH<150)
       { flag1 = 2;}

    if(Length_ramp>=23000)
    {
        flag1=0;
        Length_ramp=0;
    }

//    if(flag1 == 2)
//    {
//        podao_time++;
//    }
//    if(podao_time>=50)
//    {
//        flag1=0;
//        podao_time=0;
//    }

}



//-----------障碍物------------
float Get_angle(float Ax, float Ay, float Bx, float By, float Cx, float Cy)
{
    float BA = sqrt((Ax - Bx) * (Ax - Bx) + (Ay - By) * (Ay - By));
    float BC = sqrt((Cx - Bx) * (Cx - Bx) + (Cy - By) * (Cy - By));

    float SBA_BC = (Ax - Bx) * (Cx - Bx) + (Ay - By) * (Cy - By);

    float angle = acos(SBA_BC / (BA * BC));

    return angle * 57.3; // Convert angle from radians to degrees
}


uint8 enable_L_corner1 = 1, enable_R_corner1 = 1;
void get_turning_point(void)
{
    L_corner_flag1 = 0;// 初始化变量
    L_corner_row1 = 0;
    L_corner_col1 = 0;
    L_corner_angle1 = 0;
    if (enable_L_corner1) //如果使能搜索左拐点
    {
        if (data_stastics_l > 9 && start_point_l[1] >= image_h / 2 && left_lost_num <= image_h / 2)
        {
            for (int i = 0; i < data_stastics_l - 9; i++)
            {
                if (points_l[i + 8][1] > 10)
                {
                    if (((points_l[i][0] - points_l[i + 4][0]) * (points_l[i + 8][0] - points_l[i + 4][0])) *
                        ((points_l[i][1] - points_l[i + 4][1]) * (points_l[i + 8][1] - points_l[i + 4][1]) >= 0)) //初步确认为锐角或者直角 向量法
                    {
                        L_corner_angle1 = Get_angle(points_l[i][0], points_l[i][1], points_l[i + 4][0], points_l[i + 4][1], points_l[i + 8][0], points_l[i + 8][1]); //求角度
                        if (points_l[i + 4][0] > points_l[i + 8][0] && L_corner_angle1 >= 85 && L_corner_angle1 <= 125)
                        {
                            //                            L_corner_angle1_1 = L_corner_angle1;
                            L_corner_flag1 = 1;
                            L_corner_row1 = points_l[i + 4][1];
                            L_corner_col1 = points_l[i + 4][0];
                            break;
                        }
                    }
                }
            }
        }
    }
    R_corner_flag1 = 0;//初始化变量
    R_corner_row1 = 0;
    R_corner_col1 = 0;
    R_corner_angle1 = 0;
    if (enable_R_corner1)    //如果使能搜索右拐点
    {
        if (data_stastics_r > 9 && start_point_r[1] >= image_h / 2 && right_lost_num <= image_h / 2)
        {
            for (int i = 0; i < data_stastics_r - 9; i++)
            {
                if (points_r[i + 8][1] > 10)
                {
                    if (((points_r[i][0] - points_r[i + 4][0]) * (points_r[i + 8][0] - points_r[i + 4][0])) *
                        ((points_r[i][1] - points_r[i + 4][1]) * (points_r[i + 8][1] - points_r[i + 4][1]) >= 0)) //初步确认为锐角或者直角 向量法
                    {
                        R_corner_angle1 = Get_angle(points_r[i][0], points_r[i][1], points_r[i + 4][0], points_r[i + 4][1], points_r[i + 8][0], points_r[i + 8][1]); //求角度
                        if (points_r[i + 8][0] > points_r[i + 4][0] && R_corner_angle1 >= 85 && R_corner_angle1 <= 125)
                        {
                            R_corner_flag1 = 1;
                            R_corner_row1 = points_r[i + 4][1];
                            R_corner_col1 = points_r[i + 4][0];
                            break;
                        }
                    }
                }
            }
        }
    }
}


//单边桥
int ob_err_F = 0;
int ob_err_N = 0;
int ob_err_F1 = 0;
int ob_err_N1 = 0;
int single_mode=0;
int once=0;

extern int bridge_pri;
void Check_obstacle(void)
{
    if(imag[12][2]==0&&imag[10][2]==0&&imag[8][2]==0&&imag[7][2]==0&&

            imag[21][2]==0&&imag[20][2]==0&&imag[19][2]==0&&imag[17][2]==0&&imag[15][2]==0&&imag[4][2]==0

            &&imag[5][2]==0&&Lost_right_Flag==0&&Lost_left_Flag==0&&annulus_L_memory==0&&annulus_R_memory==0

        &&danbianqiaowupan==0&&Crossroad_memory==0&&flag1==0&&zebra_crossing_flag==0&&Check_obstacle_flag==0&&!once)
    {
        ob_err_F = points_r[35][0] - points_l[35][0];
        ob_err_N = points_r[20][0] - points_l[20][0];
        if (ob_err_F < 120 || ob_err_N < 140)
        {
            if ((L_corner_angle1 > 100 && L_corner_angle1 < 110 && R_corner_angle1 > 100 && R_corner_angle1 < 110))
            {
                now_yaw = QEKF_INS.YawTotalAngle;
                SingleBridge_mode=1;
                once=1;

            }
        }
    }

}
//障碍物标志位判断赋值

int ob_err_F_small = 0;// 小障碍专用
int ob_err_N_small = 0;
int ob_err = 13;
int ob_err_time = 100;
void Check_obstacle1(void)
{
  //  if(annulus_L_memory==0&&annulus_R_memory==0&&yuanhuanwupan==0&&Crossroad_memory==0&&flag1==0&&zebra_crossing_flag==0)
//        if(SingleBridge_mode==0 && annulus_L_memory==0 &&zebra_crossing_flag==0)&& SingleBridge_mode==0
//        {
            ob_err_F_small = points_r[55][0] - points_l[55][0];
            ob_err_N_small = points_r[40][0] - points_l[40][0];
        if ( ob_err_F_small < 45|| ob_err_N_small < 95)
        {
            //buzzer();
            if ((L_corner_angle1 > 95 && L_corner_angle1 < 125 && R_corner_angle1 == 0))
            {     //左障碍
                Check_obstacle_flag = 1;
                buzzer();
            }
            if ((R_corner_angle1 > 95 && R_corner_angle1 < 125 && L_corner_angle1 == 0))
            {    //右障碍
                Check_obstacle_flag = 2;
                buzzer();
            }
         }
            if(Check_obstacle_flag == 1)
            {
                zhonxianpiancha =  ob_err;
            }
            if(Check_obstacle_flag == 2)
            {
                zhonxianpiancha = -ob_err;//左转
            }
}

//识别到障碍物对偏差进行平移
void Obstacle_handling(void)
{
//    if ((Check_obstacle_flag == 1))//左障碍
//    {
//        C = 28;
//    }
//    if ((Check_obstacle_flag == 2))//右障碍
//    {
//        C = -28;
//
//    }
//
//    else
//    {
//        C = 0;
//        Check_obstacle_flag = 0;
//    }
}


//=========偏差获取===========
const uint8 Weight[image_h]=
{
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端00 ——09 行权重
        6, 7, 9,11,13,15,17,19,20,20,              //图像最远端30 ——39 行权重
       19,17,15,13,11, 9, 7, 5, 3, 1,              //图像最远端40 ——49 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端10 ——19 行权重
        1, 1, 1, 1, 1, 1, 1, 3, 4, 5,              //图像最远端20 ——29 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端50 ——59 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端60 ——69 行权重

};




uint8_t n_sample = 8;                       //滑动加权滤波算法采样个数
float sample_array[8] = {0};                //采样队列
float A_Sliding_weighted_filter(float xdat) //滑动加权滤波算法
{
  long array_sum = 0;                       //采样队列和
  for(int i=1;i<n_sample;i++)
  {
      sample_array[i-1] = sample_array[i];
      array_sum += sample_array[i] * i;
  }
  sample_array[n_sample-1] = xdat;
  array_sum += xdat * n_sample;
  float filte_value= (array_sum / (11*n_sample/2.0)) * 9 / 7.0;
  return filte_value;
}


double  zuobiaochuli(uint8 center_line[image_h])
{
    double sum;
    sum=
            0.80f*center_line[39]  +
            0.87f*center_line[41]  +
            0.90f*center_line[43]  +
            0.95f*center_line[45]  +
            1.00f*center_line[47]  +
            1.03f*center_line[49]  +
            1.06f*center_line[51]  +
            1.08f*center_line[53]  +
            1.10f*center_line[55]  +
            1.13f*center_line[57]  +
            1.17f*center_line[59]  +
            1.20f*center_line[61]  +
            1.21f*center_line[65]  +
            1.28f*center_line[68]  - 1316;
    sum = sum /14;
 //   sum = A_Sliding_weighted_filter(sum);

/*===================================================元素特殊处理===================================================*/
//    if(hengduan_flag==1)sum=0;
    if(annulus_L_memory == 3)sum=sum-11;
    else  if (annulus_L_memory == 6 )sum=sum-12;
    if(annulus_R_memory == 3)sum=sum+11;
    else  if (annulus_R_memory == 6 )sum=sum+12;

    return sum;

}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     赛道超宽部分数
  @note
-------------------------------------------------------------------------------------------------------------------*/
uint8 Half_width[80] = {
    // 顶部（远场） - 保持较窄的宽度（透视效应）
    18, 18, 18, 18, 18, 18, 18, 18, 18, 18,  // 行0-9
    18, 18, 18, 18, 18, 18, 18, 18, 18, 18,  // 行10-19

    // 中部过渡区 - 宽度逐渐增加
    19, 19, 19, 20, 20, 20, 21, 21, 22, 22,  // 行20-29
    23, 23, 24, 24, 25, 25, 26, 26, 27, 27,  // 行30-39

    // 近场区 - 宽度快速增加
    28, 29, 30, 31, 32, 33, 34, 35, 36, 37,  // 行40-49
    38, 39, 40, 41, 42, 43, 44, 45, 46, 47,  // 行50-59

    // 底部（最近） - 最大宽度
    48, 49, 50, 51, 52, 53, 54, 55, 56, 57,  // 行60-69
    58, 59, 60, 61, 62, 63, 64, 65, 66, 67   // 行70-79
};
int HALF_WIDTH;
void Extrawide_counting1(void) {
    int i;

    // 存储每行宽度的数组（动态分配或全局定义更佳）

    // 从起始行到图像底部遍历
    for (i = Endline; i < image_h - 1; i++) {
        // 边界值约束（防止越界）
        if (right[i] > 188) right[i] = 188;
        if (right[i] < 0) right[i] = 0;
        if (left[i] > 188) left[i] = 188;
        if (left[i] < 0) left[i] = 0;

        // 计算当前行宽度并存储
        HALF_WIDTH = right[i] - left[i];
    }

}





//===================================================显  示===================================================
//最大分辨率 135 * 240
void IPS_show(void)
{
    if(image_close)
    {
       int i;
       //显示图像
       ips200_show_gray_image(0, 0, imag, image_w, image_h, image_w+6, image_h*2, 0);
       for (i =Endline ; i < image_h-1; i++)
       {
           if(right[i]>188)right[i]=188;
           if(right[i]<0)right[i]=0;

           if(left[i]>188)left[i]=188;
           if(left[i]<0)left[i]=0;

           if(middle[i]>188)middle[i]=185;
           if(middle[i]<0)middle[i]=0;

           ips200_draw_point(middle[i], i,RGB565_GREEN);//显示起点 显示中线
           ips200_draw_point(left[i]   , i,RGB565_RED);//显示起点 显示左边线
           ips200_draw_point(right[i]   , i,RGB565_PURPLE );//显示起点 显示右边线
       }
       ips200_show_float(190,  0, zhonxianpiancha, 3, 1);
    }
}

//===================================================元素识别===================================================
void Element_recognition(void)
{
    Mid_Col();
    inflection_point(); //拐点判断
    left_straight();    //左直线
    right_straight();   //右直线
    get_turning_point();//对障碍物处理基本角度进行计算-十字也要用到
    left_dandiao();     //左单调
    right_dandiao();    //右单调
    Check_Zhidao();     //直道识别
    check_bend_dir();   //弯道识别
    middle_line();      //中线数组求取

//    zhonxianpiancha=zuobiaochuli(middle);//中线偏差求取
//
//    crossroad();        //十字处理
//
//    danbianqiaowupan=danbianqiao_wupan();//单边桥误判处理
//    if(!flag1&&!Right_straight_flag&&!Left_straight_flag)
//    Check_obstacle();   //判断单边桥
//    if(!flag1&&once==0&&!hengduan_flag&&!flag1)
//    {Check_obstacle1();} //小障碍
    // if(!hengduan_flag&&!SingleBridge_mode&&!Check_obstacle_flag&&!annulus_L_Flag&&!annulus_R_Flag&&!flag1)
    // {hengduan();}         //横断障碍跳跃识别



//    podao();//坡道

//    if(!flag1){
//   annulus_L();        //左圆环
   // annulus_R();        //右圆环
   // }

//    Extrawide_counting1();
  // zebra_crossing(imag);//斑马线



//    //元素识别蜂鸣器
//    if((hengduan_flag||annulus_L_memory==2||annulus_L_memory==7||annulus_R_memory==2||annulus_R_memory==7||hengduan_flag||Crossroad_memory==1||flag1||Check_obstacle_flag)&&zebra_crossing_flag==0)
//        gpio_set_level(P33_10, 1);
//    else
//        gpio_set_level(P33_10, 0);

}

int width;
//===================================================图像处理===================================================
void image_process(void)
{
            Get_image(mt9v03x_image);
            image_filter(imag[image_w]);//形态学滤波，膨胀和腐蚀的思想
            Get01change_Dajin();
            image_draw_rectan1(imag);//预处理-边界画黑框
            data_stastics_l = 0;
            data_stastics_r = 0;

            if (get_start_point(image_h - 2))//找到起点了，再执行八领域，没找到就一直找
            {
                search_l_r((uint16)USE_num, imag, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &Endline);
                get_left(data_stastics_l);
                get_right(data_stastics_r);
                Finish_Flag=1;
            }
            lost_left();
            lost_right();
            Element_recognition(); //元素处理

           // IPS_show();//图像显示

}





