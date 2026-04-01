#include "zf_common_headfile.h"
#include "math.h"

float Err;
uint8 L_start_y = 0;//初始点
uint8 L_start_x = 0;
uint8 R_start_y = 0;
uint8 R_start_x = 0;

uint8 wx[MT9V03X_H]={0};
/*-------------------------------------------------------------------------------------------------------------------
  @brief     中线获取
  @note
-------------------------------------------------------------------------------------------------------------------*/
void Midline_get()
{
    int i;
    int pingyizhuxian;
    for(i=MT9V03X_H-2;i>1;i--)
    {
        wx[i]=( zx[i]+yx[i] )/2;//中线数组
    }

//    if(circle_flag!=1&&Cross_Flag!=1&&Barrier_Flag!=1&&Zebra_Flag!=1&&Ramp_Flag!=1&&SingleBridge_Flag!=1&&Ramp_End_Flag!=1&&Jumpbarrier_Flag!=1)//元素互斥
//    {
//        if(Left_Lost_Time>=2&&Right_Lost_Time==0)//左弯 通过右边界偏移
//        {
//            for(int i=Boundry_Startlost_Left;i>Finnalline;i--)
//             {
//                if(yx[i]-Half_zhidao_width[i]-20>=2)
//                    wx[i]=yx[i]-Half_zhidao_width[i]-20;
//                else wx[i]=2;
//             }
//        }
//        if(Right_Lost_Time>=2&&Left_Lost_Time==0)//右弯 通过左边界偏移
//        {
//            for(int i=Boundry_Startlost_right;i>Finnalline;i--)
//             {
//                if(zx[i]+Half_zhidao_width[i]+20<=MT9V03X_W-3)
//                    wx[i]=zx[i]+Half_zhidao_width[i]+20;
//                else wx[i]=MT9V03X_W-3;
//             }
//        }
//    }

    for(i=Finnalline;i>1;i--)
    {
        pingyizhuxian=Finnalline+3;
        if(pingyizhuxian>=MT9V03X_H-3) pingyizhuxian=MT9V03X_H-3;
        wx[i]=wx[pingyizhuxian];
    }
}

//大津法求阈值
uint8 Threshold = 0;
uint8 Threshold_Lup = 0;
uint8 Threshold_Rup = 0;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      qiqi爆改快速大津
//  @return     uint8
//  @since      v1.1
//  Sample usage:   OTSU_Threshold = otsuThreshold(mt9v03x_image,0,MT9V03X_H,0,MT9V03X_W);
//-------------------------------------------------------------------------------------------------------------------
uint8 otsuThreshold(uint8 *ima,uint8 h_start,uint8 h_end,uint8 w_start,uint8 w_end)   //注意计算阈值的一定要是原图像
{
    #define GrayScale 256
    int Pixel_Max=0;
    int Pixel_Min=255;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = (w_end-w_start) * (h_end-h_start)/4;
    uint8 threshold = 0;
    uint8* data = ima;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    uint32 gray_sum=0;
    //统计灰度级中每个像素在整幅图像中的个数
    for (i = h_start; i < h_end; i+=2)
    {
        for (j = w_start; j < w_end; j+=2)
        {
            pixelCount[(int)data[i * MT9V03X_W + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * MT9V03X_W + j];       //灰度值总和
            if(data[i * MT9V03X_W + j]>Pixel_Max)   Pixel_Max=data[i * MT9V03X_W + j];
            if(data[i * MT9V03X_W + j]<Pixel_Min)   Pixel_Min=data[i * MT9V03X_W + j];
        }
    }

    //计算每个像素值的点在整幅图像中的比例

    for (i = Pixel_Min; i < Pixel_Max; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;

    }

    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;

    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = Pixel_Min; j < Pixel_Max; j++)
    {

        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值

        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;

        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = (float)(w0 *w1* (u0 - u1)* (u0 - u1)) ;
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }

    }

        if(threshold>255)
               threshold=255;
        if(threshold<0)
               threshold=0;

    return threshold;
}




/*-------------------------------------------------------------------------------------------------------------------
  @brief     二值化
  @note
-------------------------------------------------------------------------------------------------------------------*/
//二值化数组
uint8 image[MT9V03X_H][MT9V03X_W];
void ZeroOne(uint8 *ima,uint8 h_start,uint8 height,uint8 w_start,uint8 width,uint8 yuzhi)
{
    uint8* data = ima;  //指向像素数据的指针
    for(uint8 i=h_start;i<height;i++)
    {
        for(uint8 j=w_start;j<width;j++)
        {
            if(data[i*MT9V03X_W+j]>=yuzhi) image[i][j]=255;//白
            else image[i][j]=0;//黑
        }
    }
//    if(circle_flag!=1&&Cross_Flag!=1&&Barrier_Flag!=1&&Zebra_Flag!=1&&Ramp_Flag!=1&&SingleBridge_Flag!=1&&Ramp_End_Flag!=1&&Jumpbarrier_Flag!=1)
//    {
//        for(uint8 i=h_start;i<20;i++)
//        {
//            for(uint8 j=w_start;j<width;j++)
//            {
//                image[i][j]=0;
//            }
//        }
//    }
}


//分块二值化
void fenkuai_ZeroOne(uint8 height,uint8 width,uint8 leftyuzhi,uint8 midyuzhi,uint8 rightyuzhi)
{
    uint8 i,j;
    for(i=0;i<height;i++)
    {
        for(j=0;j<width/3;j++)
        {
            if(mt9v03x_image[i][j]>=leftyuzhi) image[i][j]=255;//白
            else image[i][j]=0;//黑
        }
        for(j=width/3;j<2*width/3;j++)
        {
            if(mt9v03x_image[i][j]>=midyuzhi) image[i][j]=255;//白
            else image[i][j]=0;//黑
        }
        for(j=2*width/3;j<width;j++)
        {
            if(mt9v03x_image[i][j]>=rightyuzhi) image[i][j]=255;//白
            else image[i][j]=0;//黑
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     巡线
  @note
-------------------------------------------------------------------------------------------------------------------*/
//寻左右边线
uint8 zx[MT9V03X_H]={0};
uint8 yx[MT9V03X_H]={0};
uint8 Left_Lost_Flag[MT9V03X_H],Right_Lost_Flag[MT9V03X_H];
uint8 left_border,right_border,midline;
uint8 Finnalline;
uint8 leftlost_mode;
uint8 rightlost_mode;
uint8 cross_xunxian_mode=0;
void XUNXIAN(uint8 height,uint8 width)
{
    leftlost_mode=0;
    rightlost_mode=0;
    uint8 leftpoint=0;
    uint8 rightpoint=0;
    cross_xunxian_mode=0;
    Finnalline=0;
    midline=MT9V03X_W/2; //初始化中线为绝对中值
    volatile int White_Column[MT9V03X_W]={0};//每列白列长度
    uint8 Longest_White_Column_Left[2]={0};
    uint8 Longest_White_Column_Right[2]={0};
    uint8 cnt=0;
    if(left_circle_flag==1&&circle_mode==3)
    {
        midline=MT9V03X_W/4;
    }
    if(right_circle_flag==1&&circle_mode==3)
    {
        midline=3*MT9V03X_W/4;
    }

    for(uint8 h=height-1;h>0;h--)//从下往上扫线 大概率不用改
    {
        uint8 left_cross_lost=0;
        uint8 right_cross_lost=0;
        if(h >= height-1)//找到第一个左右边界
        {
            for(uint8 w=midline;w>=2;w--)
            {
                if(image[h][w-2]==0&&image[h][w-1]==0&&image[h][w]==255)
                {
                    left_border=w;
                    Left_Lost_Flag[h] = 0; //左丢线数组，丢线置1，不丢线置0
                    break;
                }
                else if(w<=2&&image[h][w]==255)
                {
                    leftlost_mode=1;
                    left_border=1;
                    Left_Lost_Flag[h] = 1; //左丢线数组，丢线置1，不丢线置0
                    break;
                }
                else if(w<=2&&image[h][w]==0)
                {
//                    leftlost_mode=2;
                    left_border=1;
                    Left_Lost_Flag[h] = 0; //左丢线数组，丢线置1，不丢线置0
                    break;
                }
            }
            for(uint8 w=midline;w<=width-2-1;w++)
            {
                if(image[h][w+2]==0&&image[h][w+1]==0&&image[h][w]==255)
                {
                    right_border=w;
                    Right_Lost_Flag[h] = 0; //右丢线数组，丢线置1，不丢线置0
                    break;
                }
                else if(w>=width-2-1&&image[h][w]==255)
                {
                    rightlost_mode=1;
                    right_border=width-2;
                    Right_Lost_Flag[h] = 1; //右丢线数组，丢线置1，不丢线置0
                    break;
                }
                else if(w>=width-2-1&&image[h][w]==0)
                {
//                    rightlost_mode=2;
                    right_border=width-2;
                    Right_Lost_Flag[h] = 0; //右丢线数组，丢线置1，不丢线置0
                    break;
                }
            }
        }
        /////////////////////后续找左右边界////////////////////
         if(h < height-1&&leftlost_mode!=2&&rightlost_mode!=2&&cross_xunxian_mode!=1)
         {
             leftpoint=left_border+5;
             if(leftpoint>width-1) leftpoint=width-1;
             for(uint8 w=leftpoint;w>=2;w--)//leftpoint
             {
                 if(image[h][w-2]==0&&image[h][w-1]==0&&image[h][w]==255)
                 {
                     left_border=w;
                     Left_Lost_Flag[h] = 0; //左丢线数组，丢线置1，不丢线置0
                     break;
                 }
                 else if(w<=2&&image[h][w]==255)
                 {
//                     left_cross_lost=1;
                     left_border=1;
                     Left_Lost_Flag[h] = 1; //左丢线数组，丢线置1，不丢线置0
                     break;
                 }
             }
             rightpoint=right_border-5;
             if(rightpoint<0) rightpoint=0;
             for(uint8 w=rightpoint;w<=width-2-1;w++)//rightpoint
             {
                 if(image[h][w+2]==0&&image[h][w+1]==0&&image[h][w]==255)
                 {
                     right_border=w;
                     Right_Lost_Flag[h] = 0; //右丢线数组，丢线置1，不丢线置0
                     break;
                 }
                 else if(w>=width-2-1&&image[h][w]==255)
                 {
//                     right_cross_lost=1;
                     right_border=width-2;
                     Right_Lost_Flag[h] = 1; //右丢线数组，丢线置1，不丢线置0
                     break;
                 }
             }
             if(left_cross_lost==1&&right_cross_lost==1) cross_xunxian_mode=1;
         }



        if(h <= height-1&&leftlost_mode==2)
        {//左边界异常
//找到第二个左右边界
            for(uint8 w=width-1;w>=2;w--)
            {
                if(image[h][w-2]==0&&image[h][w-1]==0&&image[h][w]==255)//第二个左边界
                {
                    left_border=w;
                    Left_Lost_Flag[h] = 0; //左丢线数组，丢线置1，不丢线置0
                    break;
                }
                else if(w<=2)
                {
                    left_border=1;
                    Left_Lost_Flag[h] = 1; //左丢线数组，丢线置1，不丢线置0
                    break;
                }
            }

            rightpoint=right_border-5;
            if(rightpoint<0) rightpoint=0;
            for(uint8 w=rightpoint;w<=width-2-1;w++)//第二个右边界
            {
                if(image[h][w+2]==0&&image[h][w+1]==0&&image[h][w]==255)
                {
                    right_border=w;
                    Right_Lost_Flag[h] = 0; //右丢线数组，丢线置1，不丢线置0
                    break;
                }
                else if(w>=width-2-1)
                {
                    right_border=width-2;
                    Right_Lost_Flag[h] = 1; //右丢线数组，丢线置1，不丢线置0
                    break;
                }
            }
            leftlost_mode=0;
        }

        if(h <= height-1&&rightlost_mode==2)
        {//右边界异常
//找到第二个左右边界
            leftpoint=left_border+5;
            if(leftpoint>width-1) leftpoint=width-1;
            for(uint8 w=leftpoint;w>=2;w--)
            {
                if(image[h][w-2]==0&&image[h][w-1]==0&&image[h][w]==255)//第二个左边界
                {
                    left_border=w;
                    Left_Lost_Flag[h] = 0; //左丢线数组，丢线置1，不丢线置0
                    break;
                }
                else if(w<=2)
                {
                    left_border=1;
                    Left_Lost_Flag[h] = 1; //左丢线数组，丢线置1，不丢线置0
                    break;
                }
            }

            for(uint8 w=0;w<=width-2-1;w++)//第二个右边界
            {
                if(image[h][w+2]==0&&image[h][w+1]==0&&image[h][w]==255)
                {
                    right_border=w;
                    Right_Lost_Flag[h] = 0; //右丢线数组，丢线置1，不丢线置0
                    break;
                }
                else if(w>=width-2-1)
                {
                    right_border=width-2;
                    Right_Lost_Flag[h] = 1; //右丢线数组，丢线置1，不丢线置0
                    break;
                }
            }
            rightlost_mode=0;
        }

        zx[h]=left_border;//左边界数组
        yx[h]=right_border;//右边界数组

    }
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     只寻一小部分线
  @note
-------------------------------------------------------------------------------------------------------------------*/
uint8 Longest_White_Column_Left[2]={0};
uint8 Longest_White_Column_Right[2]={0};
void XUNXIAN_MODE1(uint8 height,uint8 width)
{
    leftlost_mode=0;
    rightlost_mode=0;
    midline=MT9V03X_W/2; //初始化中线为绝对中值
    Finnalline=5;
    uint8 leftpoint=0;
    uint8 rightpoint=0;
    uint8 h=height-1;//起始点
    uint8 i,j;
    volatile int White_Column[MT9V03X_W]={0};//每列白列长度
    for (i=2;i<=MT9V03X_W-2-1;i+=2)
    {
        for (j=MT9V03X_H-2;j >= 0; j--)
        {
            if(image[j][i] == 0)
                break;
            else
                White_Column[i]++;
        }
    }
    //从左到右找左边最长白列
     Longest_White_Column_Left[0] =0;
     for(i=2;i<=MT9V03X_W-2-1;i+=2)
     {
         if (Longest_White_Column_Left[0] < White_Column[i])//找最长的那一列
         {
             Longest_White_Column_Left[0] = White_Column[i];//【0】是白列长度
             Longest_White_Column_Left[1] = i;              //【1】是下标，第j列
         }
     }

    for(uint8 w=midline;w>=2;w--)
    {
        if(image[h][w-2]==0&&image[h][w-1]==0&&image[h][w]==255)//黑黑白
        {
            left_border=w;
            break;
        }
        else if(w<=2&&image[h][w]==255)//白
        {
            leftlost_mode=1;
            left_border=2;
            break;
        }
        else if(w<=2&&image[h][w]==0)//黑
        {
            leftlost_mode=2;
            left_border=2;
            break;
        }
    }
    for(uint8 w=midline;w<=width-2-1;w++)
    {
        if(image[h][w+2]==0&&image[h][w+1]==0&&image[h][w]==255)//黑黑白
        {
            right_border=w;
            break;
        }
        else if(w>=width-2-1&&image[h][w]==255)//白
        {
            rightlost_mode=1;
            right_border=width-2-1;
            break;
        }
        else if(w>=width-2-1&&image[h][w]==0)//黑
        {
            rightlost_mode=2;
            right_border=width-2-1;
            break;
        }
    }
    if(leftlost_mode==2)
    {//左边界异常
//找到第二个左右边界
        for(uint8 w=width-2-1;w>=2;w--)
        {
            if(image[h][w-2]==0&&image[h][w-1]==0&&image[h][w]==255)//第二个左边界
            {
                left_border=w;
                break;
            }
            else if(w<=2)
            {
                left_border=2;
                break;
            }
        }

        rightpoint=right_border-5;
        if(rightpoint<2) rightpoint=2;
        for(uint8 w=rightpoint;w<=width-2-1;w++)
        {
            if(image[h][w+2]==0&&image[h][w+1]==0&&image[h][w]==255)//第二个右边界
            {
                right_border=w;
                break;
            }
            else if(w>=width-2-1)
            {
                right_border=width-2-1;
                break;
            }
        }
        leftlost_mode=0;
    }

    if(rightlost_mode==2)
    {//右边界异常
//找到第二个左右边界
        leftpoint=left_border+5;
        if(leftpoint>width-2-1) leftpoint=width-2-1;
        for(uint8 w=leftpoint;w>=2;w--)
        {
            if(image[h][w-2]==0&&image[h][w-1]==0&&image[h][w]==255)//第二个左边界
            {
                left_border=w;
                break;
            }
            else if(w<=2)
            {
                left_border=2;
                break;
            }
        }

        for(uint8 w=2;w<=width-2-1;w++)//第二个右边界
        {
            if(image[h][w+2]==0&&image[h][w+1]==0&&image[h][w]==255)//第二个右边界
            {
                right_border=w;
                break;
            }
            else if(w>=width-2-1)
            {
                right_border=width-2-1;
                break;
            }
        }
        rightlost_mode=0;
    }
    for(;h>0;h--)
    {
        if(h>=1&&image[h-1][ Longest_White_Column_Left[1] ]==0)
        {
            Finnalline=h;//截至行
            break;
        }
    }
}

void XUNXIAN_cross(uint8 height,uint8 width)
{
    int i,j;
    volatile int White_Column[MT9V03X_W]={0};//每列白列长度
    uint8 Longest_White_Column_Left[2]={0};
    uint8 Longest_White_Column_Right[2]={0};
    for(i=30; i<=MT9V03X_W-30; i++)
    {
         for (j = MT9V03X_H - 5; j >= 0; j--)
         {
             if(image[j][i] == 0)
                 break;
             else
                 White_Column[i]++;
         }
    }
    Longest_White_Column_Left[0] =0;
    Longest_White_Column_Right[0] = 0;
    if(Boundry_Start_Left==MT9V03X_H-2&&Boundry_Start_Right!=MT9V03X_H-2)//车身姿态偏左 理论白列偏右
    {
        for(i=MT9V03X_W/2+30;i<=MT9V03X_W-20;i++)
        {
             if (Longest_White_Column_Left[0] < White_Column[i])//找最长的那一列
             {
                 Longest_White_Column_Left[0] = White_Column[i];//【0】是白列长度
                 Longest_White_Column_Left[1] = i;              //【1】是下标，第j列
             }
        }
        for(i=MT9V03X_W-20;i>=MT9V03X_W/2+30;i--)//从右往左，注意条件，找到左边最长白列位置就可以停了
        {
             if (Longest_White_Column_Right[0] < White_Column[i])//找最长的那一列
             {
                 Longest_White_Column_Right[0] = White_Column[i];//【0】是白列长度
                 Longest_White_Column_Right[1] = i;              //【1】是下标，第j列
             }
        }
    }
    else if(Boundry_Start_Left!=MT9V03X_H-2&&Boundry_Start_Right==MT9V03X_H-2)//车身姿态偏右 理论白列偏左
    {
        for(i=20;i<=MT9V03X_W/2-30;i++)
        {
             if (Longest_White_Column_Left[0] < White_Column[i])//找最长的那一列
             {
                 Longest_White_Column_Left[0] = White_Column[i];//【0】是白列长度
                 Longest_White_Column_Left[1] = i;              //【1】是下标，第j列
             }
        }
        for(i=MT9V03X_W/2-30;i>=20;i--)//从右往左，注意条件，找到左边最长白列位置就可以停了
        {
             if (Longest_White_Column_Right[0] < White_Column[i])//找最长的那一列
             {
                 Longest_White_Column_Right[0] = White_Column[i];//【0】是白列长度
                 Longest_White_Column_Right[1] = i;              //【1】是下标，第j列
             }
        }
    }
    else//车身姿态居中 理论白列居中
    {
        for(i=70;i<=MT9V03X_W-60;i++)
        {
             if (Longest_White_Column_Left[0] < White_Column[i])//找最长的那一列
             {
                 Longest_White_Column_Left[0] = White_Column[i];//【0】是白列长度
                 Longest_White_Column_Left[1] = i;              //【1】是下标，第j列
             }
        }
        for(i=MT9V03X_W-70;i>=60;i--)//从右往左，注意条件，找到左边最长白列位置就可以停了
        {
             if (Longest_White_Column_Right[0] < White_Column[i])//找最长的那一列
             {
                 Longest_White_Column_Right[0] = White_Column[i];//【0】是白列长度
                 Longest_White_Column_Right[1] = i;              //【1】是下标，第j列
             }
        }
    }

    for(uint8 h=height-1;h>0;h--)//从下往上扫线 大概率不用改
    {
        for(uint8 w=Longest_White_Column_Left[1];w>=2;w--)
        {
            if(image[h][w-2]==0&&image[h][w-1]==0&&image[h][w]==255)
            {
              left_border=w;
              Left_Lost_Flag[h] = 0; //左丢线数组，丢线置1，不丢线置0
              break;
            }
            else if(w<=2&&image[h][w]==255)
            {
              left_border=1;
              Left_Lost_Flag[h] = 1; //左丢线数组，丢线置1，不丢线置0
              break;
            }
        }
        for(uint8 w=Longest_White_Column_Right[1];w<=width-2-1;w++)
        {
            if(image[h][w+2]==0&&image[h][w+1]==0&&image[h][w]==255)
            {
              right_border=w;
              Right_Lost_Flag[h] = 0; //右丢线数组，丢线置1，不丢线置0
              break;
            }
            else if(w>=width-2-1&&image[h][w]==255)
            {
              right_border=width-2;
              Right_Lost_Flag[h] = 1; //右丢线数组，丢线置1，不丢线置0
              break;
            }
        }
        zx[h]=left_border;//左边界数组
        yx[h]=right_border;//右边界数组
        if(h>=1&&image[h-1][ Longest_White_Column_Left[1]]==0)
        {
            Finnalline=h;//截至行
            break;
        }
    }
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief    处理赛道信息
  @note
-------------------------------------------------------------------------------------------------------------------*/
uint8 Left_Lost_Time,Right_Lost_Time,Both_Lost_Time,
    Boundry_Start_Left,Boundry_Start_Right,Boundry_Startlost_Left,Boundry_Startlost_right,Sum_Lost_Time;
/////处理赛道信息
void Track_Analysis()
{
    uint8 i;
    Left_Lost_Time=Right_Lost_Time=Both_Lost_Time=Sum_Lost_Time=Boundry_Start_Left
            =Boundry_Start_Right=Boundry_Startlost_Left=Boundry_Startlost_right=0;
    for (i = MT9V03X_H - 2; i > Finnalline; i--)//赛道数据初步分析
      {
          if (zx[i]  == 1)//单边丢线数
              Left_Lost_Time++;
          if (yx[i] == MT9V03X_W-2)
              Right_Lost_Time++;
          if (zx[i]  == 1 && yx[i] == MT9V03X_W-2)//双边丢线数
              Both_Lost_Time++;

          if (Boundry_Startlost_Left ==  0 && zx[i]  == 1)//记录第一个丢线点，边界起始点
              Boundry_Startlost_Left = i;
          if (Boundry_Startlost_right ==  0 && yx[i] == MT9V03X_W-2)
              Boundry_Startlost_right = i;

          if (Boundry_Start_Left ==  0 && zx[i]  != 1)//记录第一个非丢线点，边界起始点
              Boundry_Start_Left = i;
          if (Boundry_Start_Right == 0 && yx[i] != MT9V03X_W-2)
              Boundry_Start_Right = i;
      }
//    if(Left_Lost_Time>=Right_Lost_Time)
//    {
//        Both_Lost_Time = Right_Lost_Time;
//    }
//    else
//    {
//        Both_Lost_Time = Left_Lost_Time;
//    }


}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     弯道判断
  @note
-------------------------------------------------------------------------------------------------------------------*/
uint8 left_control_flag=0;
uint8 right_control_flag=0;
void is_wangdao()
{
    uint8 i;
    left_control_flag=0;
    right_control_flag=0;
    //*******************************************判断弯道***********************************************************//
    //           Road_Wide[i]=Right_Line[i]-Left_Line[i];
    if(!trueshortflag)
    {
        if( (Left_Lost_Time>Right_Lost_Time)// 还可以加一个斜率变化明显 非必要不加 容易算出无穷大
                &&Left_Lost_Time>20
                &&Both_Lost_Time<3
                &&Finnalline!=0
                &&Boundry_Start_Right>= Boundry_Start_Left
                                                          )
        {
           left_control_flag=1; //左转弯
        }
        if( (Right_Lost_Time>Left_Lost_Time)// 还可以加一个斜率变化明显 非必要不加 容易算出无穷大
                &&Right_Lost_Time>20
                &&Both_Lost_Time<3
                &&Finnalline!=0
                &&Boundry_Start_Left>= Boundry_Start_Right
                                                          )
        {
           right_control_flag=1; //右转弯
        }

//         if(left_control_flag==1)
//         {
//             for(i=Boundry_Startlost_Left;i>0;i--)
//             {
//                if(yx[i]-Half_wangdao_width[i]>=2)
//                    wx[i]=yx[i]-Half_wangdao_width[i];
//                else wx[i]=2;
//             }
//         }
     //
//         if(right_control_flag==1)
//         {
//             for(i=Boundry_Startlost_right;i>0;i--)
//             {
//                if(zx[i]+Half_wangdao_width[i]<=MT9V03X_W-3)
//                    wx[i]=zx[i]+Half_wangdao_width[i];
//                else wx[i]=MT9V03X_W-3;
//             }
//         }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     直道判断
  @note
-------------------------------------------------------------------------------------------------------------------*/
uint8 shortflag=0;
uint8 trueshortflag=0;
void is_straight()
{
    //**********************************************判断直道*********************************************************//
   if(!left_control_flag&&!right_control_flag)
   {
       uint8 shortwrong = 0;
       float kfirst=0;
       float ksecond=0;
       float kerror=0;
//       advanced_regression(0, 4, 8, 12, 16);
       advanced_regression(0, 25, 27, 29, 31);
       kfirst = parameterB;

//       advanced_regression(0, 18, 22, 26, 30);
       advanced_regression(0, 33, 35, 37, 39);
       ksecond = parameterB;

       if (FMy_Abs(0, kfirst) >= 0.7 || FMy_Abs(0, ksecond) >= 0.7) { shortwrong = 1; }
       kerror = FMy_Abs(kfirst, ksecond);

       if (kerror <= 0.3) shortflag = shortflag + 1;
       if (kerror > 0.3) { shortflag = 0; }
       if (shortwrong == 1) { shortflag = 0; }
       if (shortflag >= 2) { shortflag = 2; trueshortflag = 1; }
       if (shortflag < 2) trueshortflag = 0;
   }
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     拟合函数
  @note
-------------------------------------------------------------------------------------------------------------------*/
float parameterB=0;
float parameterA=0;
void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2) //斜率函数
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
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += wx[i];
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += wx[i];
        }
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x的平均值
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y的平均值
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (wx[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (wx[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;

    }
    else if (type == 1)     //拟合左线
    {
        /**计算sumX sumY**/
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += zx[i];
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += zx[i];
        }
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x的平均值
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y的平均值
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (zx[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (zx[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 2)         //拟合右线
    {
        /**计算sumX sumY**/
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += yx[i];
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += yx[i];
        }
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x的平均值
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y的平均值
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (yx[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (yx[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     求两数之差绝对值的浮点数
  @note
-------------------------------------------------------------------------------------------------------------------*/
float FMy_Abs(float a, float b)//求两数之差绝对值的浮点数
{

    if ((a - b) > 0)
        return ((float)(a - b));
    else return ((float)(b - a));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      给图像画黑框为迷宫法做准备
//  @return     void
//  @since      v1.0
//  Sample usage:   image_draw_rectan(Image_use);
//-------------------------------------------------------------------------------------------------------------------
void image_draw_rectan()
{
    uint8 i = 0;
    //局部阈值时候开启
    for (i = 0; i < MT9V03X_H; i++)//左右1行
    {
        mt9v03x_image[i][0] = 0;
        mt9v03x_image[i][MT9V03X_W - 1] = 0;
    }
    for (i = 0; i < MT9V03X_W; i++)//上下1行
    {
        mt9v03x_image[0][i] = 0;
        mt9v03x_image[MT9V03X_H-1][i] = 0;
    }
    //局部阈值时候开启


}
void image_draw_rectan_zeroone()
{
    uint8 i = 0;
    //二值化迷宫的时候开启
    for (i = 0; i < MT9V03X_H; i++)
    {
        image[i][0] = 0;
        image[i][MT9V03X_W - 1] = 0;
    }
    for (i = 0; i < MT9V03X_W; i++)
    {
        image[0][i] = 0;
        image[MT9V03X_H-1][i] = 0;
    }
    //二值化迷宫的时候开启
}

/*---------------------------------------------------------------
 【函    数】search_neighborhood 局部阈值版
 【功    能】迷宫法找边界
 【参    数】无
 【返 回 值】无
 【注意事项】
 ----------------------------------------------------------------*/
struct LEFT_EDGE
{
    int16 row;  //行坐标
    int16 col;  //列坐标
};
struct RIGHT_EDGE
{
    int16 row;  //行坐标
    int16 col;  //列坐标
};
uint8 L_edge_row[countnum],L_edge_col[countnum];
uint8 R_edge_row[countnum],R_edge_col[countnum];
struct LEFT_EDGE  L_edge[countnum];     //左边界结构体
struct RIGHT_EDGE R_edge[countnum];    //右边界结构体
uint8 L_edge_count=0, R_edge_count = 0;                  //左右边点的个数
uint8 new_L_edge_count=0, new_R_edge_count=0;
uint8 dire_left,dire_right;                                 //记录上一个点的相对位置
uint8 turn_left,turn_right;
uint8 L_search_amount = countnum, R_search_amount = countnum;  //左右边界搜点时最多允许的点

const int dir_front[4][2] = {{0,  -1},
                             {1,  0},
                             {0,  1},
                             {-1, 0}};
const int dir_frontleft[4][2] = {{-1, -1},
                                {1,  -1},
                                {1,  1},
                                {-1, 1}};
const int dir_frontright[4][2] = {{1,  -1},
                                 {1,  1},
                                 {-1, 1},
                                 {-1, -1}};
#define clip_value (7)
void search_neighborhood(void)
{
    L_edge_count = 0;//左边点个数清0
    R_edge_count = 0;//右边点个数清0
    new_L_edge_count=0;
    new_R_edge_count=0;
    uint8 zx_flag[MT9V03X_H]={0};
    uint8 yx_flag[MT9V03X_H]={0};
    //变量声明
    uint8 new_row=0;
    uint8 new_col=0;
    uint8 cnt_left;
    uint8 cnt_right;
    L_edge[0].row = L_start_y;
    L_edge[0].col = L_start_x;
    L_edge_count+=1;
    int16 curr_row = L_start_y;//初始化行坐标
    int16 curr_col = L_start_x;//初始化列坐标
    zx[curr_row]=curr_col;
    zx_flag[curr_row]=1;
    dire_left = 0; //初始化上个边界点的来向
    turn_left = 0;//转向次数 防止死循环
    //开始搜线，最多取150个点，不会往下搜，共7个方位
    while(L_edge_count < L_search_amount)    //最多搜索150个点
    {
        if(turn_left>=4) break;
        int local_thres = 0;
        for(int dy = -1; dy <= 1; dy++)
        {
            for(int dx = -1; dx <= 1; dx++)
            {
                local_thres += mt9v03x_image[curr_row + dy][curr_col + dx];
            }
        }
        local_thres /= 9;
        local_thres -= clip_value;
        int front_value = mt9v03x_image[ curr_row + dir_front[dire_left][1] ][ curr_col + dir_front[dire_left][0] ];
        int frontleft_value = mt9v03x_image[ curr_row + dir_frontleft[dire_left][1] ][ curr_col +  dir_frontleft[dire_left][0] ];
        //搜线过程
        if(front_value<local_thres)   //上黑
        {
            dire_left = (dire_left+1)%4;
            turn_left++;
        }
        else if(frontleft_value<local_thres)    //左上黑
        {
            curr_row = curr_row + dir_front[dire_left][1];
            curr_col = curr_col + dir_front[dire_left][0];
            ////越界退出 行越界和列越界（向上向下向左向右）
            if(curr_row <1||curr_row>MT9V03X_H-2||curr_col<1||curr_col>MT9V03X_W-2||curr_row <= Finnalline+1)  break;
            L_edge[L_edge_count].row = curr_row;
            L_edge[L_edge_count].col = curr_col;
            L_edge_count = L_edge_count + 1;
            turn_left = 0;
        }
        else                  //上白 左上白
        {
            curr_row = curr_row + dir_frontleft[dire_left][1];
            curr_col = curr_col + dir_frontleft[dire_left][0];
            dire_left = (dire_left + 3) % 4;
            ////越界退出 行越界和列越界（向上向下向左向右）
            if(curr_row <1||curr_row>MT9V03X_H-2||curr_col<1||curr_col>MT9V03X_W-2||curr_row <= Finnalline+1)  break;
            L_edge[L_edge_count].row = curr_row;
            L_edge[L_edge_count].col = curr_col;
            L_edge_count = L_edge_count + 1;
            turn_left = 0;
        }
        ////////处理爬完的边界
        if(zx_flag[curr_row]!=1)
        {
            zx[curr_row]=curr_col;
            zx_flag[curr_row]=1;
        }
    }
    get_turningleft_point();//找左拐点
    if( (L_corner_flag==1&&mt9v03x_image[L_corner_row][L_corner_col+1]>Threshold)||(Cross_Flag==1&&mt9v03x_image[MT9V03X_H-2][20]>Threshold) )//找种子
    {
        if(L_corner_flag==1)
        {
            for(int i=L_corner_row;i>0;i--)
            {
                if(mt9v03x_image[i-1][L_corner_col+1]<=Threshold)
                {
                    new_row=i;
                    break;
                }
            }
            new_col=L_corner_col+1;
        }
        else if(Cross_Flag==1)
        {
            for(int i=MT9V03X_H-2;i>0;i--)
            {
                if(mt9v03x_image[i-1][20]<=Threshold)
                {
                    new_row=i;
                    break;
                }
            }
            new_col=20;
        }
        if(new_row>15)//确保种子正确
        {
            new_L_edge_count=L_edge_count;//记录下此时种子起始边界数
            curr_row=new_row;curr_col=new_col;
            L_edge[L_edge_count].row = new_row;
            L_edge[L_edge_count].col = new_col;
            zx[curr_row]=curr_col;
            zx_flag[curr_row]=2;
            dire_left = 0; //初始化上个边界点的来向
            turn_left = 0;//转向次数 防止死循环
            cnt_left = L_edge_count;
            while(L_edge_count < L_search_amount)
            {
//                if(curr_row==new_row&&curr_col==new_col&&cnt_left<L_edge_count) break;
                if(turn_left>=4) break;
                int local_thres = 0;
                for(int dy = -1; dy <= 1; dy++)
                {
                    for(int dx = -1; dx <= 1; dx++)
                    {
                        local_thres += mt9v03x_image[curr_row + dy][curr_col + dx];
                    }
                }
                local_thres /= 9;
                local_thres -= clip_value;
                int front_value = mt9v03x_image[ curr_row + dir_front[dire_left][1] ][ curr_col + dir_front[dire_left][0] ];
                int frontleft_value = mt9v03x_image[ curr_row + dir_frontleft[dire_left][1] ][ curr_col +  dir_frontleft[dire_left][0] ];
                //搜线过程
                if(front_value<local_thres)   //上黑
                {
                    dire_left = (dire_left+1)%4;
                    turn_left++;
                }
                else if(frontleft_value<local_thres)    //左上黑
                {
                    curr_row = curr_row + dir_front[dire_left][1];
                    curr_col = curr_col + dir_front[dire_left][0];
                    ////越界退出 行越界和列越界（向上向下向左向右）
                    if(curr_row <1||curr_row>MT9V03X_H-2||curr_col<1||curr_col>MT9V03X_W-2||curr_row <= Finnalline+1)  break;
                    L_edge[L_edge_count].row = curr_row;
                    L_edge[L_edge_count].col = curr_col;
                    L_edge_count = L_edge_count + 1;
                    turn_left = 0;
                }
                else                  //上白 左上白
                {
                    curr_row = curr_row + dir_frontleft[dire_left][1];
                    curr_col = curr_col + dir_frontleft[dire_left][0];
                    dire_left = (dire_left + 3) % 4;
                    ////越界退出 行越界和列越界（向上向下向左向右）
                    if(curr_row <1||curr_row>MT9V03X_H-2||curr_col<1||curr_col>MT9V03X_W-2||curr_row <= Finnalline+1)  break;
                    L_edge[L_edge_count].row = curr_row;
                    L_edge[L_edge_count].col = curr_col;
                    L_edge_count = L_edge_count + 1;
                    turn_left = 0;
                }
                ////////处理爬完的边界
                if(zx_flag[curr_row]!=2)
                {
                    zx[curr_row]=curr_col;
                    zx_flag[curr_row]=2;
                }
            }
        }
    }
     R_edge[0].row = R_start_y;
     R_edge[0].col = R_start_x;
     R_edge_count+=1;
     curr_row = R_start_y;
     curr_col = R_start_x;
     yx[curr_row]=curr_col;
     yx_flag[curr_row]=1;
     dire_right = 0;
     turn_right = 0;
     while(R_edge_count < R_search_amount)
     {
         if(turn_right>=4) break;
         int local_thres = 0;
         for(int dy = -1; dy <= 1; dy++)
         {
             for(int dx = -1; dx <= 1; dx++)
             {
                 local_thres += mt9v03x_image[curr_row + dy][curr_col + dx];
             }
         }
         local_thres /= 9;
         local_thres -= clip_value;
         int front_value = mt9v03x_image[ curr_row + dir_front[dire_right][1] ][ curr_col + dir_front[dire_right][0] ];
         int frontright_value = mt9v03x_image[ curr_row + dir_frontright[dire_right][1] ][ curr_col +  dir_frontright[dire_right][0] ];
         //爬线过程
         if(front_value<local_thres)   //上黑
         {
             dire_right = (dire_right+3)%4;
             turn_right++;
         }
         else if(frontright_value<local_thres)    //右上黑
         {
             curr_row = curr_row + dir_front[dire_right][1];
             curr_col = curr_col + dir_front[dire_right][0];
             ////越界退出 行越界和列越界（向上向下向左向右）
             if(curr_row <1||curr_row>MT9V03X_H-2||curr_col<1||curr_col>MT9V03X_W-2||curr_row <= Finnalline+1)  break;
             R_edge[R_edge_count].row = curr_row;
             R_edge[R_edge_count].col = curr_col;
             R_edge_count = R_edge_count + 1;
             turn_right = 0;
         }
         else                  //上白 右上白
         {
             curr_row = curr_row + dir_frontright[dire_right][1];
             curr_col = curr_col + dir_frontright[dire_right][0];
             dire_right = (dire_right + 1) % 4;
             ////越界退出 行越界和列越界（向上向下向左向右）
             if(curr_row <1||curr_row>MT9V03X_H-2||curr_col<1||curr_col>MT9V03X_W-2||curr_row <= Finnalline+1)  break;
             R_edge[R_edge_count].row = curr_row;
             R_edge[R_edge_count].col = curr_col;
             R_edge_count = R_edge_count + 1;
             turn_right = 0;
         }
         if(yx_flag[curr_row]!=1)
         {
             yx[curr_row]=curr_col;
             yx_flag[curr_row]=1;
         }
     }
         get_turningright_point();//找右拐点
         if((R_corner_flag==1&&mt9v03x_image[R_corner_row][R_corner_col-1]>Threshold) || (Cross_Flag==1&&mt9v03x_image[MT9V03X_H-2][MT9V03X_W-20]>Threshold))
         {
             if(R_corner_flag==1)
             {
                 for(int i=R_corner_row;i>0;i--)
                 {
                     if(mt9v03x_image[i-1][R_corner_col-1]<=Threshold)
                     {
                         new_row=i;
                         break;
                     }
                 }
                 new_col=R_corner_col-1;
             }
             else if(Cross_Flag==1)
             {
                 for(int i=MT9V03X_H-2;i>0;i--)
                 {
                     if(mt9v03x_image[i-1][MT9V03X_W-20]<=Threshold)
                     {
                         new_row=i;
                         break;
                     }
                 }
                 new_col=MT9V03X_W-20;
             }
             if(new_row>15)//确保种子正确
             {
                 new_R_edge_count=R_edge_count;//记录下此时种子起始边界数
                 curr_row=new_row;curr_col=new_col;
                 R_edge[R_edge_count].row = new_row;
                 R_edge[R_edge_count].col = new_col;
                 yx[curr_row]=curr_col;
                 yx_flag[curr_row]=2;
                 dire_right = 0; //初始化上个边界点的来向
                 turn_right = 0;//转向次数 防止死循环
                 cnt_right = R_edge_count;
                 while(R_edge_count < R_search_amount)
                 {
                     ////越界退出 行越界和列越界（向上向下向左向右）
//                     if(curr_row==new_row&&curr_col==new_col&&cnt_right<R_edge_count) break;
                     if(turn_right>=4) break;
                     int local_thres = 0;
                     for(int dy = -1; dy <= 1; dy++)
                     {
                         for(int dx = -1; dx <= 1; dx++)
                         {
                             local_thres += mt9v03x_image[curr_row + dy][curr_col + dx];
                         }
                     }
                     local_thres /= 9;
                     local_thres -= clip_value;
                     int front_value = mt9v03x_image[ curr_row + dir_front[dire_right][1] ][ curr_col + dir_front[dire_right][0] ];
                     int frontright_value = mt9v03x_image[ curr_row + dir_frontright[dire_right][1] ][ curr_col +  dir_frontright[dire_right][0] ];
                     //爬线过程
                     if(front_value<local_thres)   //上黑
                     {
                         dire_right = (dire_right+3)%4;
                         turn_right++;
                     }
                     else if(frontright_value<local_thres)    //右上黑
                     {
                         curr_row = curr_row + dir_front[dire_right][1];
                         curr_col = curr_col + dir_front[dire_right][0];
                         if(curr_row <1||curr_row>MT9V03X_H-2||curr_col<1||curr_col>MT9V03X_W-2||curr_row <= Finnalline+1)  break;
                         R_edge[R_edge_count].row = curr_row;
                         R_edge[R_edge_count].col = curr_col;
                         R_edge_count = R_edge_count + 1;
                         turn_right = 0;
                     }
                     else                  //上白 右上白
                     {
                         curr_row = curr_row + dir_frontright[dire_right][1];
                         curr_col = curr_col + dir_frontright[dire_right][0];
                         dire_right = (dire_right + 1) % 4;
                         if(curr_row <1||curr_row>MT9V03X_H-2||curr_col<1||curr_col>MT9V03X_W-2||curr_row <= Finnalline+1)  break;
                         R_edge[R_edge_count].row = curr_row;
                         R_edge[R_edge_count].col = curr_col;
                         R_edge_count = R_edge_count + 1;
                         turn_right = 0;
                     }
                     if(yx_flag[curr_row]!=2)
                     {
                         yx[curr_row]=curr_col;
                         yx_flag[curr_row]=2;
                     }
                 }
             }
         }
//         for(int i = 0;i<countnum;i++)
//         {
//             L_edge_row[i]=L_edge[i].row;
//             L_edge_col[i]=L_edge[i].col;
//
//             R_edge_row[i]=R_edge[i].row;
//             R_edge_col[i]=R_edge[i].col;
//
//         }看图像
}

/*---------------------------------------------------------------
 【函    数】search_neighborhood 二值化阈值版
 【功    能】迷宫法找边界
 【参    数】无
 【返 回 值】无
 【注意事项】
 ----------------------------------------------------------------*/
void search_neighborhood_zeroone(void)
{
    L_edge_count = 0;//左边点个数清0
    R_edge_count = 0;//右边点个数清0
    new_L_edge_count=0;
    new_R_edge_count=0;
    uint8 zx_flag[MT9V03X_H]={0};
    uint8 yx_flag[MT9V03X_H]={0};
    //变量声明
    uint8 new_row=0;
    uint8 new_col=0;
    uint8 cnt_left;
    uint8 cnt_right;
    L_edge[0].row = L_start_y;
    L_edge[0].col = L_start_x;
    L_edge_count+=1;
    int16 curr_row = L_start_y;//初始化行坐标
    int16 curr_col = L_start_x;//初始化列坐标
    zx[curr_row]=curr_col;
    zx_flag[curr_row]=1;
    dire_left = 0; //初始化上个边界点的来向
    turn_left = 0;//转向次数 防止死循环
    //开始搜线，最多取150个点，不会往下搜，共7个方位
    while(L_edge_count < L_search_amount)    //最多搜索150个点
    {
        if(turn_left>=4) break;
        int front_value = image[ curr_row + dir_front[dire_left][1] ][ curr_col + dir_front[dire_left][0] ];
        int frontleft_value = image[ curr_row + dir_frontleft[dire_left][1] ][ curr_col +  dir_frontleft[dire_left][0] ];
        //搜线过程
        if(front_value==0)   //上黑
        {
            dire_left = (dire_left+1)%4;
            turn_left++;
        }
        else if(frontleft_value==0)    //左上黑
        {
            curr_row = curr_row + dir_front[dire_left][1];
            curr_col = curr_col + dir_front[dire_left][0];
            ////越界退出 行越界和列越界（向上向下向左向右）
            if(curr_row <1||curr_row>MT9V03X_H-2||curr_col<1||curr_col>MT9V03X_W-2||curr_row <= Finnalline+1)  break;
            L_edge[L_edge_count].row = curr_row;
            L_edge[L_edge_count].col = curr_col;
            L_edge_count = L_edge_count + 1;
            turn_left = 0;
        }
        else                  //上白 左上白
        {
            curr_row = curr_row + dir_frontleft[dire_left][1];
            curr_col = curr_col + dir_frontleft[dire_left][0];
            dire_left = (dire_left + 3) % 4;
            ////越界退出 行越界和列越界（向上向下向左向右）
            if(curr_row <1||curr_row>MT9V03X_H-2||curr_col<1||curr_col>MT9V03X_W-2||curr_row <= Finnalline+1)  break;
            L_edge[L_edge_count].row = curr_row;
            L_edge[L_edge_count].col = curr_col;
            L_edge_count = L_edge_count + 1;
            turn_left = 0;
        }
        ////////处理爬完的边界
        if(zx_flag[curr_row]!=1)
        {
            zx[curr_row]=curr_col;
            zx_flag[curr_row]=1;
        }
    }
    get_turningleft_point();//找左拐点
    if( (L_corner_flag==1&&image[L_corner_row][L_corner_col+1]==255)||(Cross_Flag==1&&image[MT9V03X_H-2][20]==255) )//找种子
    {
        if(L_corner_flag==1)
        {
            for(int i=L_corner_row;i>0;i--)
            {
                if(image[i-1][L_corner_col+5]==0)
                {
                    new_row=i;
                    new_col=L_corner_col+5;
                    break;
                }
            }
//            if(new_row<=40)
//            {
//                for(int i=L_corner_row;i>0;i--)
//                {
//                    if(image[i-1][L_corner_col+15]==0)
//                    {
//                        new_row=i;
//                        new_col=L_corner_col+15;
//                        break;
//                    }
//                }
//            }
        }
        else if(Cross_Flag==1)
        {
            for(int i=MT9V03X_H-5;i>0;i--)
            {
                if(image[i-1][40]==0)
                {
                    new_row=i;
                    new_col=40;
                    break;
                }
            }
//            if(new_row<=40)
//            {
//                for(int i=MT9V03X_H-5;i>0;i--)
//                {
//                    if(image[i-1][60]==0)
//                    {
//                        new_row=i;
//                        new_col=60;
//                        break;
//                    }
//                }
//            }
        }
        if(new_row>15)//确保种子正确
        {
            new_L_edge_count=L_edge_count;//记录下此时种子起始边界数
            curr_row=new_row;curr_col=new_col;
            L_edge[L_edge_count].row = new_row;
            L_edge[L_edge_count].col = new_col;
            zx[curr_row]=curr_col;
            zx_flag[curr_row]=2;
            dire_left = 0; //初始化上个边界点的来向
            turn_left = 0;//转向次数 防止死循环
            cnt_left = L_edge_count;
            while(L_edge_count < L_search_amount)
            {
//                if(curr_row==new_row&&curr_col==new_col&&cnt_left<L_edge_count) break;
                if(turn_left>=4) break;
                int front_value = image[ curr_row + dir_front[dire_left][1] ][ curr_col + dir_front[dire_left][0] ];
                int frontleft_value = image[ curr_row + dir_frontleft[dire_left][1] ][ curr_col +  dir_frontleft[dire_left][0] ];
                //搜线过程
                if(front_value==0)   //上黑
                {
                    dire_left = (dire_left+1)%4;
                    turn_left++;
                }
                else if(frontleft_value==0)    //左上黑
                {
                    curr_row = curr_row + dir_front[dire_left][1];
                    curr_col = curr_col + dir_front[dire_left][0];
                    ////越界退出 行越界和列越界（向上向下向左向右）
                    if(curr_row <1||curr_row>MT9V03X_H-2||curr_col<1||curr_col>MT9V03X_W-2||curr_row <= Finnalline+1)  break;
                    L_edge[L_edge_count].row = curr_row;
                    L_edge[L_edge_count].col = curr_col;
                    L_edge_count = L_edge_count + 1;
                    turn_left = 0;
                }
                else                  //上白 左上白
                {
                    curr_row = curr_row + dir_frontleft[dire_left][1];
                    curr_col = curr_col + dir_frontleft[dire_left][0];
                    dire_left = (dire_left + 3) % 4;
                    ////越界退出 行越界和列越界（向上向下向左向右）
                    if(curr_row <1||curr_row>MT9V03X_H-2||curr_col<1||curr_col>MT9V03X_W-2||curr_row <= Finnalline+1)  break;
                    L_edge[L_edge_count].row = curr_row;
                    L_edge[L_edge_count].col = curr_col;
                    L_edge_count = L_edge_count + 1;
                    turn_left = 0;
                }
                ////////处理爬完的边界
                if(zx_flag[curr_row]!=2)
                {
                    zx[curr_row]=curr_col;
                    zx_flag[curr_row]=2;
                }
            }
        }
    }
     R_edge[0].row = R_start_y;
     R_edge[0].col = R_start_x;
     R_edge_count+=1;
     curr_row = R_start_y;
     curr_col = R_start_x;
     yx[curr_row]=curr_col;
     yx_flag[curr_row]=1;
     dire_right = 0;
     turn_right = 0;
     while(R_edge_count < R_search_amount)
     {
         if(turn_right>=4) break;
         int front_value = image[ curr_row + dir_front[dire_right][1] ][ curr_col + dir_front[dire_right][0] ];
         int frontright_value = image[ curr_row + dir_frontright[dire_right][1] ][ curr_col +  dir_frontright[dire_right][0] ];
         //爬线过程
         if(front_value==0)   //上黑
         {
             dire_right = (dire_right+3)%4;
             turn_right++;
         }
         else if(frontright_value==0)    //右上黑
         {
             curr_row = curr_row + dir_front[dire_right][1];
             curr_col = curr_col + dir_front[dire_right][0];
             ////越界退出 行越界和列越界（向上向下向左向右）
             if(curr_row <1||curr_row>MT9V03X_H-2||curr_col<1||curr_col>MT9V03X_W-2||curr_row <= Finnalline+1)  break;
             R_edge[R_edge_count].row = curr_row;
             R_edge[R_edge_count].col = curr_col;
             R_edge_count = R_edge_count + 1;
             turn_right = 0;
         }
         else                  //上白 右上白
         {
             curr_row = curr_row + dir_frontright[dire_right][1];
             curr_col = curr_col + dir_frontright[dire_right][0];
             dire_right = (dire_right + 1) % 4;
             ////越界退出 行越界和列越界（向上向下向左向右）
             if(curr_row <1||curr_row>MT9V03X_H-2||curr_col<1||curr_col>MT9V03X_W-2||curr_row <= Finnalline+1)  break;
             R_edge[R_edge_count].row = curr_row;
             R_edge[R_edge_count].col = curr_col;
             R_edge_count = R_edge_count + 1;
             turn_right = 0;
         }
         if(yx_flag[curr_row]!=1)
         {
             yx[curr_row]=curr_col;
             yx_flag[curr_row]=1;
         }
     }
         get_turningright_point();//找右拐点
         if((R_corner_flag==1&&image[R_corner_row][R_corner_col-1]==255) || (Cross_Flag==1&&image[MT9V03X_H-2][MT9V03X_W-20]==255))
         {
             if(R_corner_flag==1)
             {
                 for(int i=R_corner_row;i>0;i--)
                 {
                     if(image[i-1][R_corner_col-5]==0)
                     {
                         new_row=i;
                         new_col=R_corner_col-5;
                         break;
                     }
                 }
//                 if(new_row<=40)
//                 {
//                     for(int i=R_corner_row;i>0;i--)
//                     {
//                         if(image[i-1][R_corner_col-15]==0)
//                         {
//                             new_row=i;
//                             new_col=R_corner_col-15;
//                             break;
//                         }
//                     }
//                 }
             }
             else if(Cross_Flag==1)
             {
                 for(int i=MT9V03X_H-5;i>0;i--)
                 {
                     if(image[i-1][MT9V03X_W-40]==0)
                     {
                         new_row=i;
                         new_col=MT9V03X_W-40;
                         break;
                     }
                 }
//                 if(new_row<=40)
//                 {
//                     for(int i=MT9V03X_H-5;i>0;i--)
//                     {
//                         if(image[i-1][MT9V03X_W-60]==0)
//                         {
//                             new_row=i;
//                             new_col=MT9V03X_W-60;
//                             break;
//                         }
//                     }
//                 }
             }
             if(new_row>15)//确保种子正确
             {
                 new_R_edge_count=R_edge_count;//记录下此时种子起始边界数
                 curr_row=new_row;curr_col=new_col;
                 R_edge[R_edge_count].row = new_row;
                 R_edge[R_edge_count].col = new_col;
                 yx[curr_row]=curr_col;
                 yx_flag[curr_row]=2;
                 dire_right = 0; //初始化上个边界点的来向
                 turn_right = 0;//转向次数 防止死循环
                 cnt_right = R_edge_count;
                 while(R_edge_count < R_search_amount)
                 {
                     ////越界退出 行越界和列越界（向上向下向左向右）
//                     if(curr_row==new_row&&curr_col==new_col&&cnt_right<R_edge_count) break;
                     if(turn_right>=4) break;
                     int front_value = image[ curr_row + dir_front[dire_right][1] ][ curr_col + dir_front[dire_right][0] ];
                     int frontright_value = image[ curr_row + dir_frontright[dire_right][1] ][ curr_col +  dir_frontright[dire_right][0] ];
                     //爬线过程
                     if(front_value==0)   //上黑
                     {
                         dire_right = (dire_right+3)%4;
                         turn_right++;
                     }
                     else if(frontright_value==0)    //右上黑
                     {
                         curr_row = curr_row + dir_front[dire_right][1];
                         curr_col = curr_col + dir_front[dire_right][0];
                         if(curr_row <1||curr_row>MT9V03X_H-2||curr_col<1||curr_col>MT9V03X_W-2||curr_row <= Finnalline+1)  break;
                         R_edge[R_edge_count].row = curr_row;
                         R_edge[R_edge_count].col = curr_col;
                         R_edge_count = R_edge_count + 1;
                         turn_right = 0;
                     }
                     else                  //上白 右上白
                     {
                         curr_row = curr_row + dir_frontright[dire_right][1];
                         curr_col = curr_col + dir_frontright[dire_right][0];
                         dire_right = (dire_right + 1) % 4;
                         if(curr_row <1||curr_row>MT9V03X_H-2||curr_col<1||curr_col>MT9V03X_W-2||curr_row <= Finnalline+1)  break;
                         R_edge[R_edge_count].row = curr_row;
                         R_edge[R_edge_count].col = curr_col;
                         R_edge_count = R_edge_count + 1;
                         turn_right = 0;
                     }
                     if(yx_flag[curr_row]!=2)
                     {
                         yx[curr_row]=curr_col;
                         yx_flag[curr_row]=2;
                     }
                 }
             }
         }
//         for(int i = 0;i<countnum;i++)
//         {
//             L_edge_row[i]=L_edge[i].row;
//             L_edge_col[i]=L_edge[i].col;
//
//             R_edge_row[i]=R_edge[i].row;
//             R_edge_col[i]=R_edge[i].col;
//
//         }看图像
}

/*---------------------------------------------------------------
 【函    数】clear_find_point
 【功    能】八邻域边界初始化
 【参    数】无
 【返 回 值】
 【注意事项】
 ----------------------------------------------------------------*/
void clear_find_point(void)
{
    for(int i = 0;i<countnum;i++)
    {
        L_edge[i].row = 0;
        L_edge[i].col = 0;
        R_edge[i].row = 0;
        R_edge[i].col = MT9V03X_W-1;
    }
}


int16 L_corner_flag = 0;//左拐点存在标志
int16 L_corner_row = 0;//左拐点所在行
int16 L_corner_col = 0;//左拐点所在列
float L_corner_angle = 0;//左拐点角度
int16 R_corner_flag = 0;//右拐点存在标志
int16 R_corner_row = 0;//右拐点所在行
int16 R_corner_col = 0;//右拐点所在列
float R_corner_angle = 0;//右拐点角度
uint8 enable_L_corner=1,enable_R_corner=1;
//圆环拐点标志位
int16 L_circle_corner_flag = 0;//左拐点存在标志
int16 L_circle_corner_row = 0;//左拐点所在行
int16 L_circle_corner_col = 0;//左拐点所在列
int16 R_circle_corner_flag = 0;//右拐点存在标志
int16 R_circle_corner_row = 0;//右拐点所在行
int16 R_circle_corner_col = 0;//右拐点所在列
/*---------------------------------------------------------------
 【函    数】get_turningleft_point
 【功    能】左下拐点检测
 【参    数】无
 【返 回 值】
 【注意事项】
 ----------------------------------------------------------------*/
void get_turningleft_point(void)
{
    L_corner_flag = 0;// 初始化变量
    L_corner_row = 0;
    L_corner_col = 0;
    L_corner_angle = 0;
    L_circle_corner_flag = 0;
    L_circle_corner_row = 0;
    L_circle_corner_col = 0;
    if(enable_L_corner) //如果使能搜索左拐点
    {
        if(L_edge_count > 9)
        {
            for(int i = 0; i<L_edge_count-9;i++)
            {
                if(L_edge[i+8].row>5)
                {
                    if((L_edge[i].col - L_edge[i + 4].col) * (L_edge[i + 8].col - L_edge[i + 4].col) +
                       (L_edge[i].row - L_edge[i + 4].row) * (L_edge[i + 8].row - L_edge[i + 4].row) >= 0) //初步确认为锐角或者直角 向量法
                    {
                        if(L_edge[i+4].col>L_edge[i+8].col)
                        {
                            L_corner_flag = 1;
                            L_corner_row = L_edge[i+4].row;
                            L_corner_col = L_edge[i+4].col;
                            break;
                        }
                    }
                }
            }
        }
    }
}
/*---------------------------------------------------------------
 【函    数】get_turningright_point
 【功    能】右下拐点检测
 【参    数】无
 【返 回 值】
 【注意事项】
 ----------------------------------------------------------------*/
void get_turningright_point(void)
{
    R_corner_flag = 0;//初始化变量
    R_corner_row = 0;
    R_corner_col = 0;
    R_corner_angle = 0;
    R_circle_corner_flag = 0;
    R_circle_corner_row = 0;
    R_circle_corner_col = 0;
    if(enable_R_corner)    //如果使能搜索右拐点
    {
        if(R_edge_count > 9)
        {
            for(int i = 0; i<R_edge_count-9;i++)
            {
                if(R_edge[i+8].row>5)
                {
                    if((R_edge[i].col - R_edge[i + 4].col) * (R_edge[i + 8].col - R_edge[i + 4].col) +
                    (R_edge[i].row - R_edge[i + 4].row) * (R_edge[i + 8].row - R_edge[i + 4].row) >= 0) //初步确认为锐角或者直角 向量法
                    {
                        R_corner_flag = 1;
                        R_corner_row = R_edge[i+4].row;
                        R_corner_col = R_edge[i+4].col;
                        break;
                    }
                }
            }
        }
    }
}

int16 L_upcorner_flag = 0;//左拐点存在标志
int16 L_upcorner_row = 0;//左拐点所在行
int16 L_upcorner_col = 0;//左拐点所在列
int L_upcorner_angle = 0;//左拐点角度
int16 R_upcorner_flag = 0;//右拐点存在标志
int16 R_upcorner_row = 0;//右拐点所在行
int16 R_upcorner_col = 0;//右拐点所在列
int R_upcorner_angle = 0;//右拐点角度
uint8 enable_L_upcorner=1,enable_R_upcorner=1;
int serchendline;
/*---------------------------------------------------------------
 【函    数】get_upturningleft_point
 【功    能】左上拐点检测
 【参    数】无
 【返 回 值】
 【注意事项】
 ----------------------------------------------------------------*/
void get_upturningleft_point(void)
{
    L_upcorner_flag = 0;// 初始化变量
    L_upcorner_row = 0;
    L_upcorner_col = 0;
    L_upcorner_angle = 0;
    if(enable_L_upcorner) //如果使能搜索左拐点
    {
//        if( (L_edge_count-new_L_edge_count) > 9)
//        {
//            for(int i = new_L_edge_count; i<L_edge_count-9;i++)
//            {
//                if(L_edge[i+8].row>5)
//                {
//                    if((L_edge[i].col - L_edge[i + 4].col) * (L_edge[i + 8].col - L_edge[i + 4].col) +
//                       (L_edge[i].row - L_edge[i + 4].row) * (L_edge[i + 8].row - L_edge[i + 4].row) < 0) //初步确认为钝角
//                    {
//                        if(L_edge[i+4].col<L_edge[i+8].col)
//                        {
//                            L_upcorner_flag = 1;
//                            L_upcorner_row = L_edge[i+4].row;
//                            L_upcorner_col = L_edge[i+4].col;
//                            break;
//                        }
//                    }
//                }
//            }
//        }
//        if(L_corner_flag==1)
//        {
//            if(Finnalline>=3)  serchendline = Finnalline;
//            else  serchendline = 3;
//            for(int i=L_corner_row-3;i>serchendline;i--)
//            {
//                if(
//                   abs(zx[i]-zx[i-1])<=5&&//角点的阈值可以更改
//                   abs(zx[i-1]-zx[i-2])<=5&&
//                   abs(zx[i-2]-zx[i-3])<=5&&
//                      (zx[i]-zx[i+1])>=5&&
//                      (zx[i]-zx[i+2])>=5&&
//                      (zx[i]-zx[i+3])>=10&&zx[i]<MT9V03X_W-30&&zx[i]>30&&zx[i]>=L_corner_col+5)
//                {
//                    if(i>40)
//                    {
//                        L_upcorner_flag = 1;
//                        L_upcorner_row = i;
//                        L_upcorner_col = zx[i];
//                        break;
//                    }
//                }
//            }
//        }
//        else
//        {
            for(int i=MT9V03X_H-1-3;i>Finnalline;i--)
            {
                if(
                   abs(zx[i]-zx[i-1])<=5&&//角点的阈值可以更改
                   abs(zx[i-1]-zx[i-2])<=5&&
                   abs(zx[i-2]-zx[i-3])<=5&&
                      (zx[i]-zx[i+1])>=5&&
                      (zx[i]-zx[i+2])>=8&&
                      (zx[i]-zx[i+3])>=10&&zx[i]<MT9V03X_W-30&&zx[i]>30)
                {
                    if(i>30)
                    {
                        L_upcorner_flag = 1;
                        L_upcorner_row = i;
                        L_upcorner_col = zx[i];
                        break;
                    }
                }
            }
//        }
    }
}

/*---------------------------------------------------------------
 【函    数】get_upturningright_point
 【功    能】右上拐点检测
 【参    数】无
 【返 回 值】
 【注意事项】
 ----------------------------------------------------------------*/
void get_upturningright_point(void)
{
    R_upcorner_flag = 0;//初始化变量
    R_upcorner_row = 0;
    R_upcorner_col = 0;
    R_upcorner_angle = 0;
    if(enable_R_corner)    //如果使能搜索右拐点
    {
//        if(R_edge_count > 9)
//        {
//            for(int i = 0; i<R_edge_count-9;i++)
//            {
//                if(R_edge[i+8].row>5)
//                {
//                    if((R_edge[i].col - R_edge[i + 4].col) * (R_edge[i + 8].col - R_edge[i + 4].col) +
//                    (R_edge[i].row - R_edge[i + 4].row) * (R_edge[i + 8].row - R_edge[i + 4].row) >= 0) //初步确认为锐角或者直角 向量法
//                    {
//                        if(R_edge[i+8].col>R_edge[i+4].col&&i<50)
//                        {
//                            R_corner_flag = 1;
//                            R_corner_row = R_edge[i+4].row;
//                            R_corner_col = R_edge[i+4].col;
//                            break;
//                        }
//                    }
//                }
//            }
//            if(R_corner_flag==1)
//            {
//                if(Finnalline>=3)  serchendline = Finnalline;
//                else  serchendline = 3;
//                for(int i=R_corner_row-3;i>serchendline;i--)
//                {
//                    if(
//                       abs(yx[i-1]-yx[i])<=5&&//角点的阈值可以更改
//                       abs(yx[i-2]-yx[i-1])<=5&&
//                       abs(yx[i-3]-yx[i-2])<=5&&
//                          (yx[i+1]-yx[i])>=5&&
//                          (yx[i+2]-yx[i])>=5&&
//                          (yx[i+3]-yx[i])>=10&&yx[i]>30&&yx[i]<MT9V03X_W-30&&yx[i]<=R_corner_col-5)
//                    {
//                        if(i>40)
//                        {
//                            R_upcorner_flag = 1;
//                            R_upcorner_row = i;
//                            R_upcorner_col = yx[i];
//                            break;
//                        }
//                    }
//                }
//            }
//            else
//            {
                for(int i=MT9V03X_H-1-3;i>Finnalline;i--)
                {
                    if(
                       abs(yx[i-1]-yx[i])<=5&&//角点的阈值可以更改
                       abs(yx[i-2]-yx[i-1])<=5&&
                       abs(yx[i-3]-yx[i-2])<=5&&
                          (yx[i+1]-yx[i])>=5&&
                          (yx[i+2]-yx[i])>=8&&
                          (yx[i+3]-yx[i])>=10&&yx[i]>30&&yx[i]<MT9V03X_W-30)
                    {
                        if(i>30)
                        {
                            R_upcorner_flag = 1;
                            R_upcorner_row = i;
                            R_upcorner_col = yx[i];
                            break;
                        }
                    }
                }
//            }
        }
}



int16 Lin_circleup_flag = 0;//左拐点存在标志
int16 Lin_circleup_row = 0;//左拐点所在行
int16 Lin_circleup_col = 0;//左拐点所在列
/*-------------------------------------------------------------------------------------------------------------------
  @brief     左圆环入环左上拐点
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_left_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
void Circle_in_Left()//单调性改变，返回值是单调性改变点所在的行数
{
    Lin_circleup_flag = 0;// 初始化变量
    Lin_circleup_row = 0;
    Lin_circleup_col = 0;
    if(L_edge_count > 9)
    {
        for(int i = 0; i<L_edge_count-9;i++)
        {
            if(L_edge[i].col<=L_edge[i+1].col&&L_edge[i+1].col<=L_edge[i+2].col&&
                    L_edge[i+2].col<=L_edge[i+3].col&&L_edge[i+3].col<=L_edge[i+4].col&&
                    L_edge[i+4].col<=L_edge[i+5].col&&L_edge[i+5].col<=L_edge[i+6].col&&
                    L_edge[i+6].col<=L_edge[i+7].col&&L_edge[i+7].col<=L_edge[i+8].col)
            {
                if(L_edge[i].row<=L_edge[i+1].row&&L_edge[i+1].row<=L_edge[i+2].row&&
                        L_edge[i+2].row<=L_edge[i+3].row&&L_edge[i+3].row<=L_edge[i+4].row&&
                        L_edge[i+4].row>L_edge[i+5].row&&L_edge[i+5].row>=L_edge[i+6].row&&
                        L_edge[i+6].row>=L_edge[i+7].row&&L_edge[i+7].row>=L_edge[i+8].row)//注意中间两个点是大于而不是大于等于
                {
                    if(L_edge[i+4].row>40)
                    {
                        Lin_circleup_flag=1;
                        Lin_circleup_row=L_edge[i+4].row;
                        Lin_circleup_col=L_edge[i+4].col;
                        break;
                    }

                }
            }
        }
    }
}



int16 Lout_circleup_flag = 0;//左拐点存在标志
int16 Lout_circleup_row = 0;//左拐点所在行
int16 Lout_circleup_col = 0;//左拐点所在列
/*-------------------------------------------------------------------------------------------------------------------
  @brief     左圆环出环右上拐点
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_left_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
void Circle_out_Left()//单调性改变，返回值是单调性改变点所在的行数
{
    Lout_circleup_flag = 0;// 初始化变量
    Lout_circleup_row = 0;
    Lout_circleup_col = 0;
    if(R_edge_count > 9)
    {
        for(int i = 0; i<R_edge_count-9;i++)
        {
            if(R_edge[i].row>=R_edge[i+1].row&&R_edge[i+1].row>=R_edge[i+2].row&&
                    R_edge[i+2].row>=R_edge[i+3].row&&R_edge[i+3].row>=R_edge[i+4].row&&
                    R_edge[i+4].row>=R_edge[i+5].row&&R_edge[i+5].row>=R_edge[i+6].row&&
                    R_edge[i+6].row>=R_edge[i+7].row&&R_edge[i+7].row>=R_edge[i+8].row)
            {
                if(R_edge[i].col>=R_edge[i+1].col&&R_edge[i+1].col>=R_edge[i+2].col&&
                        R_edge[i+2].col>=R_edge[i+3].col&&R_edge[i+3].col>=R_edge[i+4].col&&
                        R_edge[i+4].col<R_edge[i+5].col&&R_edge[i+5].col<=R_edge[i+6].col&&
                        R_edge[i+6].col<=R_edge[i+7].col&&R_edge[i+7].col<=R_edge[i+8].col)//注意中间两个点是大于而不是大于等于
                {
                    if(1)
                    {
                        Lout_circleup_flag=1;
                        Lout_circleup_row=R_edge[i+4].row;
                        Lout_circleup_col=R_edge[i+4].col;
                        break;
                    }
                }
            }
        }
    }
}

int16 Rin_circleup_flag = 0;//左拐点存在标志
int16 Rin_circleup_row = 0;//左拐点所在行
int16 Rin_circleup_col = 0;//左拐点所在列
/*-------------------------------------------------------------------------------------------------------------------
  @brief     右圆环入环右上拐点
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_left_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
void Circle_in_Right()//单调性改变，返回值是单调性改变点所在的行数
{
    Rin_circleup_flag = 0;// 初始化变量
    Rin_circleup_row = 0;
    Rin_circleup_col = 0;
    if(R_edge_count > 9)
    {
        for(int i = 0; i<R_edge_count-9;i++)
        {
            if(R_edge[i].col>=R_edge[i+1].col&&R_edge[i+1].col>=R_edge[i+2].col&&
                    R_edge[i+2].col>=R_edge[i+3].col&&R_edge[i+3].col>=R_edge[i+4].col&&
                    R_edge[i+4].col>=R_edge[i+5].col&&R_edge[i+5].col>=R_edge[i+6].col&&
                    R_edge[i+6].col>=R_edge[i+7].col&&R_edge[i+7].col>=R_edge[i+8].col)
            {
                if(R_edge[i].row<=R_edge[i+1].row&&R_edge[i+1].row<=R_edge[i+2].row&&
                        R_edge[i+2].row<=R_edge[i+3].row&&R_edge[i+3].row<=R_edge[i+4].row&&
                        R_edge[i+4].row>R_edge[i+5].row&&R_edge[i+5].row>=R_edge[i+6].row&&
                        R_edge[i+6].row>=R_edge[i+7].row&&R_edge[i+7].row>=R_edge[i+8].row)//注意中间两个点是大于而不是大于等于
                {
                    if(R_edge[i+4].row>40)
                    {
                        Rin_circleup_flag=1;
                        Rin_circleup_row=R_edge[i+4].row;
                        Rin_circleup_col=R_edge[i+4].col;
                        break;
                    }
                }
            }
        }
    }
}
//
int16 Rout_circleup_flag = 0;//左拐点存在标志
int16 Rout_circleup_row = 0;//左拐点所在行
int16 Rout_circleup_col = 0;//左拐点所在列
/*-------------------------------------------------------------------------------------------------------------------
  @brief     左圆环出环右上拐点
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_left_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
void Circle_out_Right()//单调性改变，返回值是单调性改变点所在的行数
{
    Rout_circleup_flag = 0;// 初始化变量
    Rout_circleup_row = 0;
    Rout_circleup_col = 0;
    if(L_edge_count > 9)
    {
        for(int i = 0; i<L_edge_count-9;i++)
        {
            if(L_edge[i].row>=L_edge[i+1].row&&L_edge[i+1].row>=L_edge[i+2].row&&
                    L_edge[i+2].row>=L_edge[i+3].row&&L_edge[i+3].row>=L_edge[i+4].row&&
                    L_edge[i+4].row>=L_edge[i+5].row&&L_edge[i+5].row>=L_edge[i+6].row&&
                    L_edge[i+6].row>=L_edge[i+7].row&&L_edge[i+7].row>=L_edge[i+8].row)
            {
                if(L_edge[i].col<=L_edge[i+1].col&&L_edge[i+1].col<=L_edge[i+2].col&&
                        L_edge[i+2].col<=L_edge[i+3].col&&L_edge[i+3].col<=L_edge[i+4].col&&
                        L_edge[i+4].col>L_edge[i+5].col&&L_edge[i+5].col>=L_edge[i+6].col&&
                        L_edge[i+6].col>=L_edge[i+7].col&&L_edge[i+7].col>=L_edge[i+8].col)//注意中间两个点是大于而不是大于等于
                {
                    if(1)
                    {
                        Rout_circleup_flag=1;
                        Rout_circleup_row=L_edge[i+4].row;
                        Rout_circleup_col=L_edge[i+4].col;
                        break;
                    }
                }
            }
        }
    }
}

uint8 Threshold_boundary = 20;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     找迷宫起始点 局部阈值版
  @note
-------------------------------------------------------------------------------------------------------------------*/
void fineonepoint()
{
    uint8 i,j,upward,to_left,to_right;
    uint8 L_start_flag=2,R_start_flag=2;
    uint8 h=MT9V03X_H-2;//起始点
    uint8 left_findflag = 0;
    uint8 right_findflag = 0;
    Finnalline=5;

    volatile int White_Column[MT9V03X_W]={0};//每列白列长度
    for (i=2;i<=MT9V03X_W-2-1;i+=2)
    {
        for (j=MT9V03X_H-2;j >= 0; j--)
        {
            if(mt9v03x_image[j][i] <= Threshold)
                break;
            else
                White_Column[i]++;
        }
    }
    //从左到右找左边最长白列
     Longest_White_Column_Left[0] =0;
     for(i=2;i<=MT9V03X_W-2-1;i+=2)
     {
         if (Longest_White_Column_Left[0] < White_Column[i])//找最长的那一列
         {
             Longest_White_Column_Left[0] = White_Column[i];//【0】是白列长度
             Longest_White_Column_Left[1] = i;              //【1】是下标，第j列
         }
     }
     for(;h>0;h--)
     {
         if(h>=1&&mt9v03x_image[h-1][ Longest_White_Column_Left[1] ]<=Threshold)
         {
             Finnalline=h;//截至行
             break;
         }
     }
    for(upward=MT9V03X_H-2;upward>Finnalline;upward--)
    {
        //左边界
        for(to_left= Longest_White_Column_Left[1]; to_left>0 ;to_left--)
        {
            if (calc_diff(mt9v03x_image[upward][to_left+5],mt9v03x_image[upward][to_left]) >= Threshold_boundary )
            {
                zx[upward]=to_left+1;
                if(L_start_flag)
                {
                    L_start_flag--;
                    if(!L_start_flag)
                    {
                        L_start_x=to_left+1;
                        L_start_y=upward;
                        left_findflag=1;
                    }
                }
                break;
            }
            else if(to_left<=1)
            {
                zx[upward]=1;
                break;
            }
        }

       //右边界
        for(to_right= Longest_White_Column_Left[1]; to_right<MT9V03X_W-1;to_right++)
        {
            if(calc_diff(mt9v03x_image[upward][to_right-5],mt9v03x_image[upward][to_right])>=Threshold_boundary )
            {
                yx[upward]=to_right-1;
                if(R_start_flag)
                {
                    R_start_flag--;
                    if(!R_start_flag)
                    {
                        R_start_x=to_right-1;
                        R_start_y=upward;
                        right_findflag=1;
                    }
                }
                break;
            }
            else if(to_right>=MT9V03X_W-2)
            {
                yx[upward]=MT9V03X_W-2;
                break;
            }
        }
        if(left_findflag==1&&right_findflag==1) break;
    }
}

uint8 Midline_white = 0;
uint8 Leftline_white = 0;
uint8 Rightline_white = 0;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     找迷宫起始点 二值化版
  @note
-------------------------------------------------------------------------------------------------------------------*/
void fineonepoint_zeroone()
{
    uint8 i,j,upward,to_left,to_right;
    uint8 L_start_flag=2,R_start_flag=2;
    uint8 h=MT9V03X_H-2;//起始点
    uint8 left_findflag = 0;
    uint8 right_findflag = 0;
    Finnalline=5;

    volatile int White_Column[MT9V03X_W]={0};//每列白列长度
    for (i=2;i<=MT9V03X_W-2-1;i+=2)
    {
        for (j=MT9V03X_H-2;j >= 0; j--)
        {
            if(image[j][i] == 0)
                break;
            else
                White_Column[i]++;
        }
    }
    //从左到右找左边最长白列
     Longest_White_Column_Left[0] =0;
     Longest_White_Column_Right[0] = 0;
     if(left_circle_flag==1&&(circle_mode==2||circle_mode==3))
     {
         for(i=10;i<=MT9V03X_W/2;i+=2)
         {
             if (Longest_White_Column_Left[0] < White_Column[i])//找最长的那一列
             {
                 Longest_White_Column_Left[0] = White_Column[i];//【0】是白列长度
                 Longest_White_Column_Left[1] = i;              //【1】是下标，第j列
             }
         }
         for(;h>0;h--)
         {
             if(h>=1&&image[h-1][ Longest_White_Column_Left[1] ]==0)
             {
                 Finnalline=h;//截至行
                 break;
             }
         }
     }
     else if(right_circle_flag==1&&(circle_mode==2||circle_mode==3))
     {
         for(i=MT9V03X_W-10;i>=MT9V03X_W/2;i-=2)
         {
             if (Longest_White_Column_Right[0] < White_Column[i])//找最长的那一列
             {
                 Longest_White_Column_Right[0] = White_Column[i];//【0】是白列长度
                 Longest_White_Column_Right[1] = i;              //【1】是下标，第j列
             }
         }
         for(;h>0;h--)
         {
             if(h>=1&&image[h-1][ Longest_White_Column_Right[1] ]==0)
             {
                 Finnalline=h;//截至行
                 break;
             }
         }
     }
     else
     {
         for(i=10;i<=MT9V03X_W-10;i+=2)
         {
             if (Longest_White_Column_Left[0] < White_Column[i])//找最长的那一列
             {
                 Longest_White_Column_Left[0] = White_Column[i];//【0】是白列长度
                 Longest_White_Column_Left[1] = i;              //【1】是下标，第j列
             }
         }
         for(i=MT9V03X_W-10;i>=10;i-=2)
         {
             if (Longest_White_Column_Right[0] < White_Column[i])//找最长的那一列
             {
                 Longest_White_Column_Right[0] = White_Column[i];//【0】是白列长度
                 Longest_White_Column_Right[1] = i;              //【1】是下标，第j列
             }
         }
         for(;h>0;h--)
         {
             if(h>=1&&image[h-1][ Longest_White_Column_Left[1] ]==0)
             {
                 Finnalline=h;//截至行
                 break;
             }
         }
     }

     Midline_white = White_Column[MT9V03X_W/2];//中间行白列数
     Leftline_white = Longest_White_Column_Left[0];//中间行白列数
     Rightline_white = Longest_White_Column_Right[0];//中间行白列数

    for(upward=MT9V03X_H-2;upward>Finnalline;upward--)
    {
        //左边界
        for(to_left= Longest_White_Column_Left[1]; to_left>0 ;to_left--)
        {
            if (image[upward][to_left-1]==0&&image[upward][to_left]==0&&image[upward][to_left+1]==255 )
            {
                zx[upward]=to_left+1;
                if(L_start_flag)
                {
                    L_start_flag--;
                    if(!L_start_flag)
                    {
                        L_start_x=to_left+1;
                        L_start_y=upward;
                        left_findflag=1;
                    }
                }
                break;
            }
            else if(to_left<=1)
            {
                zx[upward]=1;
                break;
            }
        }

       //右边界
        for(to_right= Longest_White_Column_Left[1]; to_right<MT9V03X_W-1;to_right++)
        {
            if(image[upward][to_right+1]==0&&image[upward][to_right]==0&&image[upward][to_right-1]==255 )
            {
                yx[upward]=to_right-1;
                if(R_start_flag)
                {
                    R_start_flag--;
                    if(!R_start_flag)
                    {
                        R_start_x=to_right-1;
                        R_start_y=upward;
                        right_findflag=1;
                    }
                }
                break;
            }
            else if(to_right>=MT9V03X_W-2)
            {
                yx[upward]=MT9V03X_W-2;
                break;
            }
        }
        if(left_findflag==1&&right_findflag==1) break;
    }
}

int16 calc_diff(int16 x, int16 y)
{
    return ( ((x-y)<<7)/(x+y) );
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    切换图像处理模式
  @note
-------------------------------------------------------------------------------------------------------------------*/
void searchmode(int i)
{
    if(i==1)
    {
        Threshold = otsuThreshold(mt9v03x_image,0,MT9V03X_H,0,MT9V03X_W);
        ZeroOne(mt9v03x_image,0,MT9V03X_H,0,MT9V03X_W,Threshold);
        image_draw_rectan();
        fineonepoint();
        clear_find_point();
        search_neighborhood();
    }
    if(i==2)
    {
        Threshold = otsuThreshold(mt9v03x_image,0,MT9V03X_H,0,MT9V03X_W);
        ZeroOne(mt9v03x_image,0,MT9V03X_H,0,MT9V03X_W,Threshold);
        image_draw_rectan_zeroone();
        fineonepoint_zeroone();
        clear_find_point();
        search_neighborhood_zeroone();
    }
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     左赛道行单调性检测
  @param     起始点，终止点
  @return    单调返回0，不单调返回断线出行数
  Sample     Monotonicity_Change_Left()
  @note
-------------------------------------------------------------------------------------------------------------------*/
int Monotonicity_Change_Left()
{
    int monotonicity_change_flag=0;
    if(L_edge_count > 9)
    {
        for(int i = 0; i<L_edge_count-9;i++)
        {
            if(L_edge[i].row>=L_edge[i+1].row&&L_edge[i+1].row>=L_edge[i+2].row&&
                    L_edge[i+2].row>=L_edge[i+3].row&&L_edge[i+3].row>=L_edge[i+4].row&&
                    L_edge[i+4].row<L_edge[i+5].row&&L_edge[i+5].row<=L_edge[i+6].row&&
                    L_edge[i+6].row<=L_edge[i+7].row&&L_edge[i+7].row<=L_edge[i+8].row)//中间单调性改变
            {
                monotonicity_change_flag=L_edge[i+4].row;
                break;
            }
        }
    }
    return monotonicity_change_flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右赛道行单调性检测
  @param     起始点，终止点
  @return    单调返回0，不单调返回断线出行数
  Sample     Monotonicity_Change_Right()
  @note
-------------------------------------------------------------------------------------------------------------------*/
int Monotonicity_Change_Right()
{
    int monotonicity_change_flag=0;
    if(R_edge_count > 9)
    {
        for(int i = 0; i<R_edge_count-9;i++)
        {
            if(R_edge[i].row>=R_edge[i+1].row&&R_edge[i+1].row>=R_edge[i+2].row&&
                    R_edge[i+2].row>=R_edge[i+3].row&&R_edge[i+3].row>=R_edge[i+4].row&&
                    R_edge[i+4].row<R_edge[i+5].row&&R_edge[i+5].row<=R_edge[i+6].row&&
                    R_edge[i+6].row<=R_edge[i+7].row&&R_edge[i+7].row<=R_edge[i+8].row)//中间单调性改变
            {
                monotonicity_change_flag=R_edge[i+4].row;
                break;
            }
        }
    }
    return monotonicity_change_flag;
}

int forwardline=0;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     偏差计算
  @note
-------------------------------------------------------------------------------------------------------------------*/
float Err_Sum1(void)
{
//    forwardline = Qianzhan;//46
    forwardline = 48;//46
    int i;
    float err=0;
    float err_last=0;
    float k = 1.0;
    err_last = Err;
    if(circle_mode==2)
    {
        forwardline = 70;
    }
    if(circle_mode==3)
    {
        forwardline = 70;
    }
    if(circle_mode==4)
    {
        forwardline = 70;
    }
    if(Barrier_Flag==1)
    {
        forwardline = 70;
    }
    if(SingleBridge_mode==2)
    {
        forwardline = 40;
//        k=0.5;
    }



    if(forwardline>=116) forwardline = 116;
    for(i=forwardline+3;i>forwardline;i--)//常规误差计算
    {
        err+= MT9V03X_W/2-wx[i] ;
    }
    err=k*err/3.0;
    if(circle_flag==1)
    {
        if(abs(err-err_last)>80)
        {
            err = err_last;
        }
    }
    else if(SingleBridge_Flag==1)
    {
        if(abs(err-err_last)>80)
        {
            err = err_last;
        }
    }
    else
    {
        if(abs(err-err_last)>40)
        {
            err = err_last;
        }
    }
    return err;
}



int16 L_SingleBridge_flag = 0;//左拐点存在标志
int16 L_SingleBridge_row = 0;//左拐点所在行
int16 L_SingleBridge_col = 0;//左拐点所在列
/*---------------------------------------------------------------
 【函    数】get_SingleBridgeleft_point
 【功    能】左上拐点检测
 【参    数】无
 【返 回 值】
 【注意事项】
 ----------------------------------------------------------------*/
void get_SingleBridgeleft_point(void)
{
    L_SingleBridge_flag = 0;// 初始化变量
    L_SingleBridge_row = 0;
    L_SingleBridge_col = 0;
    if(1) //如果使能搜索左拐点
    {
        for(int i=MT9V03X_H-1-5;i>Finnalline;i--)
        {
            if(
               abs(zx[i]-zx[i-1])<=3&&//角点的阈值可以更改
               abs(zx[i-1]-zx[i-2])<=3&&
               abs(zx[i-2]-zx[i-3])<=3&&
                  (zx[i]-zx[i+2])>=10&&
                  (zx[i]-zx[i+3])>=10&&
                  (zx[i]-zx[i+4])>=10&&zx[i]<MT9V03X_W-30&&zx[i]>30)
            {
                L_SingleBridge_flag = 1;
                L_SingleBridge_row = i;
                L_SingleBridge_col = zx[i];
                break;
            }
        }
    }
}
int16 R_SingleBridge_flag = 0;//右拐点存在标志
int16 R_SingleBridge_row = 0;//右拐点所在行
int16 R_SingleBridge_col = 0;//右拐点所在列
/*---------------------------------------------------------------
 【函    数】get_SingleBridgeright_point
 【功    能】右上拐点检测
 【参    数】无
 【返 回 值】
 【注意事项】
 ----------------------------------------------------------------*/
void get_SingleBridgeright_point(void)
{
    R_SingleBridge_flag = 0;//初始化变量
    R_SingleBridge_row = 0;
    R_SingleBridge_col = 0;
    if(1)    //如果使能搜索右拐点
    {
        for(int i=MT9V03X_H-1-5;i>Finnalline;i--)
        {
            if(
               abs(yx[i-1]-yx[i])<=3&&//角点的阈值可以更改
               abs(yx[i-2]-yx[i-1])<=3&&
               abs(yx[i-3]-yx[i-2])<=3&&
                  (yx[i+2]-yx[i])>=10&&
                  (yx[i+3]-yx[i])>=10&&
                  (yx[i+4]-yx[i])>=10&&yx[i]>30&&yx[i]<MT9V03X_W-30)
            {
                R_SingleBridge_flag = 1;
                R_SingleBridge_row = i;
                R_SingleBridge_col = yx[i];
                break;
            }
        }
    }
}
