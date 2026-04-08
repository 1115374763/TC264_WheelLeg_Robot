#include "dht11.h"

// 切换为输出模式
static void DHT11_Set_Output(void) {
    // 设置为推挽输出模式
    gpio_set_dir(DHT11_PIN, GPO, GPO_PUSH_PULL);
}

// 切换为输入模式
static void DHT11_Set_Input(void) {
    // 设置为上拉输入模式
    gpio_set_dir(DHT11_PIN, GPI, GPI_PULL_UP);
}

// 复位DHT11 (主机发送起始信号)
static void DHT11_Reset(void) {
    DHT11_Set_Output();
    gpio_set_level(DHT11_PIN, 0); // 拉低
    system_delay_ms(20);          // 拉低至少18ms，让DHT11检测到起始信号
    gpio_set_level(DHT11_PIN, 1); // 拉高
    system_delay_us(30);          // 拉高20~40us，准备读取响应
}

// 等待DHT11回应
// 返回 0:检测到响应，1:未检测到响应(超时)
static uint8 DHT11_Check(void) {
    uint8 retry = 0;
    DHT11_Set_Input(); // 主机设为输入

    // 等待 DHT11 拉低 (DHT11响应会有80us的低电平)
    while (gpio_get_level(DHT11_PIN) == 1 && retry < 100) {
        retry++;
        system_delay_us(1);
    }
    if (retry >= 100) return 1;
    retry = 0;

    // 等待 DHT11 拉高 (响应后会有80us的高电平)
    while (gpio_get_level(DHT11_PIN) == 0 && retry < 100) {
        retry++;
        system_delay_us(1);
    }
    if (retry >= 100) return 1;

    return 0; // 成功检测到响应
}

// 从DHT11读取一个位 (Bit)
static uint8 DHT11_Read_Bit(void) {
    uint8 retry = 0;
    
    // 等待变为低电平 (每一位数据开始前都有50us的低电平)
    while (gpio_get_level(DHT11_PIN) == 1 && retry < 100) {
        retry++;
        system_delay_us(1);
    }
    retry = 0;
    // 等待变高电平 (低电平结束，准备判断高电平持续时间)
    while (gpio_get_level(DHT11_PIN) == 0 && retry < 100) {
        retry++;
        system_delay_us(1);
    }
    
    // 延时 40us 用于判断：
    // 如果 40us 后还是高电平，说明是数据 '1' (70us高电平)
    // 如果 40us 后已经是低电平了，说明是数据 '0' (26~28us高电平)
    system_delay_us(40); 
    if (gpio_get_level(DHT11_PIN) == 1) {
        return 1;
    } else {
        return 0;
    }
}

// 从DHT11读取一个字节 (Byte)
static uint8 DHT11_Read_Byte(void) {
    uint8 i, dat = 0;
    for (i = 0; i < 8; i++) {
        dat <<= 1;
        dat |= DHT11_Read_Bit();
    }
    return dat;
}

// 读取一次温湿度数据
uint8 DHT11_Read_Data(uint8 *temp, uint8 *humi) {
    uint8 buf[5];
    uint8 i;
    
    // 1. 发送 20ms 起始信号 
    // 【注意】这里绝对不能关中断，否则20ms不控电机会让车子瞬间倒地！
    DHT11_Reset();
    
    // =========================================================
    // 2. 进入核心读取区：关闭全局中断！
    // 防止 1ms 定时器打断 DHT11 几十微秒的精密时序判断
    // =========================================================
    boolean interrupt_state = disableInterrupts();

    if (DHT11_Check() == 0) { // 检测到响应
        for (i = 0; i < 5; i++) {
            buf[i] = DHT11_Read_Byte(); // 连续读5个字节
        }
        
        // 3. 读取完毕（约耗时 4ms），立刻恢复中断！把控制权还给平衡车
        restoreInterrupts(interrupt_state);
        
        // 校验和验证
        if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]) {
            *humi = buf[0]; // 湿度整数位
            *temp = buf[2]; // 温度整数位
            return 0; // 成功
        }
    } else {
        // 如果没响应，也要记得把中断开回来
        restoreInterrupts(interrupt_state);
    }
    
    return 1; // 失败
}

// 初始化函数
uint8 DHT11_Init(void) {
    // 初始化引脚：输出模式，默认输出高电平，推挽输出
    gpio_init(DHT11_PIN, GPO, 1, GPO_PUSH_PULL); 
    DHT11_Reset();
    return DHT11_Check();
}