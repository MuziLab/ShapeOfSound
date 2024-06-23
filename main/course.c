/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "freertos/event_groups.h"



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include <math.h>
#include "driver/i2s_std.h"

#include "freertos/queue.h"
#include "math.h"
#include "driver/touch_pad.h"
#include <inttypes.h>

#include <complex.h>
#include "nvs.h"
#include "nvs_flash.h"


TaskHandle_t task_1_music_show = NULL;
TaskHandle_t task_1_1_music_get_number = NULL;//任务句柄


#define EXAMPLE_STD_BCLK_IO1 GPIO_NUM_4 // i2s
#define EXAMPLE_STD_WS_IO1 GPIO_NUM_5   
#define EXAMPLE_STD_DOUT_IO1 GPIO_NUM_6
#define EXAMPLE_STD_DIN_IO1 GPIO_NUM_7 


static const char *TAG = "example";
#define ssid_1  "realme GT5"
#define password_1 "88888888"//wifi


#define WIFI_CONNECTED_BIT BIT0//静态ip所需
#define WIFI_FAIL_BIT      BIT1
static EventGroupHandle_t s_wifi_event_group;
#define EXAMPLE_STATIC_IP_ADDR "192.168.157.227"
#define EXAMPLE_STATIC_NETMASK_ADDR "255.255.255.0"
#define EXAMPLE_STATIC_GW_ADDR "192.168.157.202"
#define EXAMPLE_MAIN_DNS_SERVER "8.8.8.8"
#define EXAMPLE_BACKUP_DNS_SERVER "8.8.4.4"

#define WIFI_CONNECT_TIMES 3//wifi重连次数
static int s_retry_num = 0;//static,之后再看

#define GRAVITY (-0.05)
#define INITIAL_VELOCITY 1//重力


#define MIN_LIGHT 15//亮度
uint8_t light_coefficient = 1; // 注意,为保障格式正确,这个数一定要是无符号数
uint8_t mic_sensitive = 0;
uint8_t sleep_coefficient = 10;//这个是阈值,如果振幅都小于这个数,就开始sleep准备
uint8_t sleep_time = 1;
uint8_t gravity_coefficient = 1;
#define SLEEP_TIME_CO 690//这个是一秒

#define EXAMPLE_BUFF_SIZE 1024//缓冲区大小

#define SAMPLING_RANGE 4//fft之后为1024,这个设定了一个灯条取多大范围内的最大值

#define EEPROM_HOST SPI2_HOST

#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 15
#define PIN_NUM_CLK -1
#define PIN_NUM_CS -1

#define LIGHT_NUMBER 256
#define COLUMN_NUMBER 16
#define THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE 16
// 这三个为spi全局变量
uint16_t *send_buffer;
spi_device_handle_t spi;
spi_transaction_t t_1;





static i2s_chan_handle_t tx_chan; // I2S tx channel handler
static i2s_chan_handle_t rx_chan; // I2S rx channel handler


uint8_t green_global[THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE] = {0};
uint8_t blue_global[THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE] = {0};
uint8_t red_global[THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE] = {0};


QueueHandle_t xQueue;

typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} RGB_struct;

typedef struct
{
    float v;
    int a;
    float x;
} Gravity;

Gravity du[COLUMN_NUMBER] = {0};

void read_from_flash_uint8(uint8_t* addr_1,uint8_t* addr_2,uint8_t* addr_3,uint8_t* addr_4,uint8_t* addr_5)
{
    nvs_handle_t nvs_handle_1;
    nvs_open("storage",NVS_READONLY,&nvs_handle_1);
    nvs_get_u8(nvs_handle_1,"number1",addr_1);
    nvs_get_u8(nvs_handle_1,"number2",addr_2);
    nvs_get_u8(nvs_handle_1,"number3",addr_3);
    nvs_get_u8(nvs_handle_1,"number4",addr_4);
    nvs_get_u8(nvs_handle_1,"number5",addr_5);
    nvs_commit(nvs_handle_1);
    nvs_close(nvs_handle_1);

}


void save_to_flash_uint8(uint8_t number_1,uint8_t number_2,uint8_t number_3,uint8_t number_4,uint8_t number_5)
{
//这个函数必须nvs_flash_init();因为连wifi所以不用了
    nvs_handle_t nvs_handle_1;
    nvs_open("storage",NVS_READWRITE,&nvs_handle_1);
    nvs_set_u8(nvs_handle_1,"number1",number_1);
    nvs_set_u8(nvs_handle_1,"number2",number_2);
    nvs_set_u8(nvs_handle_1,"number3",number_3);
    nvs_set_u8(nvs_handle_1,"number4",number_4);
    nvs_set_u8(nvs_handle_1,"number5",number_5);
    nvs_commit(nvs_handle_1);
    nvs_close(nvs_handle_1);
}


static void du_compute(int *qun)
{ // qun为fft计算得到的结果

    for (size_t i = 0; i < COLUMN_NUMBER; i++)
    {

        if (du[i].x < qun[i])
        {
            du[i].x = qun[i];
            du[i].v = INITIAL_VELOCITY;
        }
        else
        {
            du[i].x = du[i].x + du[i].v;
            du[i].v = du[i].v + (GRAVITY*gravity_coefficient);
        }
        if (du[i].x <= 0)
        {
            du[i].x = 0;
            du[i].v = 0;
        }
    }
}

void global_change_1(int line_index)
{
    for (size_t i = 0; i < THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE; i++)
    {
        red_global[i] = (uint8_t)(i * light_coefficient);
        blue_global[i] = (uint8_t)(light_coefficient * i * (1 - ((line_index) / 10)));
        green_global[i] = (uint8_t)(light_coefficient * i * (((line_index) / 10)));
    }
}

void global_change_2(int line_index)
{
    for (size_t i = 0; i < THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE; i++)
    {
        red_global[i] = (uint8_t)(i * light_coefficient);
        blue_global[i] = (uint8_t)( light_coefficient * (THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE - i-1) * (COLUMN_NUMBER-1 - (line_index))/15);
        green_global[i] = (uint8_t)( light_coefficient * (THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE - i-1) * (line_index)/15);
    }
}

void global_change_3(int line_index)
{
    for (size_t i = 0; i < THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE; i++)
    {
        red_global[i] = (uint8_t)(i * light_coefficient);
        blue_global[i] = (uint8_t)(0.1 * light_coefficient * (THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE - i-1) * (COLUMN_NUMBER-1 - (line_index)));
        green_global[i] = (uint8_t)(0.1 * light_coefficient * (THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE - i-1) * (line_index));
    }
}//两种改颜色的方法

static void i2s_example_init_std_duplex(void)
{
    /* Setp 1: Determine the I2S channel configuration and allocate both channels
     * The default configuration can be generated by the helper macro,
     * it only requires the I2S controller id and I2S role */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan));

    /* Step 2: Setting the configurations of standard mode, and initialize rx & tx channels
     * The slot configuration and clock configuration can be generated by the macros
     * These two helper macros is defined in 'i2s_std.h' which can only be used in STD mode.
     * They can help to specify the slot and clock configurations for initialization or re-configuring */
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg =
            {
                .data_bit_width = 24,
                .slot_bit_width = 32,
                .slot_mode = I2S_SLOT_MODE_MONO,
                .slot_mask = I2S_STD_SLOT_LEFT,
                .ws_width = 32,
                .ws_pol = false,
                .bit_shift = true,
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false},
        .slot_cfg.slot_mask = I2S_STD_SLOT_LEFT,
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, // some codecs may require mclk signal, this example doesn't need it
            .bclk = EXAMPLE_STD_BCLK_IO1,
            .ws = EXAMPLE_STD_WS_IO1,
            .dout = EXAMPLE_STD_DOUT_IO1,
            .din = EXAMPLE_STD_DIN_IO1, // In duplex mode, bind output and input to a same gpio can loopback internally
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    /* Initialize the channels */
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
}

static int16_t bit_and(uint8_t byte_1, uint8_t byte_2, uint8_t byte_3)
{

    uint32_t middle = (byte_1 << 24) | (byte_2 << 16) | (byte_3 << 8); // 三个byte为采集到的原始数据,这个用一个32位的高位储存数据,这样可以保证之后的转换正常
    int32_t result = (int32_t)middle;                                  // 原始数据即为补码,这里改回int形,上面不改是为了单纯的存数据,防止补码移位出问题,应该?自己写的忘了,
    result = result >> (16 - mic_sensitive);                           // 右移16位,舍弃了原始数据的低八位,这里可以改变灵敏的,但是太灵敏应该会出现超过数据存储极限长度,导致之后的计算出问题
    int16_t result_1 = (int16_t)result;
    return result_1;
}

static void loud_change(uint8_t* byte_1, uint8_t* byte_2, uint8_t* byte_3)
{

    uint32_t middle = (*byte_1 << 24) | (*byte_2 << 16) | (*byte_3 << 8); // 三个byte为采集到的原始数据,这个用一个32位的高位储存数据,这样可以保证之后的转换正常
    int32_t result = (int32_t)middle;                                  // 原始数据即为补码,这里改回int形,上面不改是为了单纯的存数据,防止补码移位出问题,应该?自己写的忘了,
    result = result >> 8;
    result = result*4;
    *byte_1 = (result>>16)& 0xff;
    *byte_2 = (result>>8)& 0xff;
    *byte_3 = result& 0xff;

}



#define PI 3.14159265358979323846

// 位逆序操作
void bit_reverse_shuffle(complex float *X, int N) {//这是个非常神奇的算法,
    int i, j, k;
    for (i = 1, j = N / 2; i < N - 1; i++) {
        if (i < j) {//这里面的j是来自上一步处理的j,上一个循环处理的j如果满足要求,就是所要的位逆序,神奇
            complex float temp = X[i];
            X[i] = X[j];
            X[j] = temp;
        }
        k = N / 2;
        while (k <= j) {
            j -= k;//j = j-k;0
            k /= 2;
        }
        j += k;
    }
}

// 迭代FFT实现
void iterative_fft(complex float *X, int N) {
    int s, k, j, m;
    complex float w, wm, temp;
    // 首先进行位逆序操作
    bit_reverse_shuffle(X, N);

    for (s = 1; s <= log2(N); s++) {
        m = 1 << s; // m = 2^s
        wm = cexp(-2.0 * PI * I / m); // 计算旋转因子
        for (k = 0; k < N; k += m) {
            w = 1.0;
            for (j = 0; j < m / 2; j++) {
                complex float t = w * X[k + j + m / 2];
                temp = X[k + j];
                X[k + j] = temp + t;
                X[k + j + m / 2] = temp - t;
                w *= wm;
            }
        }
    }
}

void blackman_window(float *window, int N) {
    for (int n = 0; n < N; n++) {
        window[n] = 0.42 - 0.5 * cos(2 * PI * n / (N - 1)) + 0.08 * cos(4 * PI * n / (N - 1));
    }
}

#define N_SAMPLES 1024
int N = N_SAMPLES;
float wind[N_SAMPLES];

static void receive_data_2(void *feiaonf)
{
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    uint8_t *r_buf = calloc(3 * EXAMPLE_BUFF_SIZE, sizeof(uint8_t));
    int16_t result[1024];
    assert(r_buf); // Check if r_buf allocation success
    blackman_window(wind, N);
    size_t r_bytes = 0;
    vTaskDelay(100);
    while (1)
    {

        if (i2s_channel_read(rx_chan, r_buf, 3 * EXAMPLE_BUFF_SIZE * sizeof(uint8_t), &r_bytes, 1000) == ESP_OK)
        {
            
            for (int i = 0; i < 3 * EXAMPLE_BUFF_SIZE; i += 3)
            {
                result[i / 3] = bit_and(r_buf[i + 2], r_buf[i + 1], r_buf[i]);
                loud_change(&r_buf[i+2],&r_buf[i + 1], &r_buf[i]);
            }
            i2s_channel_write(tx_chan, r_buf, 3 * EXAMPLE_BUFF_SIZE * sizeof(uint8_t), &r_bytes, 1000);
            complex float complex_input[N_SAMPLES] = {0};
            for (int i = 0; i < N; i++)
            {
                complex_input[i] = result[i]*wind[i];
            }
            iterative_fft(complex_input,N);
            float y1_cf[N_SAMPLES/2] = {0};
            for (size_t i = 0; i < N/2; i++)
            {   
                int middle = 0;
                y1_cf[i] =10*log10f(cabs(complex_input[i])*cabs(complex_input[i])/(1024*512));
                if (middle<0)
                {
                    middle = 0;
                }
            }

            int *the_end = calloc(COLUMN_NUMBER, sizeof(int));
            int count = SAMPLING_RANGE;
            float max_db = 0;
            for (int i_1 = 0; i_1 < COLUMN_NUMBER; i_1++)
            {   
                for (int i = 0; i < SAMPLING_RANGE; i++)
                {
                    if (y1_cf[i + count] > max_db)
                    {
                        max_db = y1_cf[i + count];
                    }
                }
                the_end[i_1] = max_db;
                count = count + SAMPLING_RANGE;

                max_db = 0;
            }
        
        if (xQueueSend(xQueue, &the_end, portMAX_DELAY) == pdPASS)
        {
            // 成功发送数据到队列
        }
        }

    }

}


static const uint16_t table[16] = {
    0x1111, 0x7111, 0x1711, 0x7711, 0x1171, 0x7171, 0x1771, 0x7771,
    0x1117, 0x7117, 0x1717, 0x7717, 0x1177, 0x7177, 0x1777, 0x7777};


void translate(RGB_struct *rgb_input, u_int16_t *buf)
{
    /*buf is the send buf of spi,rgb_input is the struct array*/
    int n = 0;
    buf[n++] = 0;
    for (size_t i = 0; i < LIGHT_NUMBER; i++)
    {
        buf[n++] = table[0x0f & (rgb_input[i].green >> 4)];
        buf[n++] = table[0x0f & (rgb_input[i].green)];
        buf[n++] = table[0x0f & (rgb_input[i].red >> 4)];
        buf[n++] = table[0x0f & (rgb_input[i].red)];
        buf[n++] = table[0x0f & (rgb_input[i].blue >> 4)];
        buf[n++] = table[0x0f & (rgb_input[i].blue)];
    }
}

spi_device_handle_t spi_initialize(void)
{
    spi_device_handle_t spi;
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,

    };

    spi_device_interface_config_t devcfg = {

        .clock_speed_hz = 3.2 * 1000 * 1000, // Clock out at 10 MHz
        .mode = 0,                           // SPI mode 0
        .spics_io_num = -1,                  // CS pin
        .queue_size = 7,
        .address_bits = 0,
        .command_bits = 0,
        .flags = SPI_DEVICE_TXBIT_LSBFIRST,

    };
    // Initialize the SPI bus
    ret = spi_bus_initialize(EEPROM_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(EEPROM_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));

    return spi;
}

void change_the_form_and_package_line(RGB_struct *target_to_change, int index_of_target, RGB_struct *the_final__result)
{ /*index_of_target starts from 0*/
    /*if copy, index is the max index*/
    /*index就是列数,这个函数输入每列的rgb,,对应列数,和最终发送的存储空间*/

    if (index_of_target % 2 == 0)
    {
        for (size_t i = 0; i < THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE; i++)
        {
            the_final__result[i + index_of_target * THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE] = target_to_change[i];
        }
    }
    else
    {
        for (size_t i = 0; i < THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE; i++)
        {
            int count = THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE - 1;
            the_final__result[(index_of_target + 1) * THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE - i - 1] = target_to_change[i];
        }
    }
}

void change_the_form_and_package_point(RGB_struct target_to_change, int index_of_target, RGB_struct *the_final__result, int the_point)
{
    /*索引都是从0开始的*/
    if (the_point <= THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE) 
    {
        if (index_of_target % 2 == 0)
        {

            the_final__result[the_point + index_of_target * THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE] = target_to_change;
        }
        else
        {

            int count = THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE - 1;
            the_final__result[(index_of_target + 1) * THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE - the_point - 1] = target_to_change;
        }
    }

}

void lighter(RGB_struct *input_date)
{

    spi_transaction_t t_1 = {
        .length = 8 * 12 * LIGHT_NUMBER + 16,
        .tx_buffer = send_buffer,
    };

    translate(input_date, send_buffer);
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t_1));
}
void lighter_2(RGB_struct *input_date)//轮询传输
{

    spi_transaction_t t_1 = {
        .length = 8 * 12 * LIGHT_NUMBER + 16,
        .tx_buffer = send_buffer,
    };

    translate(input_date, send_buffer);
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t_1));
}

void task_3(void)
{ /*熄灯*/
    RGB_struct all_light[LIGHT_NUMBER] = {0};
    lighter_2(all_light);
}


void shape_of_music(void)
{
    RGB_struct input_data_all[LIGHT_NUMBER] = {0}; // 所有灯的要输入的信息
    RGB_struct input_data_a_line[THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE] = {0};
    int *get_number = NULL;
    float difference[COLUMN_NUMBER] = {0};
    RGB_struct du_RGB;

    du_RGB.blue = 0;
    du_RGB.green = 0;
    int sleep_number = 0;
    while (1)
    {

        if (xQueueReceive(xQueue, &get_number, portMAX_DELAY) == pdPASS)
        {
            // 成功从队列接收数据
            // valueToReceive 现在包含了发送的值
        }
        for (size_t i = 0; i < COLUMN_NUMBER; i++)
        {   
            get_number[i] = (get_number[i]) / 2;
            //get_number[i] = log10(1+1000*get_number[i]);
            if (get_number[i] < 0)
            {
                get_number[i] = 0;
            }
            if (get_number[i] > THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE)
            {
                get_number[i] = THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE;
            } // 注意,getnumber是0-15,代表真实的亮灯数,而后面的index是真实数-1
        }
        for (size_t i = 0; i < COLUMN_NUMBER; i++)
        {

            global_change_2(i);//change the global color number,to make model changing easily
            if (get_number[i]>sleep_coefficient)//判断是否大于系数,如果大,就重置number为零,否则++,当number大到一定值时,便熄灯
            {
                sleep_number = 0;
                
            }else
            {
                sleep_number++;
            }

            if (sleep_number>(SLEEP_TIME_CO*sleep_time))//判断number是否大到一定程度
            {
                break;
                
            }
            
            
            for (size_t light_index_each_line = 0; light_index_each_line < THE_NUMBER_OF_LIGHTS_IN_EACH_RANGE; light_index_each_line++)
            {
                if (light_index_each_line < get_number[i]) 
                {
                    input_data_a_line[light_index_each_line].blue = blue_global[light_index_each_line];
                    input_data_a_line[light_index_each_line].green = green_global[light_index_each_line];
                    input_data_a_line[light_index_each_line].red = red_global[light_index_each_line];
                }
                else
                {
                    input_data_a_line[light_index_each_line].blue = 0;
                    input_data_a_line[light_index_each_line].green = 0;
                    input_data_a_line[light_index_each_line].red = 0;
                }
            }
            change_the_form_and_package_line(input_data_a_line, i, input_data_all);

        }
        if (sleep_number>(SLEEP_TIME_CO*sleep_time))
        {
            task_3();
        }else{        
        du_compute(get_number);
        du_RGB.red = (uint8_t) MIN_LIGHT * light_coefficient;
        for (size_t i = 0; i < COLUMN_NUMBER; i++)
        {
            change_the_form_and_package_point(du_RGB, i, input_data_all, du[i].x);
        }
        lighter(input_data_all);
        }
        free(get_number);
    }
}





void task_2(int*x,int*y,int length_of_x)
{
    RGB_struct input_data_all[LIGHT_NUMBER] = {0}; // 所有灯的要输入的信息
    RGB_struct du_RGB;
    du_RGB.red = (uint8_t) MIN_LIGHT * light_coefficient;
    du_RGB.blue = 0;
    du_RGB.green = 0;

    for (size_t i = 0; i < length_of_x; i++)
    {   
        if (y[i]<0||x[i]<0)
        {   
            RGB_struct input_data_all[LIGHT_NUMBER] = {0};
            break;
        }
        //printf("%d||%d***************\n",x[i],y[i]);
        change_the_form_and_package_point(du_RGB, x[i], input_data_all, y[i]);
    }
    

    lighter_2(input_data_all);
    //printf("\n");
}

void task_4(int*x,int*y,int length_of_x,RGB_struct*rgb_input)
{
    RGB_struct input_data_all[LIGHT_NUMBER] = {0}; // 所有灯的要输入的信息
    for (size_t i = 0; i < length_of_x; i++)
    {   
        if (y[i]<0||x[i]<0)
        {   
            RGB_struct input_data_all[LIGHT_NUMBER] = {0};
            break;
        }
        change_the_form_and_package_point(rgb_input[i], x[i], input_data_all, y[i]);
    }
    

    lighter_2(input_data_all);

}


int length_of_tail = 2;//尾巴长度,从1开始
int x_array[512] = {0};
int y_array[512] = {0};
int coordinate_count = 0;
RGB_struct get_rgb ={0};
RGB_struct rgb_store[512] = {0};





void mode_function(int mode_number)
{
    switch (mode_number)
    {
    case 1:
        vTaskResume(task_1_music_show);
        vTaskResume(task_1_1_music_get_number);

        break;
    case 2:
        task_3();
        vTaskSuspend(task_1_music_show);
        vTaskSuspend(task_1_1_music_get_number);
        memset(x_array,0,sizeof(x_array));
        memset(y_array,0,sizeof(y_array));
        coordinate_count = 0;
        if (xQueueReset(xQueue) != pdPASS) 
        {
        }
        
        break;
    case 3:
        task_3();
        vTaskSuspend(task_1_music_show);
        vTaskSuspend(task_1_1_music_get_number);
        if (xQueueReset(xQueue) != pdPASS) 
        {
        }
        break;
    case 4:
        task_3();
        vTaskSuspend(task_1_music_show);
        vTaskSuspend(task_1_1_music_get_number);
        memset(x_array,0,sizeof(x_array));
        memset(y_array,0,sizeof(y_array));
        memset(rgb_store,0,sizeof(rgb_store));
        coordinate_count = 0;
        if (xQueueReset(xQueue) != pdPASS) 
        {
        }
        break; 
    default:
        break;
    }
}



static void recv_from_android(const int sock)
{   
    //用90以后的ascii
    //下面规定通信方法,首先模式切换就是改mode_number,2模式有两个数,2和4.在模式1中,需要通信的点是保存数据,发来的数据有两个类型,A和B,其中A需要保存
    //模式二有两个,2或者是4,其中2模式是尾迹模式,接受到的数据有尾巴长度,这个用字母D,还有X,Y轴坐标,这个用F,后两位表示x和y,还有清除标志字母C,
    //要实现的功能是根据尾巴长度设置数组大小并持续接受信号,在屏幕上显示轨迹,先用红色吧,因为tcp的可靠性,不担心出问题,
    //模式4是模式2.2,这个模式有坐标,清除,撤销,rgb,没了应该,实现了的功能是,点击和移动一样,都是把显示一个灯(add_light)和其他已有的灯(store_light),然后松开是把这个灯
    //加到已有灯上面,点击清除和撤销的话就是把仓库清零,和仓库最后一个清0
    //通信规定是坐标一样F,rgb用G,清除用C,和之前的一样,撤销用H,好,上午到这,今天上午就把昨天写的bug变成程序了
    //程序是这样,读取坐标,并显示这个坐标和仓库的值,当松手时,即收到"I"时把这个坐标加进仓库
    //改进,手机在进行模式4时,为默认rgb,默认rgb为蓝色,然后用默认rgb进行显示这个坐标的数,当抬手时,会发送一个I,这时候count就+1,如果这时候收到rgb值,
    //就会把改rgb_store的值的count-1改,并显示
    int len;
    char rx_buffer[128];
 
    
    int mode_number = 1;//任务标识,1是默认任务,2是昨天加的任务,3是熄灯
    do{  //do,做一遍,再看while,直到while成立,break
        len = recv(sock,rx_buffer, sizeof(rx_buffer) - 1, 0);//-1是为了保留结尾字符,c语言str机制,如果len小于等于0,说明接受出错
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            rx_buffer[len] = 0; // str的结束标志,将所有发来的东西用str处理,这样方便管理
            printf(rx_buffer);
            for (size_t i = 0; i < len; i++)
            {   
                if (rx_buffer[i]=='M')
                {
                    mode_number = rx_buffer[i+1]-90;//用大写字母后的ascii做数字
                    mode_function(mode_number); //这个并不是实现模式功能的函数,而是保证不冲突
                }
                
                if (rx_buffer[i]=='L')
                {
                    light_coefficient = (uint8_t) rx_buffer[i+1]-90;
                }
                
                
                
                switch (mode_number)
                {
                case 1:
                    if(rx_buffer[i]=='A'){
                       mic_sensitive = rx_buffer[i+1]-90;
                       sleep_time = rx_buffer[i+2]-90;
                       sleep_coefficient = rx_buffer[i+3]-90;
                       gravity_coefficient = rx_buffer[i+4]-90;
                       save_to_flash_uint8(mic_sensitive,sleep_time,sleep_coefficient,gravity_coefficient,light_coefficient);
                       printf("%d\n",light_coefficient);
                        }
                    if(rx_buffer[i]=='B'){
                       mic_sensitive = rx_buffer[i+1]-90;
                       sleep_time = rx_buffer[i+2]-90;
                       sleep_coefficient = rx_buffer[i+3]-90;
                       gravity_coefficient = rx_buffer[i+4]-90;
                        }
                    break;
                case 2:
                if (rx_buffer[i]=='C')
                {
                    task_3();
                    memset(x_array,0,sizeof(x_array));
                    memset(y_array,0,sizeof(y_array));
                    coordinate_count = 0;
                }
                if (rx_buffer[i]=='D')
                {
                    task_3();
                    length_of_tail = rx_buffer[i+1]-90;
                    memset(x_array,0,sizeof(x_array));
                    memset(y_array,0,sizeof(y_array));
                    coordinate_count = 0;
                    
                }
                if (rx_buffer[i]=='F')
                {
                    
                    x_array[coordinate_count] = rx_buffer[i+1]-91;
                    y_array[coordinate_count] = rx_buffer[i+2]-91;
                    coordinate_count++;
                    task_2(x_array,y_array,length_of_tail);
                }
                if (coordinate_count>=length_of_tail)
                {
                    coordinate_count = 0;
                }
                break;
                case 4:
                switch (rx_buffer[i])
                {
                case 'G':
                if (coordinate_count>0)
                {   
                    get_rgb.red =(rx_buffer[i+1]-90);
                    get_rgb.green =(rx_buffer[i+2]-90);
                    get_rgb.blue =(rx_buffer[i+3]-90);
                    rgb_store[coordinate_count-1].red = get_rgb.red*light_coefficient;
                    rgb_store[coordinate_count-1].green = get_rgb.green*light_coefficient;
                    rgb_store[coordinate_count-1].blue = get_rgb.blue*light_coefficient;
                    task_4(x_array,y_array,coordinate_count,rgb_store);
                }
                

                    break;
                case 'F':
                    x_array[coordinate_count] = rx_buffer[i+1]-91;
                    y_array[coordinate_count] = rx_buffer[i+2]-91;
                    if (coordinate_count == 0)
                    {
                        get_rgb.blue = 15*light_coefficient;
                    }
                    
                    rgb_store[coordinate_count] = get_rgb;
                    task_4(x_array,y_array,coordinate_count+1,rgb_store);
                break;
                
                case 'I':
                    coordinate_count++;

                break;
                case 'L':
                    rgb_store[coordinate_count-1].red = get_rgb.red*light_coefficient;
                    rgb_store[coordinate_count-1].green = get_rgb.green*light_coefficient;
                    rgb_store[coordinate_count-1].blue = get_rgb.blue*light_coefficient;
                    task_4(x_array,y_array,coordinate_count,rgb_store);
                break;
                
                //还有撤回按键要弄,看样子明天还得干.


                default:
                    break;
                }

                    break;  
                


                
                default:
                    break;
                }
                
                
            }
            
    }
    }while (len>0);
}




static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = 120;
    int keepInterval = 60;
    int keepCount = 5;
    struct sockaddr_storage dest_addr;


    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(1);
        ip_protocol = IPPROTO_IP;
    }//设置ip地址,端口,ipv4等

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);//开始创建socket
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));//看不懂,不知道干啥的

// #if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
//     // Note that by default IPV6 binds to both protocols, it is must be disabled
//     // if both protocols used at the same time (used in CI)
//     setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
// #endif

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));//socket绑定ip
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;//关闭连接的,goto指令哈哈
    }
    ESP_LOGI(TAG, "Socket bound, port %d", 1);

    err = listen(listen_sock, 1);//监听
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);//sock为套接字描述符,服务器连接到客户端时,给出一个sock,可以用这个sock进行通信
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        //保活设置,注意,很多c语言函数喜欢得到变量的地址,然后从地址取值,所以下面传入的都是地址,&取地址,然后在函数内部通过地址取值,(*+地址)
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }//这个函数只是把地址变为str,好让后面的函数好打印


        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        recv_from_android(sock);
        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP://汇编里的goto,稀罕物啊
    close(listen_sock);
    vTaskDelete(NULL);
}


static esp_err_t example_set_dns_server(esp_netif_t *netif, uint32_t addr, esp_netif_dns_type_t type)
{
    if (addr && (addr != IPADDR_NONE)) {
        esp_netif_dns_info_t dns;
        dns.ip.u_addr.ip4.addr = addr;
        dns.ip.type = IPADDR_TYPE_V4;
        ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, type, &dns));
    }
    return ESP_OK;
}





static void example_set_static_ip(esp_netif_t *netif)
{
    if (esp_netif_dhcpc_stop(netif) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop dhcp client");
        return;
    }
    esp_netif_ip_info_t ip;
    memset(&ip, 0 , sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr(EXAMPLE_STATIC_IP_ADDR);
    ip.netmask.addr = ipaddr_addr(EXAMPLE_STATIC_NETMASK_ADDR);
    ip.gw.addr = ipaddr_addr(EXAMPLE_STATIC_GW_ADDR);
    if (esp_netif_set_ip_info(netif, &ip) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ip info");
        return;
    }
    ESP_LOGD(TAG, "Success to set static ip: %s, netmask: %s, gw: %s", EXAMPLE_STATIC_IP_ADDR, EXAMPLE_STATIC_NETMASK_ADDR, EXAMPLE_STATIC_GW_ADDR);
    ESP_ERROR_CHECK(example_set_dns_server(netif, ipaddr_addr(EXAMPLE_MAIN_DNS_SERVER), ESP_NETIF_DNS_MAIN));
    ESP_ERROR_CHECK(example_set_dns_server(netif, ipaddr_addr(EXAMPLE_BACKUP_DNS_SERVER), ESP_NETIF_DNS_BACKUP));
}


void wifi_call_back(void* event_handler_arg,esp_event_base_t event_base,int32_t event_id,void* event_data)
{   
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        //example_set_static_ip(event_handler_arg);
        printf("***************************************\n");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 3) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "static ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET, 5, NULL);
    }

}


void wifi_connecter(void)/////这个函数实现了连接wifi的功能
{

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_call_back,
                                                        sta_netif,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_call_back,
                                                        sta_netif,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ssid_1,
            .password = password_1,
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,//更高级的WPA2,不用懂
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);


    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 ssid_1, password_1);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 ssid_1, password_1);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}





void app_main(void)
{   
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }


    i2s_example_init_std_duplex();
    xQueue = xQueueCreate(10, sizeof(int *));

    send_buffer = calloc(LIGHT_NUMBER + 1, 12);
    spi = spi_initialize();
    t_1.length = 8 * 12 * LIGHT_NUMBER + 16;
    t_1.tx_buffer = send_buffer;
    //save_to_flash_uint8(mic_sensitive,sleep_time,sleep_coefficient,gravity_coefficient,light_coefficient);
    read_from_flash_uint8(&mic_sensitive,&sleep_time,&sleep_coefficient,&gravity_coefficient,&light_coefficient);
    xTaskCreate(receive_data_2, "receive_data_2", 8192*4, NULL, 5, &task_1_1_music_get_number);
    xTaskCreate(shape_of_music, "task_1_music", 8192, NULL, 5, &task_1_music_show);
    wifi_connecter();
}

