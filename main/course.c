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

int try_time = 0;
#define TRY_TIMES 4

int tick_1 = 0;

bool wifi_state =false;
//每次触摸时看是否为true,是就不再连接,否就尝试连接,连接时看try是否超标,超标就不再进行,并且把try归零.加载状态就看wifi_state和trytime.有一个符合就break

#define EXAMPLE_STD_BCLK_IO1 GPIO_NUM_25 // i2s
#define EXAMPLE_STD_WS_IO1 GPIO_NUM_26   
#define EXAMPLE_STD_DOUT_IO1 GPIO_NUM_27
#define EXAMPLE_STD_DIN_IO1 GPIO_NUM_14 


static const char *TAG = "example";
#define ssid_1  "muzi"
#define password_1 "88888888"//wifi


#define WIFI_CONNECTED_BIT BIT0//静态ip所需
#define WIFI_FAIL_BIT      BIT1
static EventGroupHandle_t s_wifi_event_group;

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
#define PIN_NUM_MOSI GPIO_NUM_12
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
                .msb_right =false,
                 //.left_align = true,
               // .big_endian = false,
                //.bit_order_lsb = false
                },
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

static int16_t bit_and_32(uint32_t input){
    int32_t middle = (int)input;
    middle = middle>>(16-mic_sensitive);
    return middle;
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
    uint32_t *r_buf = calloc(EXAMPLE_BUFF_SIZE, sizeof(uint32_t));
    int16_t result[1024];
    assert(r_buf); // Check if r_buf allocation success
    blackman_window(wind, N);
    size_t r_bytes = 0;
    vTaskDelay(100);
    while (1)
    {

        if (i2s_channel_read(rx_chan, r_buf, EXAMPLE_BUFF_SIZE *sizeof(uint32_t), &r_bytes, 1000) == ESP_OK)
        {
            
            // for (int i = 0; i < 3 * EXAMPLE_BUFF_SIZE; i += 3)
            // {
            //     result[i / 3] = bit_and(r_buf[i + 2], r_buf[i + 1], r_buf[i]);
            //     //loud_change(&r_buf[i+2],&r_buf[i + 1], &r_buf[i]);
            // }
            for (int i = 0; i < EXAMPLE_BUFF_SIZE; i ++)
            {
                result[i] = bit_and_32(r_buf[i]);
                //loud_change(&r_buf[i+2],&r_buf[i + 1], &r_buf[i]);
            }
            //i2s_channel_write(tx_chan, r_buf, 3 * EXAMPLE_BUFF_SIZE * sizeof(uint8_t), &r_bytes, 1000);
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

// void lighter(RGB_struct *input_date)
// {

//     spi_transaction_t t_1 = {
//         .length = 8 * 12 * LIGHT_NUMBER + 16,
//         .tx_buffer = send_buffer,
//     };

//     translate(input_date, send_buffer);
//     ESP_ERROR_CHECK(spi_device_transmit(spi, &t_1));
// }
void lighter_2(RGB_struct *input_date)//加队列,不知道可行不
{
    vTaskDelay(1/portTICK_PERIOD_MS);
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
        lighter_2(input_data_all);
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
        change_the_form_and_package_point(du_RGB, x[i], input_data_all, y[i]);
    }
    
    lighter_2(input_data_all);
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
    int x_test[256] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,14,14,14,14,14,14,14,14,14,14,14,14,14,13,12,11,10,9,8,7,6,5,4,3,2,1,1,1,1,1,1,1,1,1,1,1,1,1,2,3,4,5,6,7,8,9,10,11,12,13,13,13,13,13,13,13,13,13,13,13,13,12,11,10,9,8,7,6,5,4,3,2,2,2,2,2,2,2,2,2,2,2,3,4,5,6,7,8,9,10,11,12,12,12,12,12,12,12,12,12,12,11,10,9,8,7,6,5,4,3,3,3,3,3,3,3,3,3,4,5,6,7,8,9,10,11,11,11,11,11,11,11,11,10,9,8,7,6,5,4,4,4,4,4,4,4,5,6,7,8,9,10,10,10,10,10,10,9,8,7,6,5,5,5,5,5,6,7,8,9,9,9,9,8,7,6,6,6,7,8,8,9
};
    int y_test[256] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,3,4,5,6,7,8,9,10,11,12,13,14,14,14,14,14,14,14,14,14,14,14,14,14,14,13,12,11,10,9,8,7,6,5,4,3,2,2,2,2,2,2,2,2,2,2,2,2,2,3,4,5,6,7,8,9,10,11,12,13,13,13,13,13,13,13,13,13,13,13,13,12,11,10,9,8,7,6,5,4,3,3,3,3,3,3,3,3,3,3,3,4,5,6,7,8,9,10,11,12,12,12,12,12,12,12,12,12,12,11,10,9,8,7,6,5,4,4,4,4,4,4,4,4,4,5,6,7,8,9,10,11,11,11,11,11,11,11,11,10,9,8,7,6,5,5,5,5,5,5,5,6,7,8,9,10,10,10,10,10,10,9,8,7,6,6,6,6,6,7,8,9,9,9,9,8,7,7,7,8,8
};
int rgb [256] = {0,1,1,2,2,2,3,3,3,3,4,4,4,4,4,5,5,5,5,5,5,6,6,6,6,6,6,6,7,7,7,7,7,7,7,7,8,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9,9,9,10,10,10,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11,11,11,11,11,11,12,12,12,12,12,12,12,12,12,12,12,12,12,13,13,13,13,13,13,13,13,13,13,13,13,13,13,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,13,13,13,13,13,13,13,13,13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,11,11,11,11,11,11,10,10,10,10,10,10,10,10,10,10,10,9,9,9,9,9,9,9,9,9,9,8,8,8,8,8,8,8,8,8,7,7,7,7,7,7,7,7,6,6,6,6,6,6,6,5,5,5,5,5,5,4,4,4,4,4,3,3,3,3,2,2,2,1,1,0
};

int x_2[32]={2,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13,14,14,15,15,16,16,16};
int y_2[32]={6,7,8,5,9,4,10,3,11,3,12,3,13,4,14,5,15,4,14,3,13,3,12,3,11,4,10,5,9,6,7,8};
void test_aix(void)
{
    task_2(x_2,y_2,32);
}
void test_ma(void)
{   


    //printf("%d %d %d %d\n", rgb_test[0].blue, rgb_test[0].green, rgb_test[1].blue, rgb_test[1].red);
    //改,这样,先红色0,然后蓝色变强,然后绿色变强同时蓝色变弱,然后红色+2,重复即可
//     for (size_t r_i = 0; r_i < 15; r_i=r_i+2)
// {
//     for (size_t i_count = 0; i_count < 32; i_count++)
//     {
//         if (i_count<16)
//         {
//             rgb_store[i_count*(r_i+1)].blue = i_count;
//             rgb_store[i_count*(r_i+1)].red = r_i;
//             rgb_store[i_count*(r_i+1)].green = 0;
//         }else{
//             rgb_store[i_count*(r_i+1)].blue = 15-(i_count-16);
//             rgb_store[i_count*(r_i+1)].red = r_i;
//             rgb_store[i_count*(r_i+1)].green = i_count-16;
//         }
        
//     }
    
// }
int count_1 = 0;
while (1)
{   
    if (count_1%2)
    {
        
        for (size_t i = 0; i < 256; i++)
{   rgb_store[i].blue = 0;
    rgb_store[i].red = 15*light_coefficient;
    task_4(x_test,y_test,i+1,rgb_store);
    vTaskDelay(1);
}
    }else{
    for (size_t i = 0; i < 256; i++)
{   
    rgb_store[i].red = 0;
    rgb_store[i].blue = 15*light_coefficient;
}

for (size_t i = 0; i < 256; i++)
{
    task_4(x_test,y_test,i+1,rgb_store);
    vTaskDelay(1);
}
    }
    count_1++;
if (tick_1 == 1)
{
    tick_1 = 0;
    break;
}

    
}
    vTaskResume(task_1_music_show);
    vTaskResume(task_1_1_music_get_number);
}



void mode_function(int mode_number)
{
    switch (mode_number)
    {
    case 1:
        vTaskResume(task_1_music_show);
        vTaskResume(task_1_1_music_get_number);

        break;
    case 2:
        
        vTaskSuspend(task_1_music_show);
        vTaskSuspend(task_1_1_music_get_number);
        task_3();
        memset(x_array,0,sizeof(x_array));
        memset(y_array,0,sizeof(y_array));
        coordinate_count = 0;
        if (xQueueReset(xQueue) != pdPASS) 
        {
        }
        
        break;
    case 3:
        
        vTaskSuspend(task_1_music_show);
        vTaskSuspend(task_1_1_music_get_number);
        memset(rgb_store,0,sizeof(rgb_store));
        task_3();
        if (xQueueReset(xQueue) != pdPASS) 
        {
        }
        test_ma();
        break;
    case 4:
        
        vTaskSuspend(task_1_music_show);
        vTaskSuspend(task_1_1_music_get_number);
        task_3();
        memset(x_array,0,sizeof(x_array));
        memset(y_array,0,sizeof(y_array));
        memset(rgb_store,0,sizeof(rgb_store));
        coordinate_count = 0;
        get_rgb.blue = 15;
        get_rgb.red = 0;
        get_rgb.green = 0;
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
                case 3:
                
                break;

                case 1:
                    if(rx_buffer[i]=='A'){
                       mic_sensitive = rx_buffer[i+1]-90;
                       sleep_time = rx_buffer[i+2]-90;
                       sleep_coefficient = rx_buffer[i+3]-90;
                       gravity_coefficient = rx_buffer[i+4]-90;
                       save_to_flash_uint8(mic_sensitive,sleep_time,sleep_coefficient,gravity_coefficient,light_coefficient);
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
                    
                    x_array[coordinate_count] = 15-(rx_buffer[i+1]-91);
                    y_array[coordinate_count] = 15-(rx_buffer[i+2]-91);
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
                    x_array[coordinate_count] = 15-(rx_buffer[i+1]-91);
                    y_array[coordinate_count] = 15-(rx_buffer[i+2]-91);
                    rgb_store[coordinate_count].red= get_rgb.red*light_coefficient;
                    rgb_store[coordinate_count].blue= get_rgb.blue*light_coefficient;
                    rgb_store[coordinate_count].green= get_rgb.green*light_coefficient;
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

                case 'C':
                mode_function(mode_number);//重启模式
                break;
                case 'H':
                coordinate_count--;
                rgb_store[coordinate_count].red = 0;
                rgb_store[coordinate_count].blue = 0;
                rgb_store[coordinate_count].green = 0;
                x_array[coordinate_count] = 0;
                y_array[coordinate_count] = 0;
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


char ip_addr[16];


static void tcp_cilent_task()
{
    int sock = 0;
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(ip_addr);
    server_addr.sin_port = htons(12345);

    while (1)
    {   
        sock =  socket(AF_INET, SOCK_STREAM, 0);
        int err = connect(sock, (struct sockaddr *)&server_addr, sizeof(ip_addr));
        if (err == 0)
        {
            printf("tcp连接成功\n");
            break;
        }
        vTaskDelay(100);
        printf("出错了\n");
        close(sock);//关闭它
    }
    recv_from_android(sock);
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








void get_ip_str(const ip4_addr_t *ip, char *ip_str, size_t max_len) {
    //用于将ip地址转为字符串
    snprintf(ip_str, max_len, IPSTR, IP2STR(ip));
}

//连接时设置一个动画,就是马天洋的动画,在按下按钮的同时,进行wifi连接和动画播放,每次放完一次动画查一下连接次数,如果是0就不放动画了,否则就放
void call_back(void* event_handler_arg,esp_event_base_t event_base,int32_t event_id,void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {   
        if (try_time<TRY_TIMES)
        { 
            try_time++;
            esp_wifi_connect();
            
            printf("尝试次数:%d\n",try_time);

        }
        else
        {
            printf("连接失败\n");
            wifi_state = false;
            try_time = 0;
            tick_1 = 1;
        }
        
  
    }
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        printf("连接成功\n");
        ip_event_got_ip_t *gw_add = event_data;
        wifi_state = true;
        try_time = 0;
        tick_1 = 1;
        get_ip_str(&gw_add->ip_info.gw, ip_addr, sizeof(ip_addr));
        //printf("%s\n",ip_addr);
        xTaskCreate(tcp_cilent_task, "tcp_client", 4096, (void*)AF_INET, 5, NULL);
    }
}


void wifi_connecter(void)/////这个函数实现了连接wifi的功能
{
    //一些初始化的函数，包括事件循环的启用；但是不负责链接wifi,有官方函数链接
    ///////////////////////////////////
    /*事件*/
    esp_event_handler_instance_register(WIFI_EVENT,WIFI_EVENT_STA_DISCONNECTED,call_back,NULL,NULL);//连接失败
    esp_event_handler_instance_register(IP_EVENT,IP_EVENT_STA_GOT_IP,call_back,NULL,NULL);//连接成功
    /////////////////////////////
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    wifi_config_t sta_cfg = 
    {
        .sta=
        {
            .ssid = ssid_1,
            .password = password_1
        }   
    };
    esp_wifi_set_config(WIFI_IF_STA,&sta_cfg);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
    
}


#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_THRESH_PERCENT  (80)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)


static bool s_pad_activated[2];//中断用
static uint32_t s_pad_init_val[2];//


static void tp_example_set_thresholds(void)
{
    uint16_t touch_value;
    for (int i = 0; i < 2; i++) {
        //read filtered value
        touch_pad_read_filtered(i, &touch_value);
        s_pad_init_val[i] = touch_value;
        ESP_LOGI(TAG, "test init: touch pad [%d] val is %d", i, touch_value);
        //set interrupt threshold.
        ESP_ERROR_CHECK(touch_pad_set_thresh(i, touch_value * 2 / 3));

    }
}

static void tp_example_read_task(void *pvParameter)
{
    static int show_message;
    while (1) {
            //interrupt mode, enable touch interrupt
            touch_pad_intr_enable();
            for (int i = 0; i < 2; i++) {
                if (s_pad_activated[i] == true) {
                    
                if (wifi_state!=true)
                {
                    esp_wifi_connect();
                    mode_function(3);
                }


                    // Wait a while for the pad being released
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    // Clear information on pad activation
                    s_pad_activated[i] = false;
                    // Reset the counter triggering a message
                    // that application is running
                    show_message = 1;
                }
            }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/*
  Handle an interrupt triggered when a pad is touched.
  Recognize what pad has been touched and save it in a table.
 */
static void tp_example_rtc_intr(void *arg)
{
    uint32_t pad_intr = touch_pad_get_status();
    //clear interrupt
    touch_pad_clear_status();
    for (int i = 0; i < 2; i++) {
        if ((pad_intr >> i) & 0x01) {
            s_pad_activated[i] = true;
        }
    }
}

/*
 * Before reading touch pad, we need to initialize the RTC IO.
 */
static void tp_example_touch_pad_init(void)
{
    for (int i = 0; i < 2; i++) {
        //init RTC IO and mode for touch pad.
        touch_pad_config(i, TOUCH_THRESH_NO_USE);
    }
}


void touch_task(void)
{
    // Initialize touch pad peripheral, it will start a timer to run a filter
    ESP_LOGI(TAG, "Initializing touch pad");
    ESP_ERROR_CHECK(touch_pad_init());
    // If use interrupt trigger mode, should set touch sensor FSM mode at 'TOUCH_FSM_MODE_TIMER'.
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    // Set reference voltage for charging/discharging
    // For most usage scenarios, we recommend using the following combination:
    // the high reference valtage will be 2.7V - 1V = 1.7V, The low reference voltage will be 0.5V.
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    // Init touch pad IO
    tp_example_touch_pad_init();
    // Initialize and start a software filter to detect slight change of capacitance.
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
    // Set thresh hold
    tp_example_set_thresholds();
    // Register touch interrupt ISR
    touch_pad_isr_register(tp_example_rtc_intr, NULL);
    // Start a task to show what pads have been touched
    xTaskCreate(&tp_example_read_task, "touch_pad_read_task", 4096, NULL, 5, NULL);
}



void app_main(void)
{   
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

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
    esp_netif_create_default_wifi_sta();
    wifi_connecter();
    touch_task();
    
}




