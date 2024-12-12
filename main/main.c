#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "config.h"
#include "crsf.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/uart.h"

#include "nvs_flash.h"
#include "mpu6050.h"
#include "imu.h"
#include "time.h"
#include "esp_timer.h"

#include "lwip/sockets.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

// UDP Variable
#define WIFI_SSID "ruter"
#define WIFI_PASS "caksa123"

#define PORT 3333
#define HOST_IP_ADDR "192.168.0.110" // 192.168.43.32



#define ECHO_UART_PORT_NUM      2
#define ECHO_UART_BAUD_RATE     460800
#define ECHO_TASK_STACK_SIZE    3072 

char payload[128];

int loopCounts;

uint8_t crsfPacket[CRSF_PACKET_SIZE] = {0};
uint8_t crsfCmdPacket[CRSF_CMD_PACKET_SIZE] = {0};
int16_t rcChannels[CRSF_MAX_CHANNEL] = {0};
uint32_t crsfTime = 0;

// CRSF crsfclass;

#define UART_PIN UART_NUM_2
#define RX_BUF_SIZE 1024

#define RXD_PIN (GPIO_NUM_16)
#define TXD_PIN (GPIO_NUM_17)

QueueHandle_t uart_queue;

void uart_init(){
 uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_PIN, RX_BUF_SIZE * 3, RX_BUF_SIZE * 3, 10,  &uart_queue, 0);
    ESP_ERROR_CHECK(uart_param_config(UART_PIN, &uart_config));
    uart_set_pin(UART_PIN,  TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_line_inverse(UART_PIN, UART_SIGNAL_TXD_INV);
}

int sendData(const char* logName,  const char* data){
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_PIN, data, len);
    ESP_LOGI(logName,  "wrote %d bytes", txBytes);
    return txBytes;
}
static void tx_task(void *arg){
    
    uint8_t tes[] = {
        0xAA, 0xAA, 0x01, 0x01, 0xFF, 0xFF
    };

    uint8_t first[] = {
        0xEA,
        0x0D,
        0x3A,
        0xEA,
        0xEE,
        0x10,
        0x00,
        0x01,
        0x04,
        0x64,
        0xFF,
        0xFF,
        0xFC,
        0x18,
        0xC4,
        0xF0,
    };

    uint8_t second[] = {  
        0xEE,
        0x18,
        0x16,
        0xDC,
        0xB3,
        0x1F,
        0x3D,
        0xC0,
        0x37,
        0xF1,
        0x56,
        0xB4,
        0xA2,
        0x15,
        0xE0,
        0x03,
        0x1F,
        0xF8,
        0xC0,
        0x07,
        0x3E,
        0xF0,
        0x81,
        0x0F,
        0x7C,
        0x08,
    };

    uint8_t third[] = {
        0xEA,
        0x0C,
        0x14,
        0xFF,
        0x00,
        0x00,
        0x0D,
        0x00,
        0x05,
        0x07,
        0xB1,
        0x00,
        0x0D,
        0xD0,
        0x80,
    };
    uint8_t null[1] = { 0};
int count = 0;
    while(1){
        if(count <= 5){
        uart_write_bytes(UART_PIN, tes, 6);
        // uart_write_bytes(UART_PIN, first, sizeof(first)/sizeof(first[0]));
        // uart_write_bytes(UART_PIN, null, 1);
        printf("write first\n");
        // count++;
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        if(count >  5){
        printf("write second \n");
        uart_write_bytes(UART_PIN, second, sizeof(second)/sizeof(second[0]));
        // uart_write_bytes(UART_PIN, null, 1);
        vTaskDelay(0.3 / portTICK_PERIOD_MS);
        uart_write_bytes(UART_PIN, third, sizeof(third)/sizeof(third[0]));
        printf("write third \n");
        // uart_write_bytes(UART_PIN, null, 1);
        vTaskDelay(3.5 / portTICK_PERIOD_MS);
        // count++;
        }
        // TickType_t tx_start = xTaskGetTickCount();
        // sendData("TX TASK", "hello world");
        // TickType_t tx_end = xTaskGetTickCount();
        // printf("TX TIME: %lu ticks \n", tx_end - tx_start);
        // // printf("hello world\n");
        // vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
static void rx_task(){
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE +1);
    while(1){
    memset(data, 0, sizeof(data));

    TickType_t tx_start = xTaskGetTickCount();
    const int rxBytes = uart_read_bytes(UART_PIN, data, RX_BUF_SIZE *2,500 / portTICK_PERIOD_MS);
    TickType_t tx_end = xTaskGetTickCount();
    printf("RX TIME: %lu ticks \n", tx_end - tx_start);
     
    // printf("rx data %s is %d \n", data, rxBytes);
    if(rxBytes > 0){
        // ESP_LOGI("rx data", " %s is %d", data, rxBytes);
        // int sz = sizeof(data) / sizeof(data[0]);
        printf("RX %d: ", 16);
        for(int i = 0; i < 16; i++){
            printf("%02X, ", data[i]);
        }
        printf("\n");
        // ESP_LOGI("RX_TASK", "read %d  bytes: %c", rxBytes, data);
        // ESP_LOG_BUFFER_HEXDUMP("RX_TASK", data, rxBytes, ESP_LOG_INFO);
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    free(data);
}

void i2c_init() {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 22,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void adc_init() {
    esp_adc_cal_characteristics_t adc1_chars;

    // Config Pin and Voltage Range for Pin D33 or ADC1_CHANNEL_5
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
    // Config ADC Width Range Value
    adc1_config_width(ADC_WIDTH_BIT_10);
    // Calibration
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
}

void switch_init() {
    /*Switch Mode Init pin : D5 or GPIO 5 */
    gpio_config_t io_conf1 = {
        .intr_type = GPIO_INTR_DISABLE,        // Disable interrupt
        .mode = GPIO_MODE_INPUT,               // Set as output mode
        .pin_bit_mask = (1ULL << GPIO_NUM_5),  // Bit mask of the pins that you want to set
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down mode
        .pull_up_en = GPIO_PULLUP_ENABLE       // Disable pull-up mode
    };
    gpio_config(&io_conf1);

    /*Switch Mode Init pin : D4 or GPIO 4 */
    gpio_config_t io_conf2 = {
        .intr_type = GPIO_INTR_DISABLE,        // Disable interrupt
        .mode = GPIO_MODE_INPUT,               // Set as output mode
        .pin_bit_mask = (1ULL << GPIO_NUM_4),  // Bit mask of the pins that you want to set
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down mode
        .pull_up_en = GPIO_PULLUP_ENABLE       // Disable pull-up mode
    };
    gpio_config(&io_conf2);

    /*Switch Mode Init pin : D19 or GPIO 19 */
    gpio_config_t io_conf3 = {
        .intr_type = GPIO_INTR_DISABLE,        // Disable interrupt
        .mode = GPIO_MODE_INPUT,               // Set as output mode
        .pin_bit_mask = (1ULL << GPIO_NUM_19), // Bit mask of the pins that you want to set
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down mode
        .pull_up_en = GPIO_PULLUP_ENABLE       // Disable pull-up mode
    };
    gpio_config(&io_conf3);

    /*Switch Mode Init pin : D26 or GPIO 26 */
    gpio_config_t io_conf4 = {
        .intr_type = GPIO_INTR_DISABLE,        // Disable interrupt
        .mode = GPIO_MODE_INPUT,               // Set as output mode
        .pin_bit_mask = (1ULL << GPIO_NUM_26), // Bit mask of the pins that you want to set
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down mode
        .pull_up_en = GPIO_PULLUP_ENABLE       // Disable pull-up mode
    };
    gpio_config(&io_conf4);

    /*Switch Mode Init pin : D15 or GPIO 15 */
    gpio_config_t io_conf5 = {
        .intr_type = GPIO_INTR_DISABLE,        // Disable interrupt
        .mode = GPIO_MODE_INPUT,               // Set as output mode
        .pin_bit_mask = (1ULL << GPIO_NUM_15), // Bit mask of the pins that you want to set
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down mode
        .pull_up_en = GPIO_PULLUP_ENABLE       // Disable pull-up mode
    };
    gpio_config(&io_conf5);

}

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    switch (event_id) {
        case WIFI_EVENT_STA_START:
            printf("WiFi connecting WIFI_EVENT_STA_START ... \n");
            break;
        case WIFI_EVENT_STA_CONNECTED:
            printf("WiFi connected WIFI_EVENT_STA_CONNECTED ... \n");
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            printf("WiFi lost connection WIFI_EVENT_STA_DISCONNECTED ... \n");
            esp_wifi_start();
            esp_wifi_connect();
            break;
        case IP_EVENT_STA_GOT_IP:
            printf("WiFi got IP ... \n\n");
            break;
        default:
            break;
    }
}

void wifi_connection_sta() {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS} };
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    esp_wifi_set_mode(WIFI_MODE_STA);

    esp_wifi_start();
    esp_wifi_connect();
}

static void udp_client_task(void *pvParameters) {
    /* General Variable */
    // char *payload = (char *)pvParameters;
    static const char *TAG = "UDP SOCKET CLIENT";
    // char rx_buffer[128];
    // char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    while (1) {
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }

        // Set timeout
        struct timeval timeout;
        // timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        // ESP_LOGI(TAG, "Socket created, sending to %s:%d", host_ip, PORT);

        while (1) {

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                // ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            // printf("%s ", payload);

            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            // ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

float mapValue(float value, float inputMin, float inputMax, float outputMin, float outputMax) {
    if (value < inputMin) return outputMin;
    else if (value > inputMax) return outputMax;

    // Map the input value to the output range
    float inputRange = inputMax - inputMin;
    float outputRange = outputMax - outputMin;
    return ((value - inputMin) * outputRange / inputRange) + outputMin;
}

void start_process() {


    // IMU data
    struct full_imu_data left_imu_data;
    left_imu_data.q[0] = 1.0;

    struct full_imu_data right_imu_data;
    right_imu_data.q[0] = 1.0;

    /*Roll Pitch Yaw variable*/
    float roll, pitch, throttle, yaw, yaw_1;
    uint16_t roll_pwm, pitch_pwm, yaw_pwm;

    /* GPIO and ADC Variable*/
    uint16_t mode_1, mode_2, mode_pwm, arming, arming_pwm, throttle_pwm, magnet, magnet_pwm, poshold;

    // Initialize MPU6050
    mpu6050_handle_t left_imu = imu_init(I2C_NUM_0, MPU6050_I2C_ADDRESS_1);
    mpu6050_handle_t right_imu = imu_init(I2C_NUM_0, MPU6050_I2C_ADDRESS);

    /* Payload Variable */
    // char *payload = (char *)pvParameters;

    /*Calibrate Gyro*/
    for (int i = 0; i < 2000; i++) {
        imu_read_raw(left_imu, &left_imu_data.gyro, &left_imu_data.acce);
        imu_read_raw(right_imu, &right_imu_data.gyro, &right_imu_data.acce);

        imu_add(&left_imu_data.offset, left_imu_data.gyro);
        imu_add(&right_imu_data.offset, right_imu_data.gyro);
    }
    imu_divide_single(&left_imu_data.offset, 2000);
    imu_divide_single(&right_imu_data.offset, 2000);
    printf("Gyro Calibration done !!\n");

    while (true) {
        imu_read(left_imu, &left_imu_data);
        imu_read(right_imu, &right_imu_data);

        /*Get Roll Pitch Throttle Yaw*/
        roll = right_imu_data.processed.y;
        pitch = right_imu_data.processed.x;

        throttle = left_imu_data.processed.x;
        yaw = left_imu_data.processed.y;

        /*Limit Roll  Angle*/
        if (roll < -70) {
            roll = -70;
        } else if (roll > 70) {
            roll = 70;
        }

        /*Limit Pitch Angle*/
        if (pitch < -65) {
            pitch = -65;
        } else if (pitch > 65) {
            pitch = 65;
        }

        /*Mapping degree to PWM*/
        roll_pwm = mapValue(roll, -45, 45, 1000, 2000);
        pitch_pwm = mapValue(pitch, -45, 45, 2000, 1000);
        throttle_pwm = mapValue(throttle, -45, 45, 1000, 2000);
       // yaw_pwm = 1500;

        /* 0 to 360 degrees with Relative North Position */
        // if (yaw < 0)
        //     yaw += 360.0; // compass circle
        // // correct for local magnetic declination here

        yaw_1 = yaw;
        // Ensure yaw angle is within the range -180 to 180 degrees
        if (yaw > 180.0) {
            yaw -= 360.0;
        } else if (yaw < -180.0) {
            yaw += 360.0;
        }

        // Ensure yaw angle is within the range -90 to 90 degrees
        if (yaw > 90.0) {
            yaw -= 180.0;
        } else if (yaw < -90.0) {
            yaw += 180.0;
        }

        // Limit PWM Value
        if (yaw_1 < -90) {
            yaw_pwm = 1000;
        } else if (yaw_1 > 90) {
            yaw_pwm = 2000;
        } else {
            yaw_pwm = ((yaw + 90) / 180) * (2000 - 1000) + 1000;
        }

        // Deadzone
        if (yaw_pwm > 1450 && yaw_pwm < 1550) {
            yaw_pwm = 1500;
        } else if (yaw_pwm > 1450) {
            yaw_pwm += 50;
        } else if (yaw_pwm < 1550) {
            yaw_pwm -= 50;
        }

        /*Switch Mode*/
        mode_1 = gpio_get_level(GPIO_NUM_5);
        mode_2 = gpio_get_level(GPIO_NUM_19);
        if (mode_1 == 1 && mode_2 == 0) {
            mode_pwm = 2000; // pos hold
            // if (throttle_pwm > 1430 && throttle_pwm < 1570){
            //     throttle_pwm = 1500;
            // }
        } else if (mode_1 == 0 && mode_2 == 1) {
            mode_pwm = 1000; // Stabilize
        } else {
            mode_pwm = 1500; // Land
        }

        /*Switch Arming*/
        arming = gpio_get_level(GPIO_NUM_4);
        if (arming == 1) {
            arming_pwm = 2000;
        } else {
            arming_pwm = 1000;
        }

        /*Switch Magnet*/
        magnet = gpio_get_level(GPIO_NUM_26);
        if (magnet == 1) {
            magnet_pwm = 2000;
        } else {
            magnet_pwm = 1000;
        }

        /*Switch Force Poshold*/
        poshold = gpio_get_level(GPIO_NUM_15);
        if (poshold == 1) {
            // yaw_pwm = 1500;
        }
        rcChannels[0] = roll_pwm;
        rcChannels[1] = pitch_pwm;
        rcChannels[2] = throttle_pwm;
        rcChannels[3] = yaw_pwm;
        rcChannels[4]= magnet_pwm;
        rcChannels[5] = arming_pwm;
        rcChannels[6] = mode_pwm;
        // sprintf(payload, "r%dp%dy%dm%d\n", roll_pwm, pitch_pwm, yaw_pwm, mode_pwm);
        // sprintf(payload, "r%dp%dt%dy%dm%da%dg%d", roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm, mode_pwm, arming_pwm, magnet_pwm);
        sprintf(payload, "r%dp%dt%dy%d\n", roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm);
        // printf("%d\n", yaw_pwm);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
void monitor_task(void *pvParameter){
    while(1){
        printf("%s\n", payload);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
void elrs_task(void *pvParameter){
    // crsfclass.begin();
        rcChannels[0] = 1200;
        rcChannels[1] = 1500;
        rcChannels[2] = 1100;
        rcChannels[3] = 1600;
        rcChannels[4]= 1800;
        rcChannels[5] = 2000;
        rcChannels[6] = 1500;

        // uint8_t packet3[] = {0xC8, 0x04, 0x28, 0x00, 0xEA, 0x0x
        // uint8_t packet3[] = {0xEE,  0x06  ,0x2D  ,0xEE  , 0xEA  , 0x01  ,0x02  ,0x4F};
        // uint8_t packet[] = {0xC8, 0x04, 0x28, 0x00, 0xEA, 0x54};
        // uint8_t packet2[] = {0xC8, 0x06, 0x2C, 0xEE, 0xEF, 0x10, 0x00, 0xCD};

    // uint8_t packet2[6] = {};
    //     for(int i = 0;i < 6; i++){
    //         packet2[i] = ~packet[i];
    //     }
    while(1){
        // for(int i = 0; i < 6; i++){
        //     printf(" %02X ", packet[i]);
        // }
        // uart_write_bytes(UART_PIN, packet, 6);
        // uart_write_bytes(UART_PIN, packet2, 8);
        // uart_write_bytes(UART_PIN, packet2, 8);
        // uart_write_bytes(UART_PIN, packet3, 8);
        // printf("TX\n");
        // const int rxBytes = uart_read_bytes(UART_PIN, data, RX_BUF_SIZE *2,500 / portTICK_PERIOD_MS);
        
        // crsf_prepare_cmd_packet(crsfCmdPacket, ELRS_PKT_RATE_COMMAND, SETTING_1_PktRate);
        // printf("CMD PACKET :");
        // for(int i = 0; i < 8; i++){
        
        //     printf(" %02X ", crsfCmdPacket[i]);
        // }
        // printf("\n DATA PACKET");
        // crsf_prepare_data_packet(crsfPacket, rcChannels);
        // for(int i = 0; i< 26; i++){
        //     printf(" %02X ", crsfPacket[i]);
        // }
        // printf("\n");


        // uint32_t currentMicros = micros();
        // crsf_prepare_data_packet(crsfPacket, rcChannels);
    //     printf("\n hello world\n");

    if(loopCounts <= 500){ //kirim 500 packet
        crsf_prepare_data_packet(crsfPacket, rcChannels);
        // crsfclass.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
        uart_write_bytes(UART_PIN, crsfPacket, CRSF_PACKET_SIZE);
    //    printf("TX : ");
    //     for(int i = 0; i < 26; i++){
    //         printf("%02X ", crsfPacket[i]);
    //     }
    //     printf("\n");
        // printf("%s\n", crsfPacket);
        loopCounts++;
    }
    if(loopCounts >500 && loopCounts <=505){
        crsf_prepare_cmd_packet(crsfCmdPacket, ELRS_PKT_RATE_COMMAND, SETTING_1_PktRate);
        // buildElrsPacket(crsfCmdPacket,ELRS_WIFI_COMMAND,0x01);
        // crsfclass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
        uart_write_bytes(UART_PIN, crsfPacket, CRSF_PACKET_SIZE);
    loopCounts++;
    }
    if(loopCounts >505 && loopCounts <=510){
        crsf_prepare_cmd_packet(crsfCmdPacket, ELRS_POWER_COMMAND, SETTING_2_Power);
        // buildElrsPacket(crsfCmdPacket,ELRS_WIFI_COMMAND,0x01);
        // crsfclass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
        uart_write_bytes(UART_PIN, crsfPacket, CRSF_PACKET_SIZE);
    loopCounts++;
    }
    if(loopCounts >= 510) loopCounts = 0;
    //1.6ms
    vTaskDelay(10 / portTICK_PERIOD_MS);


    }
}

void app_main(void) {
    uart_init();
    // i2c_init();
    // start_process();
    // xTaskCreate(rx_task, "rx", 1024*2, NULL,  0, NULL);
    xTaskCreate(tx_task, "tx", 1024*2, NULL, 3, NULL);
    // adc_init();
    // switch_init();

    // wifi_connection_sta();
    // elrs_task();

    
    // xTaskCreate(monitor_task, "monitor", 4096, NULL, 1, NULL);
    // xTaskCreate(elrs_task, "elrs task",  4096, NULL, 2, NULL);
    
    // xTaskCreate(start_process, "start_process", 4096, NULL, 3, NULL);
    // vTaskDelay(100 / portTICK_PERIOD_MS);
    // xTaskCreate(udp_client_task, "udp_cilent_task", 4096, NULL, 4, NULL);
}