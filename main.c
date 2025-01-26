#include <stdio.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "sys/param.h"

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_mac.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_event.h"
#include "esp_system.h"

#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "nvs_flash.h"
#include "ping/ping_sock.h"


#define SSID "Your-networks-name-(SSID)" 
#define PASS "Your-internet-password" 
#define PORT 1234

#define PULSES_PER_REV 800
#define PWM_DEFINITION 1023

//Motor M1
#define PWM1 8
#define ENC1A 9
#define ENC1B 10
#define MIN1A 11
#define MIN1B 12

//Motor M2
#define PWM2 13
#define ENC2A 14
#define ENC2B 1
#define MIN2A 2
#define MIN2B 42

//Motor N1
#define PWM3 41
#define ENC3B 40
#define ENC3A 39
#define MIN3B 38
#define MIN3A 37

//Motor N2
#define PWM4 36
#define ENC4B 35
#define ENC4A 48
#define MIN4B 47
#define MIN4A 21

static const char *TAG = "Main";


struct PID_CONFIG{ //Struct to configure the PID parameters

    float Kp;
    float Ki;
    float Kd;
    float MAX_PWM;
    float previous_err1; // e(k)
    float previous_err2; // e(k-1)
    float integral_err;  // Sum of error
    float last_output;  // PID output in last control period
    float min_output;   // PID minimum output limitation
    float max_integral; // PID maximum integral value limitation
    float min_integral; // PID minimum integral value limitation

} pid[4];

struct MOTOR{ //Struct to pass the data of the motor to the callback
    
    int t;

    uint8_t MINA;    //The enable clockwise
    uint8_t MINB;    //The enable counterclockwise
    uint8_t ENC_A;   //Encoder A pin
    uint8_t ENC_B;   //Encoder B pin

    ledc_channel_t pwm_channel; //Channel uses to generate the pwm especific for each motor
    adc_channel_t  adc_channel; //Channel to read the voltage reference to the motors current 140mV/A

    adc_cali_handle_t cali; //Handle to calibrate the raw data for the adc captured voltage

    int current;
    float target;    //Desired rotational speed
    float speed;     //rotational speed of the motor w = 2pi * speed (angular velocity)
    int32_t duty;    //Duty cycle of the pwm signal
    int32_t deltaT;  //The diference in time between each pulse of the encoder

    volatile bool flag;   //Flag to see if the interuption happed=nd before reseting the pwm. ( if(false) velocity == 0)
    volatile int64_t prevTime; //Saves the previous time to calculate deltaT in the interruption
    struct PID_CONFIG *PID; //The pid data for each motor

} M[4];

bool send_recieve_flag = false; //flag to skip the PID claculation when we send and recieve the data


esp_timer_handle_t periodic_timers[5]; //Array of timers for the callbacks for each motor


adc_cali_handle_t adc_cali_handle[4]; //Handles for calibration of the raw data of the adc
adc_oneshot_unit_handle_t adc_handle; //Handle for the adc


esp_err_t init_MOTOR(struct MOTOR *m, uint8_t MA, uint8_t MB, uint8_t ENCA, uint8_t ENCB, ledc_channel_t pwm_channel, adc_channel_t adc_channel, adc_cali_handle_t adc_cal, struct PID_CONFIG *pid);
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
esp_err_t init_EnablePins(uint8_t* pins);
esp_err_t set_pwm_duty(int duty, ledc_channel_t channel);
esp_err_t init_pin(uint8_t pin, uint8_t caso);
esp_err_t init_isr(uint8_t *irs_pins, uint8_t dim);
esp_err_t init_PID(struct PID_CONFIG *PID);
void init_adc(struct MOTOR *m);
esp_err_t init_pwm();


float pid_compute(struct MOTOR *M);
void isr_handler(void *args ); 
void init_high_precision_timers();
int hex_decode(char hex);
void hex_to_float(char *hex_num, float *list);

//WiFi
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void wifi_connection();


void PID_compute_callback(void *arg){
    

    struct MOTOR *m =  (struct MOTOR* )arg;

    if(send_recieve_flag == false){

        if (m->flag == false) {
            m->t++;
        } else {
            m->t = 0;
        }

        if (m->t >= 9){

            m->speed = 0;

        } else {

            if(m->deltaT == 0){
                m->speed = 0;   //At the beggining deltaT is initialized as 0 and thus the speed is is 0
            } else {
                m->speed = (float) 1250 / (m->deltaT);
            }

        }

        m->duty = pid_compute(m);  // PID calculation

        set_pwm_duty(m->duty, m->pwm_channel);

        m->flag = false;  // Reset flag
    }
   

}

void ADCTimerCallback(void *arg){

    int raw;

    struct MOTOR *info = (struct MOTOR* )arg;

    for(int i = 0; i < 4; i++){
        adc_oneshot_read(adc_handle, info[i].adc_channel, &raw);
        adc_cali_raw_to_voltage(adc_cali_handle[i], raw, &(info[i].current));

        if(info[i].current >= 1400){
            info[i].target = 0;
        }
    }

}

void send_recieve(void *pvParameters){

    //struct MOTOR *info = (struct MOTOR* )pvParameters;
    
    char *payload = "00000000";
    char rx_buffer[10];
    char host_ip[] = "0.0.0.0"; //Your server IP
    int addr_family = 0;
    int ip_protocol = 0;
    float speeds[4];
    

    while (1)
    {
        
        struct sockaddr_in dest_addr;
        inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created ");
        ESP_LOGI(TAG, "Connecting to %s:%d", host_ip, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0)
        {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Successfully connected");
        
        while(1){

            send_recieve_flag = true;
            for(int i = 0; i < 4; i++){
                M[i].target = speeds[i] / 100.00; 
            }
            send_recieve_flag = false;
            
            int err = send(sock, payload, strlen(payload), 0);

            if(err < 0){
                ESP_LOGI(TAG, "Error ocurred during sending: errno %d", errno);
                break;    
            }

            vTaskDelay(10 / portTICK_PERIOD_MS);

            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if(len < 0){
                ESP_LOGI(TAG, "recv failed: errno %d", errno);
                break;
            } else {
                rx_buffer[len] = 0; // Null-terminate
                if(len == 8){
                    hex_to_float(rx_buffer, speeds);
                    send_recieve_flag = true;
                    for(int i = 0; i < 4; i++){
                        M[i].target = speeds[i] / 100.00; 
                    }
                    send_recieve_flag = false;
                    
                    ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                    ESP_LOGI(TAG, "%lf, %lf, %lf, %lf", M[0].target, M[1].target, M[2].target, M[3].target);
                } else {
                    ESP_LOGI(TAG, "Received incomplete data");
                }
            }
        }
        
        if (sock != -1)
        {
            shutdown(sock, 0);
            close(sock);
        }
    }

}


void app_main(void)
{    


    //Initialize the PID configuration structs
    struct PID_CONFIG pidConfig = {

        .Kp = 1000,
        .Ki = 5000,
        .Kd = 0

    };

    init_PID(&pidConfig); //Initialize the array of structs

    //Initializes the values of the structs of each motor
    init_MOTOR(&M[0], MIN1A, MIN1B, ENC1A, ENC1B, LEDC_CHANNEL_0, ADC_CHANNEL_3, adc_cali_handle[0] ,&pid[0]);
    init_MOTOR(&M[1], MIN2A, MIN2B, ENC2A, ENC2B, LEDC_CHANNEL_1, ADC_CHANNEL_4, adc_cali_handle[1] ,&pid[1]);
    init_MOTOR(&M[2], MIN3A, MIN3B, ENC3A, ENC3B, LEDC_CHANNEL_2, ADC_CHANNEL_5, adc_cali_handle[2] ,&pid[2]);
    init_MOTOR(&M[3], MIN4A, MIN4B, ENC4A, ENC4B, LEDC_CHANNEL_3, ADC_CHANNEL_6, adc_cali_handle[3] ,&pid[3]);

    //Initialize the pins for the pwm
    init_pin(PWM1, 0);
    init_pin(PWM2, 0);
    init_pin(PWM3, 0);
    init_pin(PWM4, 0);

    //Initialize the pins for the B encoders
    init_pin(ENC1B, 1);
    init_pin(ENC2B, 1);
    init_pin(ENC3B, 1);
    init_pin(ENC4B, 1);

    //Initialize the pins to generate pwm
    init_pwm();

    //Initialize the interruptions for the encoders
    uint8_t intr_pins[4] = {ENC1A, ENC2A, ENC3A, ENC4A};
    init_isr(intr_pins, 4);

    //Initialize the pins for the enable outputs of the motor
    uint8_t enable_pins[8] = {MIN1A, MIN2A, MIN3A, MIN4A, MIN1B, MIN2B, MIN3B, MIN4B};
    init_EnablePins(enable_pins);


    //Initialize the adc for each motor
    init_adc(M);

    wifi_connection();
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    //Initialize the timers for the callback functions
    init_high_precision_timers();

    
    xTaskCreate(send_recieve, "tcp_client", 8192, (void *)AF_INET, 10, NULL);
   

}

int hex_decode(char hex){
    int num = (int)hex;
    if(num <= 57 && num >= 49){
        num = num - 48;
    } else if (num <= 70 && num >= 65){
        num = num - 55;
    } else if (num <= 80 && num >= 71){
        num = -(num - 71);
    } else if (num <= 86 && num >= 81){
        num = -(num - 71);
    } else {
        return 0;
    }

    return num;
}

void hex_to_float(char *hex_num, float *list){
    int count = 0;
    for(int i = 0; i < 4; i++){
        if(hex_num[count] <= 85 && hex_num[count]  >= 71 ){
            list[i] = (float)( hex_decode(hex_num[count]) * 16 + hex_decode(hex_num[count + 1]) );    
        } else {
            list[i] = (float)( hex_decode(hex_num[count]) * 16 + hex_decode(hex_num[count + 1]) );    
        }
        count = count + 2;
    }
    return;
}

void wifi_connection()
{
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
            .ssid = SSID,
            .password = PASS}};
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_connect();
}

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        printf("WiFi connecting WIFI_EVENT_STA_START ... \n");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        printf("WiFi connected WIFI_EVENT_STA_CONNECTED ... \n");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        printf("WiFi lost connection WIFI_EVENT_STA_DISCONNECTED ... \n");
        break;
    case IP_EVENT_STA_GOT_IP:
        printf("WiFi got IP ... \n\n");
        break;
    default:
        break;
    }
}

void init_adc(struct MOTOR *m){

    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_RC_FAST,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    //Config the unit
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_config, &adc_handle));
    
    //Config the channel
    for(int i = 0; i < 4; i++){
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, m[i].adc_channel, &chan_config));
    }

    for(int i = 0; i < 4; i++){
        example_adc_calibration_init(ADC_UNIT_1, m[i].adc_channel, ADC_ATTEN_DB_12, &adc_cali_handle[i]);
    }


}

esp_err_t init_pwm(){

    ledc_channel_config_t channelConfig1 = {0};

    channelConfig1.gpio_num = PWM1;
    channelConfig1.speed_mode = LEDC_LOW_SPEED_MODE;
    channelConfig1.channel = LEDC_CHANNEL_0;
    channelConfig1.intr_type =  LEDC_INTR_DISABLE;
    channelConfig1.timer_sel = LEDC_TIMER_0;
    channelConfig1.duty = 0;

    ledc_channel_config_t channelConfig2 = {0};

    channelConfig2.gpio_num = PWM2;
    channelConfig2.speed_mode = LEDC_LOW_SPEED_MODE;
    channelConfig2.channel = LEDC_CHANNEL_1;
    channelConfig2.intr_type =  LEDC_INTR_DISABLE;
    channelConfig2.timer_sel = LEDC_TIMER_0;
    channelConfig2.duty = 0;

    ledc_channel_config_t channelConfig3 = {0};

    channelConfig3.gpio_num = PWM3;
    channelConfig3.speed_mode = LEDC_LOW_SPEED_MODE;
    channelConfig3.channel = LEDC_CHANNEL_2;
    channelConfig3.intr_type =  LEDC_INTR_DISABLE;
    channelConfig3.timer_sel = LEDC_TIMER_0;
    channelConfig3.duty = 0;

    ledc_channel_config_t channelConfig4 = {0};

    channelConfig4.gpio_num = PWM4;
    channelConfig4.speed_mode = LEDC_LOW_SPEED_MODE;
    channelConfig4.channel = LEDC_CHANNEL_3;
    channelConfig4.intr_type =  LEDC_INTR_DISABLE;
    channelConfig4.timer_sel = LEDC_TIMER_0;
    channelConfig4.duty = 0;
    

    ledc_channel_config(&channelConfig1);
    ledc_channel_config(&channelConfig2);
    ledc_channel_config(&channelConfig3);
    ledc_channel_config(&channelConfig4);
    

    ledc_timer_config_t timerConfig = {0};

    timerConfig.speed_mode = LEDC_LOW_SPEED_MODE;
    timerConfig.duty_resolution = LEDC_TIMER_10_BIT;
    timerConfig.timer_num = LEDC_TIMER_0;
    timerConfig.freq_hz = 20000; //20kHz

    ledc_timer_config(&timerConfig);

    return ESP_OK;
}

esp_err_t init_pin(uint8_t pin, uint8_t caso)
{

    if(caso == 0){
        gpio_config_t pGPIOConfig = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
        };

        pGPIOConfig.pin_bit_mask = (1ULL << pin);
        gpio_config(&pGPIOConfig);

    } else {
        gpio_config_t pGPIOConfig = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
        };

        pGPIOConfig.pin_bit_mask = (1ULL << pin);
        gpio_config(&pGPIOConfig);

    }

    return ESP_OK;

}

void init_high_precision_timers() {

    const esp_timer_create_args_t periodic_timer_args1 = {
        .callback = &PID_compute_callback,
        .name = "PID-compute1",
        .arg = &M[0]
    };

    const esp_timer_create_args_t periodic_timer_args2 = {
        .callback = &PID_compute_callback,
        .name = "PID-compute2",
        .arg = &M[1]
    };

    const esp_timer_create_args_t periodic_timer_args3 = {
        .callback = &PID_compute_callback,
        .name = "PID-compute3",
        .arg = &M[2]
    };

    const esp_timer_create_args_t periodic_timer_args4 = {
        .callback = &PID_compute_callback,
        .name = "PID-compute4",
        .arg = &M[3]
    };

    const esp_timer_create_args_t periodic_timer_args5 = {
        .callback = &ADCTimerCallback,
        .name = "ADC1",
        .arg = M
    };

    //Caculate PWM with PID and set the duty cycle
    // Create timer 1
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args1, &periodic_timers[0]));

    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timers[0], 2000)); // 2ms period

    // Create timer 2
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args2, &periodic_timers[1]));

    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timers[1], 2000)); // 2ms period

    // Create timer 3
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args3, &periodic_timers[2]));

    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timers[2], 2000)); // 2ms period

    // Create timer 4
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args4, &periodic_timers[3]));

    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timers[3], 2000)); // 2ms period


    //ADC timers to monitor the voltage reference of the current of the motors
    // Create timer 5
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args5, &periodic_timers[4]));

    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timers[4], 20000)); // 100ms period
    

}

esp_err_t init_isr(uint8_t* intr_pins, uint8_t dim){

    struct MOTOR *arg;

    gpio_config_t pGPIOConfig = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_DEF_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };

    gpio_install_isr_service(0);

    for (int i = 0; i < dim; i++) {
        pGPIOConfig.pin_bit_mask = (1ULL << intr_pins[i]);
        gpio_config(&pGPIOConfig);

        arg = &M[i];        //Pass the MOTOR struct as the argument that the callback will recieve
        gpio_isr_handler_add(intr_pins[i], isr_handler, arg);
    }
    

    return ESP_OK;

}

void isr_handler(void *args)
{

    //Get the Motor number to acces it's counter & speed
    struct MOTOR *m = ((struct MOTOR*)args);

    int64_t start = esp_timer_get_time(); // Get the current time in microseconds


    if (gpio_get_level(m->ENC_B)) {
        //m->pulses++;  // Counterclockwise
        m->deltaT = (start - m->prevTime); // Time difference
    } else {
        //m->pulses--;  // Clockwise
        m->deltaT = -(start - m->prevTime); // Time difference
    }

    m->prevTime = start;
    m->flag = true;


    
     
}

esp_err_t init_MOTOR(struct MOTOR *m, uint8_t MA, uint8_t MB, uint8_t ENCA, uint8_t ENCB, ledc_channel_t pwm_channel,  adc_channel_t adc_channel, adc_cali_handle_t adc_cal, struct PID_CONFIG *pid)
{
    m->t        = 0;
    m->current  = 0;
    m->speed    = 0;
    m->deltaT   = 0;
    m->duty     = 0;
    m->ENC_A    = ENCA;
    m->ENC_B    = ENCB;
    m->MINA     = MA;
    m->MINB     = MB;
    m->prevTime = 0;
    m->flag     = false;

    m->cali = adc_cal;
    m->PID  = pid;

    m->pwm_channel  = pwm_channel;
    m->adc_channel  = adc_channel;

    m->target   = 0;

    return ESP_OK;

}

esp_err_t init_PID(struct PID_CONFIG *PID){

    for(int i = 0; i < 4; i++){

        pid[i].Kp = PID->Kp;
        pid[i].Ki = PID->Ki;
        pid[i].Kd = PID->Kd;
        pid[i].MAX_PWM        = PWM_DEFINITION;
        pid[i].previous_err1  = 0; // e(k)
        pid[i].previous_err2  = 0; // e(k-1)
        pid[i].integral_err   = 0;// Sum of error
        pid[i].last_output    = 0;  // PID output in last control period
        pid[i].min_output     = -PWM_DEFINITION;   // PID minimum output limitation
        pid[i].max_integral   = PWM_DEFINITION - 700; // PID maximum integral value limitation
        pid[i].min_integral   = -PWM_DEFINITION + 700; // PID minimum integral value limitation

    }

    return ESP_OK;

}

esp_err_t init_EnablePins(uint8_t* pins)
{

    gpio_config_t pGPIOConfig = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };

    for (int i = 0; i < 8; i++) {
        pGPIOConfig.pin_bit_mask = (1ULL << pins[i]);
        gpio_config(&pGPIOConfig);
    }

    return ESP_OK;
}

esp_err_t set_pwm_duty(int duty, ledc_channel_t channel){

    if(duty < 0){
        duty = -duty;
    }


    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, (uint32_t)duty);

    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);

    return ESP_OK;
}

float pid_compute(struct MOTOR *M)
{
    float output;
    float dt = 0.002;
    float error  = M->target - M->speed; 

    /* Add current error to the integral error */
    M->PID->integral_err += error*dt;


    /* If the integral error is out of the range, it will be limited */
    M->PID->integral_err = MIN(M->PID->integral_err, M->PID->max_integral);
    M->PID->integral_err = MAX(M->PID->integral_err, M->PID->min_integral);

    /* Calculate the pid control value by location formula */
    /* u(k) = e(k)*Kp + (e(k)-e(k-1))*Kd + integral*Ki */
    output = error * M->PID->Kp +
             ( (error - M->PID->previous_err1) / dt ) * M->PID->Kd +
             M->PID->integral_err * M->PID->Ki;

    /* If the output is out of the range, it will be limited */
    output = MIN(output, M->PID->MAX_PWM);
    output = MAX(output, M->PID->min_output);

    if(output > 0){                  //Enable B so the motor goes clockwise
        gpio_set_level(M->MINA, 0);
        gpio_set_level(M->MINB, 1);
    } else if(output < 0){           //Enable A so the motor goes counterclockwise
        gpio_set_level(M->MINA, 1);
        gpio_set_level(M->MINB, 0);
    } else if(M->target == 0 && M->speed > 0){
        gpio_set_level(M->MINA, 1);
        gpio_set_level(M->MINB, 1);
    } else {                        //Disables bouth so it cuts the power to the motors
        gpio_set_level(M->MINA, 0);
        gpio_set_level(M->MINB, 0);
    }

    /* Update previous error */
    M->PID->previous_err1 = error;

    return output;

}

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}
