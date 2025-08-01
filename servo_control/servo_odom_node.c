#include <stdio.h>
#include <unistd.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/timer.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rmw_microros/rmw_microros.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "driver/uart.h"
#include "driver/ledc.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__, (int)temp_rc); return 1;}}

static const char *TAG = "servo_odom_node";

// Servo motor pin tanımları
#define SERVO_LEFT_PIN    GPIO_NUM_18   // Sol servo motor
#define SERVO_RIGHT_PIN   GPIO_NUM_19   // Sağ servo motor
#define SERVO_ARM_PIN     GPIO_NUM_21   // Robot kolu servo

// Encoder pin tanımları (opsiyonel)
#define ENCODER_LEFT_A    GPIO_NUM_22
#define ENCODER_LEFT_B    GPIO_NUM_23
#define ENCODER_RIGHT_A   GPIO_NUM_25
#define ENCODER_RIGHT_B   GPIO_NUM_26

// Servo motor parametreleri
#define SERVO_MIN_PULSEWIDTH    500     // 0.5ms
#define SERVO_MAX_PULSEWIDTH    2500    // 2.5ms
#define SERVO_MAX_DEGREE        180     // Maksimum açı

// Odometry parametreleri
#define WHEEL_DIAMETER          0.065   // Tekerlek çapı (metre)
#define WHEEL_BASE              0.15    // Tekerlekler arası mesafe (metre)
#define ENCODER_RESOLUTION      360     // Encoder çözünürlüğü (pulse/tur)

// Global değişkenler
static int32_t left_encoder_count = 0;
static int32_t right_encoder_count = 0;
static double x_pos = 0.0;
static double y_pos = 0.0;
static double theta = 0.0;
static double linear_velocity = 0.0;
static double angular_velocity = 0.0;

// ROS 2 tanımları
rcl_publisher_t odom_publisher;
rcl_subscription_t cmd_vel_subscription;
rcl_subscription_t servo_control_subscription;
rcl_timer_t odom_timer;

nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Int32 servo_msg;

// Executor tanımları
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Servo motor kontrol fonksiyonu
void set_servo_angle(int servo_pin, int angle)
{
    if (angle < 0) angle = 0;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;
    
    // PWM pulse width hesaplama
    uint32_t pulse_width = SERVO_MIN_PULSEWIDTH + (angle * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / SERVO_MAX_DEGREE);
    
    // LEDC ile servo kontrolü
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, pulse_width);
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, 50); // 50Hz
    
    ESP_LOGI(TAG, "Servo %d açısı: %d, pulse: %d", servo_pin, angle, pulse_width);
}

// Encoder interrupt handler
static void IRAM_ATTR encoder_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    if (gpio_num == ENCODER_LEFT_A) {
        left_encoder_count++;
    } else if (gpio_num == ENCODER_RIGHT_A) {
        right_encoder_count++;
    }
}

// Encoder başlatma
void init_encoders(void)
{
    // Encoder pinlerini giriş olarak ayarla
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ENCODER_LEFT_A) | (1ULL << ENCODER_RIGHT_A),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);
    
    // Interrupt handler'ları ayarla
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_LEFT_A, encoder_isr_handler, (void*) ENCODER_LEFT_A);
    gpio_isr_handler_add(ENCODER_RIGHT_A, encoder_isr_handler, (void*) ENCODER_RIGHT_A);
}

// Servo motor başlatma
void init_servos(void)
{
    // LEDC timer konfigürasyonu
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 50,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);
    
    // LEDC channel konfigürasyonu
    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = SERVO_LEFT_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel);
    
    // İkinci servo için
    ledc_channel.gpio_num = SERVO_RIGHT_PIN;
    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel_config(&ledc_channel);
    
    // Robot kolu servo için
    ledc_channel.gpio_num = SERVO_ARM_PIN;
    ledc_channel.channel = LEDC_CHANNEL_2;
    ledc_channel_config(&ledc_channel);
    
    // Başlangıç pozisyonları
    set_servo_angle(SERVO_LEFT_PIN, 90);
    set_servo_angle(SERVO_RIGHT_PIN, 90);
    set_servo_angle(SERVO_ARM_PIN, 90);
}

// Odometry hesaplama
void calculate_odometry(void)
{
    static int32_t prev_left_count = 0;
    static int32_t prev_right_count = 0;
    static uint32_t last_time = 0;
    
    uint32_t current_time = esp_timer_get_time() / 1000; // ms
    uint32_t dt = current_time - last_time;
    
    if (dt > 0) {
        // Encoder değişimlerini hesapla
        int32_t delta_left = left_encoder_count - prev_left_count;
        int32_t delta_right = right_encoder_count - prev_right_count;
        
        // Tekerlek hızları (m/s)
        double left_velocity = (delta_left * M_PI * WHEEL_DIAMETER) / (ENCODER_RESOLUTION * dt / 1000.0);
        double right_velocity = (delta_right * M_PI * WHEEL_DIAMETER) / (ENCODER_RESOLUTION * dt / 1000.0);
        
        // Robot hızları
        linear_velocity = (left_velocity + right_velocity) / 2.0;
        angular_velocity = (right_velocity - left_velocity) / WHEEL_BASE;
        
        // Pozisyon güncelleme
        double delta_theta = angular_velocity * (dt / 1000.0);
        theta += delta_theta;
        
        // Normalize açı
        while (theta > M_PI) theta -= 2 * M_PI;
        while (theta < -M_PI) theta += 2 * M_PI;
        
        // Pozisyon güncelleme
        double delta_x = linear_velocity * cos(theta) * (dt / 1000.0);
        double delta_y = linear_velocity * sin(theta) * (dt / 1000.0);
        
        x_pos += delta_x;
        y_pos += delta_y;
        
        // Önceki değerleri güncelle
        prev_left_count = left_encoder_count;
        prev_right_count = right_encoder_count;
        last_time = current_time;
    }
}

// Cmd_vel callback fonksiyonu
void cmd_vel_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg_in = (const geometry_msgs__msg__Twist *) msgin;
    
    double linear = msg_in->linear.x;
    double angular = msg_in->angular.z;
    
    // Differential drive kinematiği
    double left_velocity = linear - (angular * WHEEL_BASE / 2.0);
    double right_velocity = linear + (angular * WHEEL_BASE / 2.0);
    
    // Servo açılarını hesapla (basit mapping)
    int left_angle = 90 + (int)(left_velocity * 30);  // ±30 derece
    int right_angle = 90 + (int)(right_velocity * 30);
    
    // Servo açılarını sınırla
    if (left_angle < 60) left_angle = 60;
    if (left_angle > 120) left_angle = 120;
    if (right_angle < 60) right_angle = 60;
    if (right_angle > 120) right_angle = 120;
    
    // Servoları kontrol et
    set_servo_angle(SERVO_LEFT_PIN, left_angle);
    set_servo_angle(SERVO_RIGHT_PIN, right_angle);
    
    ESP_LOGI(TAG, "Cmd_vel: linear=%.2f, angular=%.2f, left_angle=%d, right_angle=%d", 
             linear, angular, left_angle, right_angle);
}

// Servo control callback fonksiyonu
void servo_control_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg_in = (const std_msgs__msg__Int32 *) msgin;
    
    // Basit servo kontrolü (0-180 arası açı)
    int angle = msg_in->data;
    if (angle >= 0 && angle <= 180) {
        set_servo_angle(SERVO_ARM_PIN, angle);
        ESP_LOGI(TAG, "Robot kolu servo açısı: %d", angle);
    }
}

// Odometry timer callback
void odom_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // Odometry hesapla
        calculate_odometry();
        
        // Odometry mesajını hazırla
        odom_msg.header.stamp.sec = esp_timer_get_time() / 1000000;
        odom_msg.header.stamp.nanosec = (esp_timer_get_time() % 1000000) * 1000;
        odom_msg.header.frame_id.data = "odom";
        odom_msg.child_frame_id.data = "base_link";
        
        // Pozisyon
        odom_msg.pose.pose.position.x = x_pos;
        odom_msg.pose.pose.position.y = y_pos;
        odom_msg.pose.pose.position.z = 0.0;
        
        // Orientation (quaternion)
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
        odom_msg.pose.pose.orientation.w = cos(theta / 2.0);
        
        // Hız
        odom_msg.twist.twist.linear.x = linear_velocity;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = angular_velocity;
        
        // Mesajı yayınla
        rcl_publish(&odom_publisher, &odom_msg, NULL);
        
        ESP_LOGI(TAG, "Odom: x=%.3f, y=%.3f, theta=%.3f, v=%.3f, w=%.3f", 
                 x_pos, y_pos, theta, linear_velocity, angular_velocity);
    }
}

void micro_ros_task(void * arg)
{
    // Micro-ROS desteğini başlat
    rmw_uros_set_custom_transport(
        true,
        (void *) &arg,
        cubemicroros_serial_transport_open,
        cubemicroros_serial_transport_close,
        cubemicroros_serial_transport_write,
        cubemicroros_serial_transport_read
    );

    // ROS 2 desteğini başlat
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Node oluştur
    RCCHECK(rclc_node_init_default(&node, "esp32_servo_odom", "", &support));

    // Publisher oluştur (Odometry)
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"));

    // Subscriber oluştur (Cmd_vel)
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // Subscriber oluştur (Servo control)
    RCCHECK(rclc_subscription_init_default(
        &servo_control_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "servo_control"));

    // Timer oluştur (50Hz odometry)
    RCCHECK(rclc_timer_init_default(
        &odom_timer,
        &support,
        RCL_MS_TO_NS(20), // 50Hz
        odom_timer_callback));

    // Executor oluştur
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscription, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &servo_control_subscription, &servo_msg, &servo_control_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));

    // Hardware başlat
    init_servos();
    init_encoders();

    ESP_LOGI(TAG, "Micro-ROS Servo Odom Node başlatıldı!");

    // Ana döngü
    do {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        usleep(10000);
    } while (true);

    // Temizlik
    RCCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCCHECK(rcl_subscription_fini(&cmd_vel_subscription, &node));
    RCCHECK(rcl_subscription_fini(&servo_control_subscription, &node));
    RCCHECK(rcl_timer_fini(&odom_timer));
    RCCHECK(rcl_node_fini(&node));
}

void app_main(void)
{
    // Micro-ROS task'ını başlat
    xTaskCreate(micro_ros_task, "micro_ros_task", 8192, NULL, 5, NULL);
} 