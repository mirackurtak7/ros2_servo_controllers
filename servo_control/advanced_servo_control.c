#include <stdio.h>
#include <unistd.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/timer.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rmw_microros/rmw_microros.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "pid_controller.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__, (int)temp_rc); return 1;}}

static const char *TAG = "advanced_servo_control";

// Servo motor pin tanımları
#define SERVO_LEFT_PIN    GPIO_NUM_18
#define SERVO_RIGHT_PIN   GPIO_NUM_19
#define SERVO_ARM_PIN     GPIO_NUM_21

// Encoder pin tanımları
#define ENCODER_LEFT_A    GPIO_NUM_22
#define ENCODER_LEFT_B    GPIO_NUM_23
#define ENCODER_RIGHT_A   GPIO_NUM_25
#define ENCODER_RIGHT_B   GPIO_NUM_26

// Servo parametreleri
#define SERVO_MIN_PULSEWIDTH    500
#define SERVO_MAX_PULSEWIDTH    2500
#define SERVO_MAX_DEGREE        180
#define SERVO_CENTER_ANGLE      90

// Robot parametreleri
#define WHEEL_DIAMETER          0.065
#define WHEEL_BASE              0.15
#define ENCODER_RESOLUTION      360

// PID parametreleri
#define PID_KP                  2.0
#define PID_KI                  0.1
#define PID_KD                  0.05
#define PID_OUTPUT_MIN          -30.0
#define PID_OUTPUT_MAX          30.0

// Global değişkenler
static int32_t left_encoder_count = 0;
static int32_t right_encoder_count = 0;
static double left_velocity = 0.0;
static double right_velocity = 0.0;
static double x_pos = 0.0;
static double y_pos = 0.0;
static double theta = 0.0;

// PID controller'lar
static pid_controller_t left_pid;
static pid_controller_t right_pid;
static pid_controller_t arm_pid;

// ROS 2 tanımları
rcl_publisher_t odom_publisher;
rcl_publisher_t velocity_publisher;
rcl_subscription_t cmd_vel_subscription;
rcl_subscription_t servo_control_subscription;
rcl_subscription_t pid_gains_subscription;
rcl_timer_t control_timer;

nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist velocity_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Int32 servo_msg;
std_msgs__msg__Float64 pid_gains_msg;

// Executor tanımları
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Servo motor kontrol fonksiyonu (PID ile)
void set_servo_angle_pid(int servo_pin, int channel, double angle)
{
    if (angle < 0) angle = 0;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;
    
    // PWM pulse width hesaplama
    uint32_t pulse_width = SERVO_MIN_PULSEWIDTH + (angle * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / SERVO_MAX_DEGREE);
    
    // LEDC ile servo kontrolü
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, pulse_width);
    
    ESP_LOGI(TAG, "Servo %d açısı: %.1f, pulse: %d", servo_pin, angle, pulse_width);
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
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ENCODER_LEFT_A) | (1ULL << ENCODER_RIGHT_A),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);
    
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
    
    // Sol servo
    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = SERVO_LEFT_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel);
    
    // Sağ servo
    ledc_channel.gpio_num = SERVO_RIGHT_PIN;
    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel_config(&ledc_channel);
    
    // Robot kolu servo
    ledc_channel.gpio_num = SERVO_ARM_PIN;
    ledc_channel.channel = LEDC_CHANNEL_2;
    ledc_channel_config(&ledc_channel);
    
    // Başlangıç pozisyonları
    set_servo_angle_pid(SERVO_LEFT_PIN, LEDC_CHANNEL_0, SERVO_CENTER_ANGLE);
    set_servo_angle_pid(SERVO_RIGHT_PIN, LEDC_CHANNEL_1, SERVO_CENTER_ANGLE);
    set_servo_angle_pid(SERVO_ARM_PIN, LEDC_CHANNEL_2, SERVO_CENTER_ANGLE);
}

// PID controller'ları başlatma
void init_pid_controllers(void)
{
    // Sol tekerlek PID
    pid_init(&left_pid, PID_KP, PID_KI, PID_KD, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    
    // Sağ tekerlek PID
    pid_init(&right_pid, PID_KP, PID_KI, PID_KD, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    
    // Robot kolu PID
    pid_init(&arm_pid, 1.0, 0.05, 0.01, -45.0, 45.0);
}

// Hız hesaplama
void calculate_velocities(void)
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
        left_velocity = (delta_left * M_PI * WHEEL_DIAMETER) / (ENCODER_RESOLUTION * dt / 1000.0);
        right_velocity = (delta_right * M_PI * WHEEL_DIAMETER) / (ENCODER_RESOLUTION * dt / 1000.0);
        
        // Önceki değerleri güncelle
        prev_left_count = left_encoder_count;
        prev_right_count = right_encoder_count;
        last_time = current_time;
    }
}

// Odometry hesaplama
void calculate_odometry(void)
{
    static double prev_left_velocity = 0.0;
    static double prev_right_velocity = 0.0;
    static uint32_t last_time = 0;
    
    uint32_t current_time = esp_timer_get_time() / 1000; // ms
    uint32_t dt = current_time - last_time;
    
    if (dt > 0) {
        // Robot hızları
        double linear_velocity = (left_velocity + right_velocity) / 2.0;
        double angular_velocity = (right_velocity - left_velocity) / WHEEL_BASE;
        
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
        prev_left_velocity = left_velocity;
        prev_right_velocity = right_velocity;
        last_time = current_time;
    }
}

// Cmd_vel callback fonksiyonu (PID ile)
void cmd_vel_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg_in = (const geometry_msgs__msg__Twist *) msgin;
    
    double target_linear = msg_in->linear.x;
    double target_angular = msg_in->angular.z;
    
    // Differential drive kinematiği
    double target_left_velocity = target_linear - (target_angular * WHEEL_BASE / 2.0);
    double target_right_velocity = target_linear + (target_angular * WHEEL_BASE / 2.0);
    
    // PID setpoint'lerini ayarla
    pid_set_setpoint(&left_pid, target_left_velocity);
    pid_set_setpoint(&right_pid, target_right_velocity);
    
    ESP_LOGI(TAG, "Target velocities: left=%.2f, right=%.2f", target_left_velocity, target_right_velocity);
}

// Servo control callback fonksiyonu
void servo_control_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg_in = (const std_msgs__msg__Int32 *) msgin;
    
    int target_angle = msg_in->data;
    if (target_angle >= 0 && target_angle <= 180) {
        pid_set_setpoint(&arm_pid, target_angle);
        ESP_LOGI(TAG, "Robot kolu hedef açısı: %d", target_angle);
    }
}

// PID gains callback fonksiyonu
void pid_gains_callback(const void * msgin)
{
    const std_msgs__msg__Float64 * msg_in = (const std_msgs__msg__Float64 *) msgin;
    
    // PID parametrelerini güncelle (basit mapping)
    double kp = msg_in->data;
    double ki = kp * 0.05;
    double kd = kp * 0.025;
    
    pid_set_gains(&left_pid, kp, ki, kd);
    pid_set_gains(&right_pid, kp, ki, kd);
    
    ESP_LOGI(TAG, "PID gains updated: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp, ki, kd);
}

// Control timer callback (PID kontrolü)
void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // Hızları hesapla
        calculate_velocities();
        
        // PID controller'ları güncelle
        double left_output = pid_update(&left_pid, left_velocity);
        double right_output = pid_update(&right_pid, right_velocity);
        
        // Servo açılarını hesapla
        double left_angle = SERVO_CENTER_ANGLE + left_output;
        double right_angle = SERVO_CENTER_ANGLE + right_output;
        
        // Servoları kontrol et
        set_servo_angle_pid(SERVO_LEFT_PIN, LEDC_CHANNEL_0, left_angle);
        set_servo_angle_pid(SERVO_RIGHT_PIN, LEDC_CHANNEL_1, right_angle);
        
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
        
        // Orientation
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
        odom_msg.pose.pose.orientation.w = cos(theta / 2.0);
        
        // Hız
        double linear_velocity = (left_velocity + right_velocity) / 2.0;
        double angular_velocity = (right_velocity - left_velocity) / WHEEL_BASE;
        
        odom_msg.twist.twist.linear.x = linear_velocity;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = angular_velocity;
        
        // Velocity mesajını hazırla
        velocity_msg.linear.x = linear_velocity;
        velocity_msg.linear.y = 0.0;
        velocity_msg.linear.z = 0.0;
        velocity_msg.angular.x = 0.0;
        velocity_msg.angular.y = 0.0;
        velocity_msg.angular.z = angular_velocity;
        
        // Mesajları yayınla
        rcl_publish(&odom_publisher, &odom_msg, NULL);
        rcl_publish(&velocity_publisher, &velocity_msg, NULL);
        
        ESP_LOGI(TAG, "Control: left_v=%.3f, right_v=%.3f, left_out=%.2f, right_out=%.2f", 
                 left_velocity, right_velocity, left_output, right_output);
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
    RCCHECK(rclc_node_init_default(&node, "esp32_advanced_servo", "", &support));

    // Publisher'lar oluştur
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"));

    RCCHECK(rclc_publisher_init_default(
        &velocity_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "velocity"));

    // Subscriber'lar oluştur
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    RCCHECK(rclc_subscription_init_default(
        &servo_control_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "servo_control"));

    RCCHECK(rclc_subscription_init_default(
        &pid_gains_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        "pid_gains"));

    // Timer oluştur (100Hz kontrol)
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(10), // 100Hz
        control_timer_callback));

    // Executor oluştur
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscription, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &servo_control_subscription, &servo_msg, &servo_control_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &pid_gains_subscription, &pid_gains_msg, &pid_gains_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // Hardware başlat
    init_servos();
    init_encoders();
    init_pid_controllers();

    ESP_LOGI(TAG, "Micro-ROS Advanced Servo Control Node başlatıldı!");

    // Ana döngü
    do {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        usleep(5000);
    } while (true);

    // Temizlik
    RCCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCCHECK(rcl_publisher_fini(&velocity_publisher, &node));
    RCCHECK(rcl_subscription_fini(&cmd_vel_subscription, &node));
    RCCHECK(rcl_subscription_fini(&servo_control_subscription, &node));
    RCCHECK(rcl_subscription_fini(&pid_gains_subscription, &node));
    RCCHECK(rcl_timer_fini(&control_timer));
    RCCHECK(rcl_node_fini(&node));
}

void app_main(void)
{
    // Micro-ROS task'ını başlat
    xTaskCreate(micro_ros_task, "micro_ros_task", 8192, NULL, 5, NULL);
} 