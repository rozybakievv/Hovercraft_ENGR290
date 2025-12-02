#include <Arduino.h>
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

// ---------------- Pin Defines ---------------- //
#define LIFT_FAN PD4
#define FAN_THRUST_PIN PD5  // OC0B

#define TRIG_PIN_LEFT PB3
#define ECHO_PIN_LEFT PD2   // INT0
#define TRIG_PIN_RIGHT PB5
#define ECHO_PIN_RIGHT PD3  // INT1

#define SERVO_PIN PB1       // OC1A

#define IR_PIN 0 // ADC0 (PC0)

#define IMU_ADDR 0x68

float TURN_ANGLE = 60.0f;
double TURN_DISTANCE = 40.0;

typedef enum
{
    FORWARD,
    TURNING,
    STOPPING
} SYS_STATE;

typedef enum
{
    STRAIGHT,
    LEFT,
    RIGHT
} TURN_DIRECTION;

float system_yaw = 0;

#define FORWARD_TIMEOUT_MS 2000  // 2 seconds
#define DISTANCE_CHANGE_THRESHOLD 10

// ---------------- IMU Variables ---------------- //
float gyro_scale = 65.5f;
float acc_scale = 8192.0f;

int16_t AccX, AccY, AccZ, GyrZ;
float accX, accY, accZ, gyrZ;
float yaw = 0.0f;
float offset_ax, offset_ay, offset_az, offset_gz;

unsigned long last_imu_time = 0;

// Ultrasonic sensor variables
volatile uint32_t pulse_width_left = 0;
volatile uint32_t pulse_width_right = 0;
volatile uint32_t echo_start_left = 0;
volatile uint32_t echo_start_right = 0;

uint32_t front_distance = 0;
uint32_t right_distance = 0;

uint16_t ir_distance = 0;

// Timing variables
volatile unsigned long milliseconds = 0;
volatile unsigned long timer0_overflow_count = 0;

// ---------------- UART Functions ---------------- //
void uartTransmit(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void uartPrint(const char* str) {
    while (*str) {
        uartTransmit(*str++);
    }
}

void uartPrintInt(uint32_t num) {
    char buffer[12];
    ltoa(num, buffer, 10);
    uartPrint(buffer);
}

void uartPrintFloat(float num) {
    char buffer[20];
    dtostrf(num, 6, 2, buffer);
    uartPrint(buffer);
}

// ---------------- Timing Functions ---------------- //
// Timer0 overflow interrupt (for micros())
ISR(TIMER0_OVF_vect) {
    timer0_overflow_count++;
}

// Timer2 interrupt (for millis())
ISR(TIMER2_COMPA_vect) {
    milliseconds++;
}

unsigned long millis() {
    unsigned long m;
    cli();
    m = milliseconds;
    sei();
    return m;
}

unsigned long micros() {
    unsigned long m;
    uint8_t t;
    
    cli();
    m = timer0_overflow_count;
    t = TCNT0;
    
    // Check if overflow flag is set
    if ((TIFR0 & (1 << TOV0)) && (t < 255)) {
        m++;
    }
    sei();
    
    // Timer0 with prescaler 64: each tick = 4us
    // Overflow every 256 ticks = 1024us
    return (m * 1024UL) + (t * 4UL);
}

// ---------------- I2C Functions ---------------- //
void i2c_init(void) {
    TWSR = 0x00;
    TWBR = 72;  // 100kHz I2C for better stability
    TWCR = (1 << TWEN);  // Enable I2C
}

void i2c_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}


void i2c_stop(void) {
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    _delay_us(100);
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t i2c_read_ack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t i2c_read_nack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

// ---------------- ADC Functions ---------------- //
void ADC_init() {
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t readADC(uint8_t channel) {
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADCW;
}

uint16_t readIR() {
    uint16_t sensorValue = readADC(IR_PIN);
    if (sensorValue > 3) {
        float distance = (6787.0 / (sensorValue - 3.0)) - 4.0;
        if (distance >= 0 && distance <= 200) {
            return (uint16_t)distance;
        }
    }
    return 9999;
}

// ---------------- US Sensor Functions ---------------- //
ISR(INT0_vect) {
    if (PIND & (1 << ECHO_PIN_LEFT)) {
        // Rising edge - start timing
        echo_start_left = micros();
    } else {
        // Falling edge - calculate pulse width
        if (echo_start_left > 0) {
            uint32_t end_time = micros();
            if (end_time >= echo_start_left) {
                pulse_width_left = end_time - echo_start_left;
            } else {
                // Handle overflow (rare case)
                pulse_width_left = (0xFFFFFFFF - echo_start_left) + end_time;
            }
            echo_start_left = 0;
        }
    }
}

ISR(INT1_vect) {
    if (PIND & (1 << ECHO_PIN_RIGHT)) {
        // Rising edge - start timing
        echo_start_right = micros();
    } else {
        // Falling edge - calculate pulse width
        if (echo_start_right > 0) {
            uint32_t end_time = micros();
            if (end_time >= echo_start_right) {
                pulse_width_right = end_time - echo_start_right;
            } else {
                // Handle overflow (rare case)
                pulse_width_right = (0xFFFFFFFF - echo_start_right) + end_time;
            }
            echo_start_right = 0;
        }
    }
}

void triggerUS_Left() {
    PORTB &= ~(1 << TRIG_PIN_LEFT);
    _delay_us(2);
    PORTB |= (1 << TRIG_PIN_LEFT);
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN_LEFT);
}

void triggerUS_Right() {
    PORTB &= ~(1 << TRIG_PIN_RIGHT);
    _delay_us(2);
    PORTB |= (1 << TRIG_PIN_RIGHT);
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN_RIGHT);
}

uint32_t getDistance_Left() {
    if (pulse_width_left == 0 || pulse_width_left > 30000) {
        return 9999;
    }
    // Distance in cm = pulse_width_us / 58
    return pulse_width_left / 58;
}

uint32_t getDistance_Right() {
    if (pulse_width_right == 0 || pulse_width_right > 30000) {
        return 9999;
    }
    // Distance in cm = pulse_width_us / 58
    return pulse_width_right / 58;
}

// ---------------- Servo Control ---------------- //
void servo_init() {
    // Configure Timer1 for Fast PWM mode with ICR1 as TOP
    // This gives us precise control over the PWM frequency and pulse width
    
    DDRB |= (1 << SERVO_PIN);  // PB1 as output
    
    // Fast PWM mode, TOP = ICR1
    TCCR1A = (1 << COM1A1) | (1 << WGM11);  // Clear OC1A on compare match, Fast PWM mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // Fast PWM, prescaler = 8
    
    // Set TOP value for 20ms period (50Hz)
    // With 16MHz clock and prescaler 8: 16MHz / 8 = 2MHz = 0.5µs per tick
    // For 20ms: 20000µs / 0.5µs = 40000 ticks
    ICR1 = 40000;
    
    // Set initial position to center (1500µs)
    // 1500µs / 0.5µs = 3000 ticks
    OCR1A = 3000;
}

void set_servo_angle(float angle) {
    if (angle > 180) angle = 180;
    if (angle < -180) angle = -180;

    float pulse_us = 1500.0f + (angle * (500.0f / 90.0f));
    uint16_t ticks = (uint16_t)(pulse_us * 2.0f);
    
    OCR1A = ticks;
}

// ---------------- Fan Control ---------------- //
void startLiftFan() {
    PORTD |= (1 << LIFT_FAN);
}

void stopLiftFan() {
    PORTD &= ~(1 << LIFT_FAN);
}

void setThrustFan(uint8_t speed) {
    OCR0B = speed;
}

// ---------------- IMU Functions ---------------- //
void mpu6050_init(void) {
    _delay_ms(100);  // Wait for MPU6050 to power up
    
    // Wake up MPU6050 (clear sleep bit)
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x6B);  // PWR_MGMT_1 register
    i2c_write(0x00);  // Clear sleep bit
    i2c_stop();
    _delay_ms(100);

    // Set gyroscope range to ±500°/s
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x1B);  // GYRO_CONFIG register
    i2c_write(0x08);  // ±500°/s (gyro_scale = 65.5)
    i2c_stop();
    _delay_ms(10);

    // Set accelerometer range to ±4g
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x1C);  // ACCEL_CONFIG register
    i2c_write(0x08);  // ±4g (acc_scale = 8192)
    i2c_stop();
    _delay_ms(10);
    
    uartPrint("MPU6050 initialized successfully!\r\n\r\n");
}

void readIMURaw() {
    uint8_t data[14];

    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x3B);  // Start reading from ACCEL_XOUT_H
    i2c_stop();
    
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 1);  // Read mode
    for (uint8_t i = 0; i < 13; i++) {
        data[i] = i2c_read_ack();
    }
    data[13] = i2c_read_nack();
    i2c_stop();

    AccX = ((int16_t)data[0] << 8) | data[1];
    AccY = ((int16_t)data[2] << 8) | data[3];
    AccZ = ((int16_t)data[4] << 8) | data[5];
    GyrZ = ((int16_t)data[12] << 8) | data[13];

    accX = (float)AccX / acc_scale;
    accY = (float)AccY / acc_scale;
    accZ = (float)AccZ / acc_scale;
    gyrZ = (float)GyrZ / gyro_scale;
}

void calibrateIMU() {
    uartPrint("=== Calibrating IMU ===\r\n");
    
    float sum_ax = 0, sum_ay = 0, sum_az = 0, sum_gz = 0;
    
    // First 100 readings are discarded
    for (int i = 0; i < 100; i++) {
        readIMURaw();
        _delay_ms(5);
    }
    
    for (int i = 0; i < 200; i++) {
        readIMURaw();
        sum_ax += accX;
        sum_ay += accY;
        sum_az += accZ;
        sum_gz += gyrZ;
        
        _delay_ms(5);
    }
    
    offset_ax = sum_ax / 200.0f;
    offset_ay = sum_ay / 200.0f;
    offset_az = (sum_az / 200.0f) - 1.0f;  // Subtract 1g
    offset_gz = sum_gz / 200.0f;
    
    uartPrint("\r\nCalibration complete!\r\n");
}

void readIMU() {
    readIMURaw();
    accX -= offset_ax;
    accY -= offset_ay;
    accZ -= offset_az;
    gyrZ -= offset_gz;
}

// ---------------- Setup ---------------- //
void setup() {
    // UART - 9600 baud
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    // ===== LIFT FAN (PD4) - Digital output ===== //
    DDRD |= (1 << LIFT_FAN);
    PORTD &= ~(1 << LIFT_FAN);

    // ===== THRUST FAN (PD5 / OC0B) - Timer0 Fast PWM ===== //
    DDRD |= (1 << FAN_THRUST_PIN);
    TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);  // Fast PWM on OC0B, non-inverting
    TCCR0B = (1 << CS01) | (1 << CS00);  // Prescaler 64 (for PWM and micros())
    OCR0B = 0;  // CHANGED: Use OCR0B
    TIMSK0 = (1 << TOIE0);

    // ===== SERVO - Initialize with hardware PWM ===== //
    servo_init();

    // ===== Timer2 - 1ms interrupt for millis() ===== //
    TCCR2A = (1 << WGM21);  // CTC mode
    TCCR2B = (1 << CS22) | (1 << CS20);  // Prescaler 128
    OCR2A = 124;  // 1ms
    TIMSK2 = (1 << OCIE2A);

    // ===== Ultrasonic sensors ===== //
    DDRB |= (1 << TRIG_PIN_LEFT) | (1 << TRIG_PIN_RIGHT);
    PORTB &= ~((1 << TRIG_PIN_LEFT) | (1 << TRIG_PIN_RIGHT));

    DDRD &= ~((1 << ECHO_PIN_LEFT) | (1 << ECHO_PIN_RIGHT));

    EICRA = (1 << ISC00) | (1 << ISC10);  // Any edge on INT0 and INT1
    EIMSK = (1 << INT0) | (1 << INT1);    // Enable INT0 and INT1

    sei();
}

float normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

float angleDifference(float target, float current) {
    float diff = target - current;
    return normalizeAngle(diff);
}

float target_yaw = 0.0f;
unsigned long turn_start_time = 0;
bool forward_timeout_active = 0;
unsigned long forward_start_time = 0;
bool stuck = false;
float turn_start_yaw = 0.0f;

int main() {
    setup();
    _delay_ms(100);
    i2c_init();
    _delay_ms(50);
    ADC_init();
    mpu6050_init();
    _delay_ms(100);
    calibrateIMU();
    
    uartPrint("System Initialized!\r\n");
    
    unsigned long last_print = 0;
    unsigned long last_us = 0;

    SYS_STATE system_state = FORWARD;
    TURN_DIRECTION turning_state = STRAIGHT;
    int turning_counter = 0;

    startLiftFan();

    while (1) {
        // ---------------- Read Value ---------------- //
        unsigned long now = millis();
        
        // Read IMU sensors
        if (now - last_imu_time >= 10) {
            readIMU();
            yaw += gyrZ * 0.01;
            yaw = normalizeAngle(yaw);
            last_imu_time = now;
        }
        
        // Read ultrasonic sensors alternately
        if (now - last_us >= 60) {
            static uint8_t us_toggle = 0;
            if (us_toggle == 0) {
                triggerUS_Left();
                _delay_ms(30);
                front_distance = getDistance_Left();
                pulse_width_left = 0;
            } else {
                triggerUS_Right();
                _delay_ms(30);
                right_distance = getDistance_Right();
                pulse_width_right = 0;
            }
            us_toggle = !us_toggle;
            last_us = now;
        }
        
        // Read IR sensor
        ir_distance = readIR();
        
        // ---------------- Debugging ---------------- //
        if (now - last_print >= 500) {
            uartPrint("L:");
            uartPrintInt(front_distance);
            uartPrint("cm R:");
            uartPrintInt(right_distance);
            uartPrint("cm IR:");
            uartPrintInt(ir_distance);
            uartPrint("cm | Yaw:");
            uartPrintFloat(yaw);
            uartPrint("| Acc X:");
            uartPrintFloat(accX);
            uartPrint("| Acc Y:");
            uartPrintFloat(accY);
            uartPrint("\r\n");
            
            last_print = now;
        }

        // ---------------- Algorithm ---------------- //
        switch(system_state)
        {
            case FORWARD: {// Forward state
            setThrustFan(255);

            float yaw_error = angleDifference(system_yaw, yaw);
            float servo_correction = -yaw_error * 2.5f;

            // Clamp servo correction
            if (servo_correction > 90) servo_correction = 90;
            if (servo_correction < -90) servo_correction = -90;
            set_servo_angle(servo_correction);

            // Check if stuck
            if (front_distance < 10 && front_distance != 9999) {
                if (!forward_timeout_active) {
                    forward_timeout_active = true;
                    forward_start_time = now;
                } else if (now - forward_start_time >= FORWARD_TIMEOUT_MS) {
                    stuck = true;

                    // Use last turn direction if available, otherwise decide based on right distance
                    if (right_distance >= 10 && right_distance != 9999) {
                        system_yaw = yaw;
                        target_yaw = normalizeAngle(system_yaw - (TURN_ANGLE-45));  // MODIFIED: -15° from start
                        system_state = TURNING;
                        turning_state = RIGHT;
                        turn_start_time = now;
                        turning_counter = 0;
                    } else if (right_distance < 10 && right_distance != 9999){
                        system_yaw = yaw;
                        target_yaw = normalizeAngle(system_yaw + (TURN_ANGLE-45));
                        system_state = TURNING;
                        turning_state = LEFT;
                        turn_start_time = now;
                        turning_counter = 0;
                    }

                    forward_timeout_active = false;
                }
            } else {
                forward_timeout_active = false;
            }
        
            // Change to TURNING state when wall detected in front
            if ((front_distance < TURN_DISTANCE) && (front_distance > 10.0) && (front_distance != 9999))
            {
                if (turning_counter < 5) turning_counter++;
                
                if (turning_counter >= 5)
                {
                    if ((right_distance >= 50) && (right_distance != 9999)) // Turn right
                    {
                        system_yaw = yaw;
                        target_yaw = normalizeAngle(yaw - TURN_ANGLE);
                        system_state = TURNING;
                        turning_state = RIGHT;
                        turn_start_time = now;
                        turning_counter = 0;
                    } else  if (right_distance < 50 && right_distance != 9999){
                        system_yaw = yaw;
                        target_yaw = normalizeAngle(yaw + TURN_ANGLE);
                        system_state = TURNING;
                        turning_state = LEFT;
                        turn_start_time = now;
                        turning_counter = 0;
                    }
                }
            }
            
            break; }
        
        case TURNING: { // Turning state            
            // Define an acceptable margin of error (e.g., within 3 degrees)
            const float TURN_COMPLETE_THRESHOLD = 1.0f; 

            // Calculate the remaining angle to turn
            float remaining_angle = angleDifference(target_yaw, yaw);
            
            // Check if the turn is complete or if a timeout occurred
            if ((fabs(remaining_angle) <= TURN_COMPLETE_THRESHOLD) || (now - turn_start_time > 3000))
            {
                // Turn is complete or timed out (max 3 seconds)
                set_servo_angle(0);
                system_yaw = yaw; // Update system_yaw to the new orientation
                turning_state = STRAIGHT;
                system_state = FORWARD;
                stuck = false;
                break;
            }

            if (turning_state == LEFT)
            {
                if (stuck == true)
                {
                    set_servo_angle(-180); 
                } else {
                    set_servo_angle(90); 
                }
                
                setThrustFan(200);
            } 
            else if (turning_state == RIGHT) 
            {
                setThrustFan(200);
                if (stuck == true)
                {
                    set_servo_angle(180);
                } else {
                    set_servo_angle(90);
                }
            }
            else
            {
                // Should not happen if logic in FORWARD state is correct
                system_state = FORWARD;
                turning_state = STRAIGHT;
            }
            
            break; }
        
            case STOPPING: { // Stopping state
                set_servo_angle(0.0);
                setThrustFan(0);
                stopLiftFan();
                break; }
            
            default:
                break;
        }
    }
    return 0;
}