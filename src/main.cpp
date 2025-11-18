#include <Arduino.h>
#define F_CPU 16000000UL // 16 MHz clock
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define LIFT_FAN PD4
#define FAN_THRUST_PIN PD6

#define TRIG_PIN_LEFT PB3  // Trigger pin
#define ECHO_PIN_LEFT PD2  // Echo pin (PD2) - INT0
#define TRIG_PIN_RIGHT PB5  // Trigger pin
#define ECHO_PIN_RIGHT PD3  // Echo pin (PD2) - INT0

#define SERVO_PIN PB1 // for OC1A

#define IR_PIN 0  // ADC channel for IR sensor (A0 / PC0)

#define IMU_ADDR 0x68

// Current gyroscope range setting
float gyro_scale = 131.0f;
float acc_scale = 16384.0f;

int16_t AccX, AccY, AccZ, GyrZ; // raw
float accX, accY, accZ, gyrZ; // converted to g or deg
float yaw = 0.0f;
float pitch = 0.0f;
float roll = 0.0f;
float velocityX = 0.0f;   // in m/s
float offset_ax, offset_ay, offset_az, offset_gz;

void setup() {
    // Init UART
    UBRR0H = 0;
    UBRR0L = 103;  // 9600 baud
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

	// Timer1
    // Configure Timer1 for servo control (50Hz PWM for servo)
    TCCR1A = (1 << COM1A1) | (1 << WGM11);  // Non-inverting mode on OC1A
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // Prescaler 8

    // Timer2 init in CTC mode, interrupt every 1ms -> schedule for sensors
    TCCR2A = (1 << WGM21);  // CTC mode
    TCCR2B = (1 << CS22) | (1 << CS20);  // Prescaler 128
    OCR2A = 124;  // (16MHz / (128 * 125)) = 1000Hz = 1ms
    TIMSK2 = (1 << OCIE2A);  // Enable compare match interrupt

    // PD4 as output (Lift Fan)
    DDRD |= (1 << PD4);
    // Start with lift fan OFF
    PORTD &= ~(1 << PD4);

    // PD6 as output (Thrust Fan - OC0A)
    DDRD |= (1 << PD6);

    // Configure Timer0 for Fast PWM on OC0A (PD6) -> Thrust fan
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);  // Fast PWM, non-inverting
    TCCR0B = (1 << CS01) | (1 << CS00);  // Prescaler 64 → ~490Hz
    OCR0A = 0;  // Start at 0

    // Configure TRIG_PIN as OUTPUT
    DDRB |= (1 << TRIG_PIN_LEFT);
    PORTB &= ~(1 << TRIG_PIN_LEFT);

    // Configure ECHO_PIN as INPUT
    DDRD &= ~(1 << ECHO_PIN_LEFT);

    // Configure TRIG_PIN_RIGHT as OUTPUT
    DDRB |= (1 << TRIG_PIN_RIGHT);
    PORTB &= ~(1 << TRIG_PIN_RIGHT);

    // Configure ECHO_PIN_RIGHT as INPUT
    DDRD &= ~(1 << ECHO_PIN_RIGHT);
    
    // Configure INT1 interrupt for rising edge (RIGHT sensor)
    EICRA |= (1 << ISC11) | (1 << ISC10);  // Rising edge on INT1
    EIMSK |= (1 << INT1);  // Enable INT1

    // Configure INT0 interrupt for rising edge
    // External Interrupt Control Register A (EICRA) -> Controls when the interrupt triggers (rising edge)
    EICRA |= (1 << ISC01) | (1 << ISC00);  // Rising edge
    // External Interrupt Mask Register (EIMSK) -> Enables specific interrupt (PD2 -> INT0)
    EIMSK |= (1 << INT0);  // Enable INT0

    // PB1 -> output for servo
    DDRB |= (1 << SERVO_PIN);

    // Set TOP value for 50Hz: (16MHz / (8 * 50Hz)) - 1 = 39999
    ICR1 = 39999;
    
    // Initialize servo to center position (1.5ms pulse)
    OCR1A = 3000;  // Center position
    
    // Enable global interrupts
    sei();
}

// Timer2 System Scheduler Interrupt setup
ISR(TIMER2_COMPA_vect) {
    system_millis++;  // System timing tracker
    
    // Schedule IMU reading every 5ms
    if (system_millis % IMU_UPDATE_INTERVAL == 0) {
        imu_ready = 1;
    }
    
    // Schedule LEFT US TRIGGER every 50ms
    if (system_millis % US_UPDATE_INTERVAL == 0) {
        us_left_trigger = 1;
    }
    
    // Schedule RIGHT US TRIGGER every 50ms with offset by 25ms
    if (system_millis % US_UPDATE_INTERVAL == 25) {
        us_right_trigger = 1;
    }
}

// ---------------- UART Functions ---------------- //
void uartTransmit(char c) {
    while (!(UCSR0A & (1 << UDRE0))); // Wait until buffer is empty
    UDR0 = c;
}

void uartPrint(const char* str) {
  while (*str) {
      uartTransmit(*str++);
  }
}

void uartPrintInt(uint32_t num) {
  char buffer[10];
  itoa(num, buffer, 10);
  uartPrint(buffer);
}

void uartPrintFloat(float num) {
    char buffer[20];
    dtostrf(num, 6, 2, buffer); // width=6, precision=2
    uartPrint(buffer);
}

// ---------------- I2C Functions ---------------- //
void i2c_init(void) {
    TWSR = 0x00;  // prescaler = 1
    TWBR = 12;    // ~400kHz @ 16MHz
}

void i2c_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void i2c_stop(void) {
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    _delay_us(100);  // Small delay for stop condition
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

// ---------------- IR Sensor Functions ---------------- //
void ADC_init() {
    ADMUX = (1 << REFS0); // AVcc 5v as reference
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // enable ADC, prescaler 128 (ADC clock = 125kHz)
}

uint16_t readADC(uint8_t channel) {
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07); // select channel 0 to 7 ADC
    ADCSRA |= (1 << ADSC); // convert values
    while (ADCSRA & (1 << ADSC)); // wait for conversion
    return ADCW; // return 10 bit ADC result
}

void update_ir() {
    uint32_t current_time = system_millis;
    
    // Check if can read IR
    if (current_time - last_ir_time < IR_UPDATE_INTERVAL) 
    {
        return;
    }

    last_ir_time = current_time;
    
    uint16_t sensorValue = readADC(IR_PIN);
    if (sensorValue > 3) {
        float distance = (6787.0 / (sensorValue - 3.0)) - 4.0;
        if (distance >= 0 && distance <= 200) {
            latest_ir_distance = (uint16_t)distance;
        }
    }
}

// ---------------- US Sensor Functions ---------------- //
volatile uint32_t pulse_start_left = 0;
volatile uint32_t pulse_width_left = 0;
volatile uint8_t measurement_ready_left = 0;

volatile uint32_t pulse_start_right = 0;
volatile uint32_t pulse_width_right = 0;
volatile uint8_t measurement_ready_right = 0;

// INT0 interrupt (left us sensor) - triggered on both rising and falling edges
ISR(INT0_vect) {
    // Check rising edge (echo pin HIGH -> sent echo pulse can start timer)
    if (PIND & (1 << ECHO_PIN_LEFT)) {
        // Rising edge - start measurement timer
        pulse_start_left = TCNT1;
        // Change to falling edge trigger
        EICRA |= (1 << ISC01);
        EICRA &= ~(1 << ISC00);
    } else {
        // Falling edge - end measurement timer -> pulse received back
        uint32_t pulse_end = TCNT1;
        
        // Handle timer overflow
        if (pulse_end < pulse_start_left) {
            pulse_width_left = (65536 - pulse_start_left) + pulse_end;
        } else {
            pulse_width_left = pulse_end - pulse_start_left;
        }
        
        // Convert to microseconds (Timer1 with prescaler 8)
        pulse_width_left = (pulse_width_left * 8) / (F_CPU / 1000000);
        
        measurement_ready_left = 1;
        
        // Change back to rising edge trigger for next measurement
        EICRA |= (1 << ISC01) | (1 << ISC00);
    }
}

// INT1 interrupt (right us sensor) - triggered on both rising and falling edges
ISR(INT1_vect) {
    // Check rising edge (echo pin HIGH -> sent echo pulse can start timer)
    if (PIND & (1 << ECHO_PIN_RIGHT)) {
        // Rising edge - start measurement timer
        pulse_start_right = TCNT1;
        // Change to falling edge trigger for INT1
        EICRA |= (1 << ISC11);
        EICRA &= ~(1 << ISC10);
    } else {
        // Falling edge - end measurement timer -> pulse received back
        uint32_t pulse_end = TCNT1;
        
        // Handle timer overflow
        if (pulse_end < pulse_start_right) {
            pulse_width_right = (65536 - pulse_start_right) + pulse_end;
        } else {
            pulse_width_right = pulse_end - pulse_start_right;
        }
        
        // Convert to microseconds (Timer1 with prescaler 8)
        pulse_width_right = (pulse_width_right * 8) / (F_CPU / 1000000);
        
        measurement_ready_right = 1;
        
        // Change back to rising edge trigger for next measurement
        EICRA |= (1 << ISC11) | (1 << ISC10);
    }
}

void sendTriggerPulse() {
    measurement_ready_left = 0;
    PORTB &= ~(1 << TRIG_PIN_LEFT); // Make sure trigger pin is LOW
    _delay_us(2);              
    PORTB |= (1 << TRIG_PIN_LEFT);  // Send trigger pulse HIGH
    _delay_us(10);             // Pulse width
    PORTB &= ~(1 << TRIG_PIN_LEFT); // Ensure trigger pin is LOW again
}

void sendTriggerPulseRight() {
    measurement_ready_right = 0;
    PORTB &= ~(1 << TRIG_PIN_RIGHT); // Make sure trigger pin is LOW
    _delay_us(2);              
    PORTB |= (1 << TRIG_PIN_RIGHT);  // Send trigger pulse HIGH
    _delay_us(10);             // Pulse width
    PORTB &= ~(1 << TRIG_PIN_RIGHT); // Ensure trigger pin is LOW again
}

// US trigger (called from main)
void trigger_ultrasonic_left() {
    if (!us_left_trigger) return;
    us_left_trigger = 0;
    sendTriggerPulse();
}

void trigger_ultrasonic_right() {
    if (!us_right_trigger) return;
    us_right_trigger = 0;
    sendTriggerPulseRight();
}

// Check if US pulse measurement is complete (called from main)
void check_ultrasonic_left() {
    if (measurement_ready_left) {
        uint32_t duration = pulse_width_left;
        latest_left_distance = (duration * 343) / 20000;
        measurement_ready_left = 0;
    }
}

void check_ultrasonic_right() {
    if (measurement_ready_right) {
        uint32_t duration = pulse_width_right;
        latest_right_distance = (duration * 343) / 20000;
        measurement_ready_right = 0;
    }
}

// ---------------- Fan Functions ---------------- //
void startLiftFan() {
	PORTD |= (1<<PD4);
}

void stopLiftFan() {
	PORTD &= ~(1<<PD4);
}

void setThrustFan(uint8_t speed) // 0 to 255 value
{
    OCR0A = speed;
}

// ---------------- IMU Sensor Functions ---------------- //
void mpu6050_init(void) {
    uartPrint("Initializing MPU6050...\r\n");
    
    // Wake up MPU-6050
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x6B); // PWR_MGMT_1 register
    i2c_write(0x00);
    i2c_stop();

    _delay_ms(10);

    // Set gyro range scale
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x1B);  // GYRO_CONFIG register
    i2c_write(1 << 3);  // Bits 4:3 set the range
    i2c_stop();
    gyro_scale = 65.5f;
    uartPrint("Gyro range set to ±500°/s\r\n");

    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x1C);  // ACCEL_CONFIG register
    i2c_write(0x08);  // Bits 4:3 set the range
    i2c_stop();
    acc_scale = 8192.0f;
    uartPrint("Accel range set to 4g\r\n");
    
    uartPrint("MPU6050 initialized successfully\r\n");
}

void readIMUCalibration() {
    uint8_t data[14];

    // Write starting register, then repeated start for read
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x3B); // ACCEL_XOUT_H - where data starts
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 1);
    for (uint8_t i = 0; i < 13; i++) data[i] = i2c_read_ack();
    data[13] = i2c_read_nack();
    i2c_stop();

    // raw data readings (skip temp)
    AccX = ((int16_t)data[0] << 8) | data[1];
    AccY = ((int16_t)data[2] << 8) | data[3];
    AccZ = ((int16_t)data[4] << 8) | data[5];
    GyrZ = ((int16_t)data[12] << 8) | data[13];

    // convert to g
    accX = (float)AccX / acc_scale;
    accY = (float)AccY / acc_scale;
    accZ = (float)AccZ / acc_scale;
    gyrZ = (float)GyrZ / gyro_scale;
}

void readIMU() {
    readIMUCalibration();  // Read raw values
    
    // Apply offsets
    accX -= offset_ax;
    accY -= offset_ay;
    accZ -= offset_az;
    gyrZ -= offset_gz;
}

void calibration() {
    uartPrint("Calibrating ... Please do not move IMU\r\n");

    int bufferSize = 1000;
    float buff_ax=0.0f, buff_ay=0.0f, buff_az=0.0f, buff_gx=0.0f, buff_gy=0.0f ,buff_gz=0.0f;
    int i = 0;

    while (i < (bufferSize + 101)) {
        readIMUCalibration();

        if (i > 100 && i <= (bufferSize+100)) // discard the 100 first readings and sum the rest
        {
            buff_ax = buff_ax + accX;
            buff_ay = buff_ay + accY;
            buff_az = buff_az + accZ;
            buff_gz = buff_gz + gyrZ;
        }
        
        if (i==(bufferSize+100)) {
            offset_ax=buff_ax/bufferSize;
            offset_ay=buff_ay/bufferSize;
            offset_az=(buff_az/bufferSize)-1.0f;
            offset_gz=buff_gz/bufferSize;
        }

        i++;
        _delay_ms(2); // to not get repeated measures
    }

    uartPrint("Calibration complete !");
}

void update_imu() {
    if (!imu_ready) return;
    imu_ready = 0;
    
    readIMU();
    yaw += gyrZ * (IMU_UPDATE_INTERVAL / 1000.0f);  // Integration with actual dt -> 5/10^3 = 5ms
}

// ---------------- Servo Functions ---------------- //
void set_servo_angle(float yaw) {
    // Constrain angle to valid range
    if (yaw < -220) yaw = -180;
    if (yaw > 220) yaw = 180;
    
    uint16_t pulse = 3000 + (yaw * 1000) / 180;
    
    OCR1A = pulse;
}

int main() {
    // ---------------- SYSTEM INIT ---------------- //
	setup();
    _delay_ms(5);
    i2c_init();
    _delay_ms(5);
    ADC_init();
    _delay_ms(5);
    mpu6050_init();
    _delay_ms(5);
    calibration();
    sei();
    uartPrint("Hovercraft Initialized ! \r\n");

    int print_counter = 0;

    // Main loop
	while (1)
	{
        // ---------------- READ SENSORS ---------------- // 
        // Time tracker
        uint32_t current_time = system_millis;
    
        update_imu();
        
        trigger_ultrasonic_left();
        trigger_ultrasonic_right();
        check_ultrasonic_left();
        check_ultrasonic_right();
        
        update_ir();

        // ---------------- DEBUGGING ---------------- // 
        // Print every 500ms
        if (++print_counter >= 50) {
            print_counter = 0;
            // Print IMU data (acceleration and yaw)
            uartPrint("Acc X:");
            uartPrintFloat(accX);
            uartPrint(" Y:");
            uartPrintFloat(accY);
            uartPrint(" Z:");
            uartPrintFloat(accZ);
            
            uartPrint(" | Yaw:");
            uartPrintFloat(yaw);
            
            // Print sensor distances
            uartPrint(" | L:");
            uartPrintInt(latest_left_distance);
            uartPrint("cm R:");
            uartPrintInt(latest_right_distance);
            uartPrint("cm IR:");
            uartPrintInt(latest_ir_distance);
            uartPrint("cm\r\n");
        }

        // ---------------- ALGORITHM ---------------- // 
        
    }
}