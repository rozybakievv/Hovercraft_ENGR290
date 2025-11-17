#include <Arduino.h>
#define F_CPU 16000000UL // 16 MHz clock
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define LIFT_FAN PD4

#define TRIG_PIN_LEFT PB3  // Trigger pin
#define ECHO_PIN_LEFT PD2  // Echo pin (PD2) - INT0
#define TRIG_PIN_RIGHT PB5  // Trigger pin
#define ECHO_PIN_RIGHT PD3  // Echo pin (PD2) - INT0

#define FAN_THRUST_PIN PD6

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
    TCCR1A = 0;
    TCCR1B |= (1 << CS11); // Prescaler 8
    TCNT1 = 0;

    // PD4 as output (Lift Fan)
    DDRD |= (1 << PD4);
    // Start with lift fan OFF
    PORTD &= ~(1 << PD4);

    // PD6 as output (Thrust Fan - OC0A)
    DDRD |= (1 << PD6);

    // Configure Timer0 for Fast PWM on OC0A (PD6)
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
    
    // Enable global interrupts
    sei();
}

// --- UART Functions --- //
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

// ---- I2C Functions ---- //
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

// --- IR Sensor Functions --- //
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

uint16_t getDistanceTop() {
    uint16_t sensorValueRaw = readADC(IR_PIN);

    if (sensorValueRaw == 0) {
        uartPrint("Error: ADC returned 0!\r\n");
        return 0;
    }
    
    // IR sensor calibration equation
    float distance = (6787.0 / (sensorValueRaw - 3.0)) - 4.0;

    // range check
    if (distance < 0 || distance > 200) {
        uartPrint("Error: Out of range!\r\n");
        return 0;
    }

    return (uint16_t)distance;
}


// --- US Sensor Functions --- //
// Variables for interrupt handling
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

// INT1 interrupt (RIGHT us sensor) - triggered on both rising and falling edges
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

uint32_t getDistanceRight() {
    sendTriggerPulseRight();

    uint16_t timeout = 100000;
    while (!measurement_ready_right && timeout--) {
        _delay_us(1);
    }

    if (measurement_ready_right == 1)
    {
        uint32_t duration = pulse_width_right;
        // Speed of sound is 0.0343 cm/us
        // Distance = (duration * 343) / 20000
        uint32_t distance = (duration * 343) / 20000;
        return distance;
    }

    return 9999;
}

uint32_t getDistanceLeft() {
    sendTriggerPulse();

    uint16_t timeout = 100000;
    while (!measurement_ready_left && timeout--) {
        _delay_us(1);
    }

    if (measurement_ready_left == 1)
    {
        uint32_t duration = pulse_width_left;
        // Speed of sound is 0.0343 cm/us
        // Distance = (duration * 343) / 20000
        uint32_t distance = (duration * 343) / 20000;
        return distance;
    }

    return 9999;
}

// --- Fan Functions --- //
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


// --- IMU Sensor Functions --- //
void mpu6050_init(void) {
    uartPrint("Initializing MPU6050...\r\n");
    
    // Wake up MPU-6050
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x6B); // PWR_MGMT_1
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
    i2c_write(0x1C);  // GYRO_CONFIG register
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
    i2c_write(0x3B); // ACCEL_XOUT_H
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

void getRoll() {
    roll = atan2(-accX, accZ) * 180.0f / M_PI;
}

int main() {
    // Input / Outputs setup and sensors initialization
	setup();
    _delay_ms(5);
    // i2c_init();
    _delay_ms(5);
    // ADC_init();
    _delay_ms(5);
    // mpu6050_init();
    _delay_ms(5);
    // calibration();
    uartPrint("Hovercraft Initialized ! \r\n");

    int counter = 0;

	// Main loop
	while (1)
	{
        // PWM THRUST
        /* setThrustFan(30);
        uartPrint("Thrust fan set to 255\r\n");
        uartPrintInt(OCR0A);
        uartPrint("\r\n");
        _delay_ms(1000); */


        // IMU Sensor
        /* readIMU();
        yaw += gyrZ * 0.01f; // gyroscope z-axis data over time (0.01f seconds delay)

        getRoll();

        // prints only 100 iterations (1s)
        if (counter >= 100) {
            uartPrint("Acc - X:");
            uartPrintFloat(accX);
            uartPrint(" Y:");
            uartPrintFloat(accY);
            uartPrint(" Z:");
            uartPrintFloat(accZ);
            
            uartPrint("| Yaw: ");
            uartPrintFloat(yaw);

            uartPrint("\r\n");

            counter = 0;
        }
        
        counter++;

        _delay_ms(10); */

        // IR Sensor
/*         uint16_t distanceIR = getDistanceTop();
        uartPrint("IR Distance\r\n");
        uartPrintInt(distanceIR);
        uartPrint("cm\r\n");
        _delay_ms(500);

        if (distanceIR > 40)
        {
            stopLiftFan();

            // TODO: stop thrust
        } else {
            // TODO: Continue
        } */
        

        // US Sensor test
        /* uint32_t distanceLeft32 = getDistanceLeft();
        if (distanceLeft32 == 9999)
        {
            uartPrint("US Sensor left error\r\n");
        } else {
            uartPrint("LEFT: ");
            uartPrintInt(distanceLeft32);
            uartPrint("cm \r\n");
        }

        uint32_t distanceRight32 = getDistanceRight();
        if (distanceRight32 == 9999) {
            uartPrint("US Sensor right error\r\n");
        } else {
            uartPrint("RIGHT: ");
            uartPrintInt(distanceRight32);
            uartPrint(" cm\r\n");
        }

        _delay_ms(500); */

        // Test lift fan
		startLiftFan();
        setThrustFan(200);
		uartPrint("Starting...\r\n");
		_delay_ms(20000);
		uartPrint("Stopping...\r\n");
		stopLiftFan();
        setThrustFan(0);
		_delay_ms(5000);
	}
}