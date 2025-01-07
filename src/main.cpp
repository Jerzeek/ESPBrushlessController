#include <Arduino.h>
// #include <DShotRMT.h>
#include <DShotESC.h>
#include <Wire.h>

// Define the GPIO pin connected to the motor and the DShot protocol used
const gpio_num_t MOTOR_PINS[] = {GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_27};
const rmt_channel_t RMT_CHANNELS[] = {RMT_CHANNEL_0, RMT_CHANNEL_1, RMT_CHANNEL_2, RMT_CHANNEL_3};

DShotESC esc0;
DShotESC esc1;
DShotESC esc2;
DShotESC esc3;
DShotESC *escs[] = {&esc0, &esc1, &esc2, &esc3};
const int NUM_MOTORS = 4;

#define PEAKSPEED 50
#define MIN_THROTTLE 9
#define SINE_DURATION 10000.f // duration of the full cycle, in millis

// Define the failsafe and initial throttle values
const auto FAILSAFE_THROTTLE = 0;
const auto INITIAL_THROTTLE = 48;

bool DEMO_MODE = false;
bool SHOW_UPDATE = true;
int accel_rate = 1;

int last_print_time = 0;
int print_rate = 500;


const int address = 0x04;
char input_buffer[10];

/*
 * Timer setup
*/
// Hardware timer for control updates
uint32_t isrCount = 0, isrTime = 0;
bool update_ok = false;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile SemaphoreHandle_t timerSemaphore;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;
volatile bool _update_ok = false;

uint16_t throttle_register[NUM_MOTORS];
int16_t current_speed[NUM_MOTORS];
uint16_t target_speed[NUM_MOTORS];

void set_demo_mode(bool demo_mode) {
	DEMO_MODE = demo_mode;
	Serial.println("Setting demo mode to " + String(demo_mode));
}

void set_speed(int motor_id, int speed) {
	target_speed[motor_id] = speed;
	Serial.println("Setting speed for motor " + String(motor_id) + " to " + String(speed));
}

void set_acceleration(int acceleration) {
	accel_rate = acceleration;
	Serial.println("Setting max acceleration to " + String(acceleration) + " steps per loop");
}

void emergencyStop() {
	for(int i = 0; i < NUM_MOTORS; i++) {
		escs[i]->sendThrottle(FAILSAFE_THROTTLE);
		current_speed[i] = FAILSAFE_THROTTLE;
		target_speed[i] = FAILSAFE_THROTTLE;
	}
	Serial.println("Emergency stop");
}



void handle_wire_input() {
	Serial.println("Handling input: " + String(input_buffer));
	if(input_buffer[0] == 'S') { // Setting speedS 0 200

		set_speed(input_buffer[1], input_buffer[2] * 256 + input_buffer[3]);
	}
	else if(input_buffer[0] == 'A') { // Set acceleration
		set_acceleration(input_buffer[1] * 256 + input_buffer[2]);
	}
	else if(input_buffer[0] == 'D') {
		set_demo_mode(input_buffer[1] == '1');
	}
	else if( input_buffer[0] == 'X') {
		emergencyStop();
	}
}

void receiveEvent(int howMany) {
  int i = 0;
  while(Wire.available()) {
    char c = Wire.read();
    Serial.print(c);
    input_buffer[i] = c;
    i++;
  }
  Serial.println("Received: " + String(input_buffer));
  handle_wire_input();
}

void handle_serial_input() {
	// Read a single byte from the Serial input
	char command = Serial.read();
	if(command == 'S' || command == 's') {
		set_speed(Serial.parseInt(), Serial.parseInt());
	}
	else if(command == 'A' || command == 'a') {
		set_acceleration(Serial.parseInt());
	}
	else if(command == 'D' || command == 'd') {
		set_demo_mode(Serial.parseInt() == 1);
	}
	else if(command == 'X' || command == 'x') {
		emergencyStop();
	}
}



void standardUpdate() {
	if(SHOW_UPDATE) {
		Serial.print("Throttle Values: " );
	}
	for(int i = 0; i < NUM_MOTORS; i++) {
		if(current_speed[i] < target_speed[i]) {
			current_speed[i] += accel_rate;
			if(current_speed[i] > target_speed[i]) {
				current_speed[i] = target_speed[i];
			}
		}
		else if(current_speed[i] > target_speed[i]) {
			current_speed[i] -= accel_rate;
			if(current_speed[i] < target_speed[i]) {
				current_speed[i] = target_speed[i];
			}
		}
		if(SHOW_UPDATE) {
			Serial.print( String(i) + ": " + String(current_speed[i]) + "[" + String(target_speed[i]) + "] ");
		}
	}
	if(SHOW_UPDATE) {
		Serial.println();
	}
}

void demoUpdate() {
	if(SHOW_UPDATE) {
		Serial.print("Throttle Values: " );
	}
	for(int i = 0; i < NUM_MOTORS; i++) {
		int16_t milliswrap = sin(millis()*2/SINE_DURATION*PI + (i*PI/2))*PEAKSPEED + PEAKSPEED;
		current_speed[i] = milliswrap;
		if(SHOW_UPDATE) {
			Serial.print( String(i) + ": " + String(milliswrap) + " ");
		}
	}
	if(SHOW_UPDATE) {
		Serial.println();
	}
}


void ARDUINO_ISR_ATTR onTimer() {
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  _update_ok = false;
  for(int i = 0; i < NUM_MOTORS; i++) {
	escs[i]->sendThrottle(throttle_register[i]);
  }
  _update_ok = true;
  isrCounter = isrCounter + 1;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

void setup()
{
	Serial.begin(115200); // set up Serial library at 9600 bps
	while (!Serial) { };
	//delay(500);
	Serial.println("MotorControlAZ v1 - setting up ESCs");
	// esc1.begin(DSHOT_MODE,NO_BIDIRECTION);
	// esc2.begin(DSHOT_MODE,NO_BIDIRECTION);
	// esc1.send_dshot_value(INITIAL_THROTTLE);
	// esc2.send_dshot_value(INITIAL_THROTTLE);

	Serial.println("Installing ESCs");

	// Don't put this in a for loop. For some reason, that makes it go crazy.
	esc0.install(MOTOR_PINS[0], RMT_CHANNELS[0]);
	esc1.install(MOTOR_PINS[1], RMT_CHANNELS[1]);
	esc2.install(MOTOR_PINS[2], RMT_CHANNELS[2]);
	esc3.install(MOTOR_PINS[3], RMT_CHANNELS[3]);

	/* This doesnt work! I don't know why. It should be the same...
	   for(int i = 0; i < 4; i++) {
		   escs[i]->install(MOTOR_PINS[i], RMT_CHANNELS[i]);
	   }
	   */

	Serial.println("Initializing ESCs");
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		escs[i]->init();
		escs[i]->setReversed(false);
		escs[i]->set3DMode(false);
	}

	Serial.print("Arming ESCs: ");
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		Serial.print(" esc");
		Serial.print(i);
		Serial.print(": ");
		Serial.print(escs[i]->throttleArm(300));
	}
	Serial.println();

 /*
	for (int t = 0; t < 5; t++)
	{
		for (int i = 0; i < 4; i++)
		{
			escs[i]->beep(t);
			delay(100);
		}
		delay(100);
	}
 */
	Serial.println("ESCs ready");
	timerSemaphore = xSemaphoreCreateBinary(); // Create semaphore to inform us when the timer has fired
	timer = timerBegin(1000000); // Set timer frequency to 1Mhz
	timerAttachInterrupt(timer, &onTimer); // Attach onTimer function to our timer.
	// Set alarm to call onTimer function every second (value in microseconds).
	// Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
	timerAlarm(timer, 10000, true, 0);
 
}

int map_throttle(int throttle)
{
	if (throttle < MIN_THROTTLE)
		return 0;
	return throttle;
}

void updateShouldPrint() {
	if(millis() - last_print_time > print_rate) {
		last_print_time = millis();
		SHOW_UPDATE = true;
	}
	else {
		SHOW_UPDATE = false;
	}
}

void updateThrottleRegister() {
	portENTER_CRITICAL(&timerMux);
	for(int i = 0; i < NUM_MOTORS; i++) {
		throttle_register[i] = map_throttle(current_speed[i]);
	}
	portEXIT_CRITICAL(&timerMux);
}

void loop()
{
	updateShouldPrint();
	if(Serial.available()) { handle_serial_input(); }

	if(DEMO_MODE) { demoUpdate(); }
	else { standardUpdate(); }

	updateThrottleRegister();

	portEXIT_CRITICAL(&timerMux);
	delay(100);
}




/*
 * Leftover bits for posterity
 */

	// If Timer has fired
	/*
	if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
		// Read the interrupt count and time
		portENTER_CRITICAL(&timerMux);
		isrCount = isrCounter;
		isrTime = lastIsrAt;
		update_ok = _update_ok;
		//write_throttles();
		portEXIT_CRITICAL(&timerMux);
	}

	if(true) {
		// Print it
		Serial.print("On timer no. ");
		Serial.print(isrCount);
		Serial.print(" at ");
		Serial.print(isrTime);
		Serial.print(" ms. ");
		Serial.println("Update ok: " + String(update_ok));
	}

	*/