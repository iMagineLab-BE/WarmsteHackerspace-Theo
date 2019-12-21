#include "EspMQTTClient.h"
#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"

// DFPlayer Mini (MP3 player)
HardwareSerial mySoftwareSerial(1);
DFRobotDFPlayerMini myDFPlayer;

#define BUTTON0_0 13
#define BUTTON0_1 12
#define BUTTON0_2 14
#define BUTTON0_3 27

#define BUTTON1_0 26
#define BUTTON1_1 25
#define BUTTON1_2 33
#define BUTTON1_3 32

#define TOGGLESW 5
#define LED1 34
#define LED2 39

#define LED_ONBOARD 2

// Init mqtt
EspMQTTClient client(
	"theo",
	"plopkoeken",
	"192.168.1.254", // MQTT Broker server ip
	"theo",			// Can be omitted if not needed
	"plopkoeken",   // Can be omitted if not needed
	"ESP32",		// Client name that uniquely identify your device
	1883			// MQTT Port: default = 1883
);

// Init buttons
static volatile uint8_t run_loop = 0;
static volatile uint8_t button_state_changed = 0;
hw_timer_t *timer = NULL;
static uint8_t led_state = 0;
char buff[3];
bool prev_sw_state = 0;
struct
{
	uint8_t button0_pressed : 1;
	uint8_t button1_pressed : 1;
	uint8_t button2_pressed : 1;
	uint8_t button3_pressed : 1;
} buttons;

struct
{
	uint8_t mode_switch : 1;
} switches;

void button_pressed()
{
	button_state_changed = 1;
}

void toggle_switch_toggled()
{
	switches.mode_switch = digitalRead(TOGGLESW);
  sprintf(buff, "%d%d%d", 0, 0, switches.mode_switch);
  client.publish("warmste/week/theo", buff);
	Serial.print("State switch.\n");
}

void IRAM_ATTR timerCallback()
{
	run_loop = 1;
}

void setup()
{
	Serial.begin(115200);

	// inputs
	pinMode(BUTTON0_0, INPUT_PULLUP);
	pinMode(BUTTON0_1, INPUT_PULLUP);
	pinMode(BUTTON0_2, INPUT_PULLUP);
	pinMode(BUTTON0_3, INPUT_PULLUP);

	pinMode(BUTTON1_0, INPUT_PULLUP);
	pinMode(BUTTON1_1, INPUT_PULLUP);
	pinMode(BUTTON1_2, INPUT_PULLUP);
	pinMode(BUTTON1_3, INPUT_PULLUP);

	pinMode(TOGGLESW, INPUT_PULLUP);

	pinMode(LED_ONBOARD, OUTPUT);
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);

	// FALLING and RISING also work as CHANGE...
	attachInterrupt(digitalPinToInterrupt(BUTTON0_0), button_pressed, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BUTTON0_1), button_pressed, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BUTTON0_2), button_pressed, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BUTTON0_3), button_pressed, CHANGE);

	attachInterrupt(digitalPinToInterrupt(BUTTON1_0), button_pressed, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BUTTON1_1), button_pressed, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BUTTON1_2), button_pressed, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BUTTON1_3), button_pressed, CHANGE);

	//attachInterrupt(digitalPinToInterrupt(TOGGLESW), toggle_switch_toggled, CHANGE);
	// attach interrupts => wake from sleep to run mainloop
	// OR run timer, that triggers main loop execution periodically?

	/* Use 1st timer of 4 */
	/* 1 tick take 1/(80MHZ/80000) = 1ms so we set divider 80 and count up */
	timer = timerBegin(0, 80000, true);

	/* attachinterrupt */
	timerAttachInterrupt(timer, &timerCallback, true);
	timerAlarmWrite(timer, 1000, true);
	timerAlarmEnable(timer);
	digitalWrite(LED_ONBOARD, led_state);
  prev_sw_state = digitalRead(TOGGLESW);
  switches.mode_switch = prev_sw_state;

  //DFPlayer Mini
  mySoftwareSerial.begin(9600, SERIAL_8N1, 16, 17);
  myDFPlayer.begin(mySoftwareSerial);
  myDFPlayer.volume(30);
  myDFPlayer.play(1);
}

void onConnectionEstablished() {
    sprintf(buff, "%d%d%d", 0, 0, switches.mode_switch);
    client.publish("warmste/week/theo", buff);
    Serial.print("MQTT connected.\n");
}

void loop()
{
	client.loop(); // for mqtt

	if (run_loop)
	{
		run_loop = 0;

		// button input handling when inputs
		if (button_state_changed)
		{
			button_state_changed = 0;
			if (!digitalRead(BUTTON0_0) || !digitalRead(BUTTON0_1) || !digitalRead(BUTTON0_2) || !digitalRead(BUTTON0_3))
			{
				buttons.button0_pressed = 1;
				digitalWrite(LED_ONBOARD, !led_state);
				led_state = !led_state;
			}
			if (!digitalRead(BUTTON1_0) || !digitalRead(BUTTON1_1) || !digitalRead(BUTTON1_2) || !digitalRead(BUTTON1_3))
			{
				buttons.button1_pressed = 1;
				digitalWrite(LED_ONBOARD, !led_state);
				led_state = !led_state;
			}
		}

		// main statemachine would go here.
    bool sw_state = digitalRead(TOGGLESW);
    if(sw_state != prev_sw_state) {
        switches.mode_switch = sw_state;
        sprintf(buff, "%d%d%d", 0, 0, switches.mode_switch);
        client.publish("warmste/week/theo", buff);
        Serial.print("State switch.\n");
    }
    prev_sw_state = sw_state;
		// first state, send mqtt
		if (sw_state)
		{
			if (true)//(client.isMqttConnected())
			{
				if (buttons.button0_pressed == 1)
				{
					buttons.button0_pressed = 0;
					//client.publish("warmste/week/theo/buttons/0", "1");
          sprintf(buff, "%d%d%d", 1, 0, switches.mode_switch);
          client.publish("warmste/week/theo", buff);
					Serial.print("Message sent.\n");
          // Play "yes"
          if (myDFPlayer.available()) {
              myDFPlayer.play(1);   
          }
				}
				if (buttons.button1_pressed == 1)
				{
					buttons.button1_pressed = 0;
					//client.publish("warmste/week/theo/buttons/1", "1");
          sprintf(buff, "%d%d%d", 0, 1, switches.mode_switch);
          client.publish("warmste/week/theo", buff);
					Serial.print("Message sent.\n");
          // Play "no"
          if (myDFPlayer.available()) {
              myDFPlayer.play(2);   
          }
				}
			}
		} else {
			if (client.isMqttConnected())
			{
				if (buttons.button0_pressed == 1)
				{
					buttons.button0_pressed = 0;
					//client.publish("warmste/week/theo/buttons/0", "1");
          sprintf(buff, "%d%d%d", 1, 0, switches.mode_switch);
          client.publish("warmste/week/theo", buff);
					Serial.print("Message sent.\n");
				}
				if (buttons.button1_pressed == 1)
				{
					buttons.button1_pressed = 0;
					//client.publish("warmste/week/theo/buttons/1", "1");
          sprintf(buff, "%d%d%d", 0, 1, switches.mode_switch);
          client.publish("warmste/week/theo", buff);
					Serial.print("Message sent.\n");
				}
			}
		}
	}
}
