#include "EspMQTTClient.h"

#define BUTTON0 13
#define BUTTON1 12
#define BUTTON2 14
#define BUTTON3 27
#define LED 2

// Init mqtt
EspMQTTClient client(
    "BOTCONTROL",
    "newcaprica",
    "172.16.0.137",       // MQTT Broker server ip
    "theo",               // Can be omitted if not needed
    "plopkoeken",         // Can be omitted if not needed
    "ESP32",              // Client name that uniquely identify your device
    1883                  // MQTT Port: default = 1883
);

// Init buttons
static volatile uint8_t run_loop = 0;
static volatile uint8_t button_state_changed = 0;
static uint8_t led_state = 0;
static uint8_t button0_state = 0;
static uint8_t button1_state = 0;
static uint8_t button2_state = 0;
static uint8_t button3_state = 0;
hw_timer_t * timer = NULL;

void button_pressed() {
    button_state_changed = 1;
}

void IRAM_ATTR timerCallback() {
    run_loop = 1;
}

void setup() {
    Serial.begin(115200);

    // inputs
    pinMode(BUTTON0, INPUT_PULLUP);
    pinMode(BUTTON1, INPUT_PULLUP);
    pinMode(BUTTON2, INPUT_PULLUP);
    pinMode(BUTTON3, INPUT_PULLUP);

    pinMode(LED, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(BUTTON0), button_pressed, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON1), button_pressed, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON2), button_pressed, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON3), button_pressed, FALLING);
    // attach interrupts => wake from sleep to run mainloop
    // OR run timer, that triggers main loop execution periodically?

    /* Use 1st timer of 4 */
    /* 1 tick take 1/(80MHZ/80000) = 1ms so we set divider 80 and count up */
    timer = timerBegin(0, 80000, true);

    /* attachinterrupt */
    timerAttachInterrupt(timer, &timerCallback, true);
    timerAlarmWrite(timer, 1000, true);
    timerAlarmEnable(timer);
    digitalWrite(LED, led_state);
}

void onConnectionEstablished() {}

void loop() {
    client.loop(); // for mqtt 

    if(run_loop) {
        run_loop = 0;

        // get intputs
        if(button_state_changed) {
            button_state_changed = 0;
            button0_state = digitalRead(BUTTON0);
            button1_state = digitalRead(BUTTON1);
            button2_state = digitalRead(BUTTON2);
            button3_state = digitalRead(BUTTON3);
            
            digitalWrite(LED, !led_state);
            led_state = !led_state;

            if(client.isMqttConnected()) {
                client.publish("warmste/week/theo", "Hi!");
                Serial.print("Message sent.\n");
            }
        }
    }
}
