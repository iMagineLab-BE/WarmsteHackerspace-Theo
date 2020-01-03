#include "EspMQTTClient.h"
#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"

/* Turn on debug mode - turn off to lower power consumption */
//#define DEBUG

/* Turn on sleep mode - turn on to lower power consumption */
#define SLEEP

/* DFPlayer Mini (MP3 player) */
HardwareSerial mySoftwareSerial(1);
DFRobotDFPlayerMini myDFPlayer;
#define BUSY_PIN 18     // DFPlayer busy pin input

/* Button 0 input */
#define BUTTON0_0 13
#define BUTTON0_1 12
#define BUTTON0_2 14
#define BUTTON0_3 27

/* Button 1 input */
#define BUTTON1_0 26
#define BUTTON1_1 25
#define BUTTON1_2 33
#define BUTTON1_3 32

/* Mode selection switch input */
#define TOGGLESW 5

/* On-board LEDs */
#define LED1 34
#define LED2 39
#define LED_ONBOARD 2

/* MQTT client configuration */
EspMQTTClient mqtt_client(
  "theo",             // Wi-Fi SSID
  "plopkoeken",       // Wi-Fi Password
  "192.168.13.254",   // MQTT Broker server ip
  "theo",             // MQTT Username - Can be omitted if not needed
  "plopkoeken",       // MQTT Password Can be omitted if not needed
  "THEO-CONTROLLER",  // Client name that uniquely identify your device
  1883                // MQTT Port: default = 1883
);

/* Variables declaration */
static volatile uint8_t button_0_state_changed = 0;
static volatile uint8_t button_1_state_changed = 0;
static volatile uint8_t switch_state_changed = 0;
static uint8_t led_state = 0;
char send_buff[3];
struct
{
  uint8_t button0_state : 1;
  uint8_t button0_initial_press : 1;
  uint8_t button1_state : 1;
  uint8_t button1_initial_press : 1;
  uint8_t button2_state : 1;
  uint8_t button2_initial_press : 1;
  uint8_t button3_state : 1;
  uint8_t button3_initial_press : 1;
} buttons;

struct
{
  uint8_t mode_switch : 1;
} switches;

/* Interrupt callback - Button 0 */
void button_0_pressed()
{
  button_0_state_changed = 1;
}

/* Interrupt callback - Button 1 */
void button_1_pressed()
{
  button_1_state_changed = 1;
}

/* Interrupt callback - Switch */
void switch_toggled()
{
  switch_state_changed = 1;
}

void setLedState(uint8_t led, uint8_t state)
{
  digitalWrite(led, state);
}

void toggleLedState(uint8_t led, uint8_t* state)
{
  *state = !*state;
  digitalWrite(led, *state);
}

bool dfplayerReady()
{
  return myDFPlayer.waitAvailable(10) && digitalRead(BUSY_PIN);
}

void setup()
{
  /* Initialise serial debug communication */
  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  /* Initialise inputs */
  pinMode(BUTTON0_0, INPUT_PULLUP);
  pinMode(BUTTON0_1, INPUT_PULLUP);
  pinMode(BUTTON0_2, INPUT_PULLUP);
  pinMode(BUTTON0_3, INPUT_PULLUP);

  pinMode(BUTTON1_0, INPUT_PULLUP);
  pinMode(BUTTON1_1, INPUT_PULLUP);
  pinMode(BUTTON1_2, INPUT_PULLUP);
  pinMode(BUTTON1_3, INPUT_PULLUP);

  pinMode(TOGGLESW, INPUT_PULLUP);

  pinMode(BUSY_PIN, INPUT);

  /* Initialise outputs */
  pinMode(LED_ONBOARD, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  /* Define interrupts on inputs */
  // FALLING and RISING also work as CHANGE...
  attachInterrupt(digitalPinToInterrupt(BUTTON0_0), button_0_pressed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON0_1), button_0_pressed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON0_2), button_0_pressed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON0_3), button_0_pressed, CHANGE);

  attachInterrupt(digitalPinToInterrupt(BUTTON1_0), button_1_pressed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON1_1), button_1_pressed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON1_2), button_1_pressed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON1_3), button_1_pressed, CHANGE);

  //attachInterrupt(digitalPinToInterrupt(TOGGLESW), switch_toggled, CHANGE);

  /* Enable sleep wakeup from input pins */
  gpio_wakeup_enable((gpio_num_t)BUTTON0_0, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)BUTTON0_1, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)BUTTON0_2, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)BUTTON0_3, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)BUTTON1_0, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)BUTTON1_1, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)BUTTON1_2, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)BUTTON1_3, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)TOGGLESW, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  setLedState(LED_ONBOARD, led_state);
  switches.mode_switch = digitalRead(TOGGLESW);

  /* DFPlayer Mini (MP3 player) initialisation */
  mySoftwareSerial.begin(9600, SERIAL_8N1, 16, 17);
  myDFPlayer.begin(mySoftwareSerial);
  myDFPlayer.reset();
  myDFPlayer.volume(30);
}

/* Callback when successfully connected to MQTT broker */
void onConnectionEstablished()
{
  sprintf(send_buff, "%d%d%d", 0, 0, switches.mode_switch);
  mqtt_client.publish("warmste/week/theo", send_buff);

  /* Say "connected" */
  if(dfplayerReady())
  {
    #ifdef DEBUG
    Serial.print("Say CONNECTED.\n");
    #endif
    
    myDFPlayer.play(3);   
  }

  #ifdef DEBUG
  Serial.print("MQTT connected.\n");
  #endif
}

/* Perform task in local mode - Saying Yes/No */
void processLocalState()
{
  if(buttons.button0_state == 1 && buttons.button0_initial_press == 1)
  {
    buttons.button0_initial_press = 0;

    /* Say "no" */
    if(dfplayerReady())
    {
      #ifdef DEBUG
      Serial.print("Say NO.\n");
      #endif

      myDFPlayer.play(1);         //Track 1 - NO   
    }
  }
  
  if(buttons.button1_state == 1 && buttons.button1_initial_press == 1)
  {
    buttons.button1_initial_press = 0;

    /* Say "yes" */
    if(dfplayerReady())
    {
      #ifdef DEBUG
      Serial.print("Say YES.\n");
      #endif

      myDFPlayer.play(2);         //Track 2 - YES
    }
  }
}

/* Perform task in remote mode - Communication with MQTT server */
void processRemoteState()
{
  if(mqtt_client.isConnected())
  {
    if(buttons.button0_state == 1 && buttons.button0_initial_press == 1)
    {
      buttons.button0_initial_press = 0;
      sprintf(send_buff, "%d%d%d", 1, 0, switches.mode_switch);
      mqtt_client.publish("warmste/week/theo", send_buff);

      #ifdef DEBUG
      Serial.print("Message sent.\n");
      #endif
    }
    
    if(buttons.button1_state == 1 && buttons.button1_initial_press == 1)
    {
      buttons.button1_initial_press = 0;
      sprintf(send_buff, "%d%d%d", 0, 1, switches.mode_switch);
      mqtt_client.publish("warmste/week/theo", send_buff);

      #ifdef DEBUG
      Serial.print("Message sent.\n");
      #endif
    }
  }
}

/* Read input state of Button 0 */
bool readStateButton0()
{
  return !digitalRead(BUTTON0_0) || !digitalRead(BUTTON0_1) || !digitalRead(BUTTON0_2) || !digitalRead(BUTTON0_3);
}

/* Read input state of Button 1 */
bool readStateButton1()
{
  return !digitalRead(BUTTON1_0) || !digitalRead(BUTTON1_1) || !digitalRead(BUTTON1_2) || !digitalRead(BUTTON1_3);
}

/* Request sleep mode */
void enterSleep()
{
  if(!buttons.button0_state && !buttons.button1_state && dfplayerReady())
  {
    /* Enter sleep mode */
    #ifdef DEBUG
    Serial.print("Enter sleep mode.\n");
    #endif

    esp_light_sleep_start();
  }
}

/* Program loop */
void loop()
{
  /* Handle button events */
  if(button_0_state_changed)
  {
    button_0_state_changed = 0;
  
    if(readStateButton0())
    {
      buttons.button0_initial_press = !buttons.button0_state;
      buttons.button0_state = 1;
      #ifdef DEBUG
      toggleLedState(LED_ONBOARD, &led_state);
      #endif
    }
    else
    {
      buttons.button0_state = 0;
      #ifdef DEBUG
      toggleLedState(LED_ONBOARD, &led_state);
      #endif
    }
  }

  if(button_1_state_changed)
  {
    button_1_state_changed = 0;
      
    if(readStateButton1())
    {
      buttons.button1_initial_press = !buttons.button1_state;
      buttons.button1_state = 1;
      #ifdef DEBUG
      toggleLedState(LED_ONBOARD, &led_state);
      #endif
    }
    else
    {
      buttons.button1_state = 0;
      #ifdef DEBUG
      toggleLedState(LED_ONBOARD, &led_state);
      #endif
    }
  }

  /* Reading switch state */  
  uint8_t sw_state = digitalRead(TOGGLESW);
  if(sw_state != switches.mode_switch)
  {
    switches.mode_switch = sw_state;

    if(mqtt_client.isConnected())
    {
      sprintf(send_buff, "%d%d%d", 0, 0, switches.mode_switch);
      mqtt_client.publish("warmste/week/theo", send_buff);
    }
    
    #ifdef DEBUG
    Serial.print("State switch.\n");
    #endif
  }
	
  /* Select mode with switch */
  if(switches.mode_switch)
  {
    /* Local state - say Yes/No */
    processLocalState();

    /* Enter sleep (low power mode) */
    #ifdef SLEEP
    enterSleep();
    #endif
  }
  else
  {  
    /* Remote state - Communicating with MQTT server */
    processRemoteState();

    /* MQTT communication handle */
    mqtt_client.loop();
  }
}
