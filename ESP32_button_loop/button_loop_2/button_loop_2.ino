// test with timer loop instead of no timer.

#define BUTTON0 13
#define BUTTON1 12
#define BUTTON2 14
#define BUTTON3 27
#define LED 2

static volatile uint8_t run_loop = 0;
static volatile uint8_t button_state_changed = 0;
static uint8_t led_state = 0;
static uint8_t button0_state = 0;
static uint8_t button1_state = 0;
static uint8_t button2_state = 0;
static uint8_t button3_state = 0;

hw_timer_t * timer = NULL;

void button_pressed(){
  button_state_changed = 1;
}

void IRAM_ATTR timerCallback(){
  run_loop = 1;
}

void setup() {
  // put your setup code here, to run once:

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

void loop() {
  // put your main code here, to run repeatedly:

  if(run_loop){
    run_loop = 0;

    // get intputs
    if(button_state_changed){
      button_state_changed = 0;
      button0_state = digitalRead(BUTTON0);
      button1_state = digitalRead(BUTTON1);
      button2_state = digitalRead(BUTTON2);
      button3_state = digitalRead(BUTTON3);


      digitalWrite(LED, !led_state);
      led_state = !led_state;
    }
  }
}
