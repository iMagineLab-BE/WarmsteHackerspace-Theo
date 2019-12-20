#define BUTTON0 13
#define BUTTON1 12
#define BUTTON2 14
#define BUTTON3 27
#define LED 2

static volatile uint8_t run_loop = 0;
static uint8_t led_state = 0;

void button_pressed(){
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

  // attach interrupts => wake from sleep to run mainloop
  // OR run timer, that triggers main loop execution periodically?

  attachInterrupt(digitalPinToInterrupt(BUTTON0), button_pressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON1), button_pressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON2), button_pressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON3), button_pressed, FALLING);

  digitalWrite(LED, led_state);
}

void loop() {
  // put your main code here, to run repeatedly:

  if(run_loop){
    run_loop = 0;
    digitalWrite(LED, !led_state);
    led_state = !led_state;
  }
  

}
