#include <LiquidCrystal.h>
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

// Servo Library, did not have time to implement otherwise.
#include <Servo.h>
Servo myservo;  // create servo object to control a servo
int potpin = 2;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

// our states for the embedded system
// running => good temp and water, thus running
enum State {
   running = 0,
   error = 1,
   idle = 2
};
enum State state = idle;

// Our global thresholds and variables
#define WATERTHRESH 100
#define TEMPTHRESH 25
volatile unsigned int waterLevel;
unsigned char water_level_port = (unsigned char)0b00000001;


// Declared for port b pins.
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;

// Declared for adc_read
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;


// This method is from the DHT library.
uint8_t bits[5];  // buffer to receive data
double humidity;
double temperature;
#define TIMEOUT 10000
#define dht_apin A0 // Analog Pin sensor is connected to
int read(uint8_t pin)
{
  // INIT BUFFERVAR TO RECEIVE DATA
  uint8_t cnt = 7;
  uint8_t idx = 0;

  // EMPTY BUFFER
  for (int i=0; i< 5; i++) bits[i] = 0;

  // REQUEST SAMPLE
  //PORTF |= (0x01 << 0);

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(20);
  digitalWrite(pin, HIGH);
  delayMicroseconds(40);
  pinMode(pin, INPUT);

  // GET ACKNOWLEDGE or TIMEOUT
  unsigned int loopCnt = TIMEOUT;
  while(digitalRead(pin) == LOW)
    if (loopCnt-- == 0) return -2;

  loopCnt = TIMEOUT;
  while(digitalRead(pin) == HIGH)
    if (loopCnt-- == 0) return -2;
  
  // READ THE OUTPUT - 40 BITS => 5 BYTES
  for (int i=0; i<40; i++)
  {
    loopCnt = TIMEOUT;
    while(digitalRead(pin) == LOW)
      if (loopCnt-- == 0) return -2;

    unsigned long t = micros();

    loopCnt = TIMEOUT;
    while(digitalRead(pin) == HIGH)
      if (loopCnt-- == 0) return -2;

    if ((micros() - t) > 40) bits[idx] |= (1 << cnt);
    if (cnt == 0)   // next byte?
    {
      cnt = 7;   
      idx++;      
    }
    else cnt--;
  }
  
  // CONVERT AND STORE
  humidity    = bits[0];  // bit[1] == 0;
  temperature = bits[2];  // bits[3] == 0;

  // TEST CHECKSUM
  uint8_t sum = bits[0] + bits[2]; // bits[1] && bits[3] both 0
  if (bits[4] != sum) return -1;

  return 0;
}
// Code developed during a lab.
unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= 0b01000000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0x00;

  // set the channel number
  // set the channel selection bits, but remove the most significant bit (bit 3)
  *my_ADMUX = bit(REFS0) | adc_channel_num;
  // set MUX bit 5
  *my_ADCSRB |= 0b00100000;

  // set the channel selection bits
  *my_ADMUX |= *my_ADCSRB;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= (1<<ADSC);
  // wait for the conversion to complete
  while(ADCSRA&(1<<ADSC));
  // return the result in the ADC data register
  *my_ADC_DATA = (ADCL | (ADCH <<8));

  return (*my_ADC_DATA);
}
void running_state()
{
  *port_b |= 0b00010001; // Enable fan and running LED
  *port_b &= 0b00010001; // Disable other LEDs
  read(dht_apin); // Get new values
  waterLevel = adc_read(water_level_port);
    
  if (waterLevel < WATERTHRESH) state = error; // Check Water
  else if ( temperature > TEMPTHRESH ) {       // Check Temp
    delay(1000);
    lcd.setCursor(0, 0);
    lcd.print("Temp:  Humidity:");
    lcd.setCursor(0, 1);
    lcd.print(temperature);
    lcd.setCursor(7, 1);
    lcd.print(humidity);
    // For servo
    val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
    val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    myservo.write(val);                  // sets the servo position according to the scaled value
    delay(15);

    return running_state();                 // Stay in state if temp and water good.
  }
  else{                       
    lcd.clear();
    state = idle;                           // Go Idle if below temp thresh.
  }
}
void idle_state() {
  delay(100);  
  lcd.clear();
  *port_b |= 0b00000100; // Turn on blue LED
  *port_b &= 0b00000100; 
  read(dht_apin); // Get new values
  waterLevel = adc_read(water_level_port);

  // Display to LCD  
  lcd.setCursor(0, 0);
  lcd.print("Temp:  Humidity:");
  lcd.setCursor(0, 1);
  lcd.print(temperature);
  lcd.setCursor(7, 1);
  lcd.print(humidity);
  
  if (waterLevel < WATERTHRESH)   // Check water level
    state = error;
    
  else if (temperature > TEMPTHRESH) // Check temp
    state = running;
}
void error_state() {
  delay(100);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Water Low:");  
  *port_b |= 0b00000010; // Turn on red LED
  *port_b &= 0b00000010; 

  while (waterLevel < WATERTHRESH) {  // Add more water
    delay(1000);    
    waterLevel = adc_read(water_level_port);
    lcd.setCursor(0, 1);
    lcd.print(waterLevel);
    lcd.setCursor(6,1);
    lcd.print("Add More");
  }
  state = idle;
  lcd.clear();    
}

void setup(){
  *port_b &= 0b01111111;
  *ddr_b &= 0b00111111;
  *ddr_b |= 0b00111110;
  read(dht_apin);
  waterLevel = adc_read(water_level_port);  
  Serial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  delay(500);//Delay to let system boot

  lcd.begin(16, 2);
}//end "setup()"

void loop(){
  //Start of Program    
  
  int speed = 0;
  Serial.print("Current humidity = ");
  Serial.print(humidity);
  Serial.print("%  ");
  Serial.print("temperature = ");
  Serial.print(temperature); 
  Serial.println("C  ");
  Serial.print("Current Waterlevel = ");    
  Serial.println(waterLevel);

  switch(state) {
    case error:
      Serial.println("Error State - Need Water");
      error_state();
      break;
    case running:
      Serial.println("Running State");
      running_state();
      break;
    case idle:
      Serial.println("Idle State");
      idle_state();
      break;      
    default:
      break;
  }
  
  delay(3000);
 
}// end loop
