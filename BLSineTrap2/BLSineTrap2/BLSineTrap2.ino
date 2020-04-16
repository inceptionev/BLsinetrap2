/*
  BLTrap
  Pinned for the Blueseries 20A ESC
  Sensed Trapezoidal brushless motor controller for low speed gimbal motor applications.
  
  TODO:
  switch to cycle timing or interrupt timing if micros() timing becomes too slow.
  handle timer overflow
 
 */
 

//Arduino assignments
#define PHA_HI 4
#define PHA_LO 5
#define PHB_HI A5
#define PHB_LO A4
#define PHC_HI A3
#define PHC_LO 8
#define RCPIN 0
#define RC_INT 2
#define ZCPIN 3
#define RCOUT 12

//Fast access assignment
#define PHA_HI_PORT PORTD
#define PHA_HI_PIN 4
#define PHA_LO_PORT PORTD
#define PHA_LO_PIN 5
#define PHB_HI_PORT PORTC
#define PHB_HI_PIN 5
#define PHB_LO_PORT PORTC
#define PHB_LO_PIN 4
#define PHC_HI_PORT PORTC
#define PHC_HI_PIN 3
#define PHC_LO_PORT PORTB
#define PHC_LO_PIN 0
#define RCPIN_PORT PORTD
#define RCPIN_PIN 0
#define RC_INT_PORT PORTD
#define RC_INT_PIN 2
#define ZC_PORT PORTD
#define ZC_PIN 3
#define RCOUT_PORT PORTB
#define RCOUT_PIN 4
#define TEST_PORT PORTD
#define TEST_PIN 1

//ADC Assignements
#define PHA_SENSE_MUX 6
#define PHB_SENSE_MUX 7
#define PHC_SENSE_MUX 0

//Trapezoidal Commutaion Detection Control Values
#define ZC_HOLDOFF 0x00FA //about 1ms when prescaler is Fs/64
#define MIN_COMMUTATION_TIME 0x03FF
#define PWMSTEPS 128
#define COMM_TIMEOUT 0x0FFF

//Sine Commutation Control Values
#define STARTUP_CYCLES 1200
#define PWMSTEPS_SINE 64
#define SINESTEPS 32
#define PHASEOFFSET 20

//Sine-Trap Transistion
#define TRANSITION_SPEED 64

//RC control
#define SPEED_OFF_THRESH 5 //out of 64

//Sine Commutation Timing Variables
char sinetable [32];
int A_HI_ON;
int A_HI_OFF;
int A_LO_ON;
int A_LO_OFF;
int B_HI_ON;
int B_HI_OFF;
int B_LO_ON;
int B_LO_OFF;
int C_HI_ON;
int C_HI_OFF;
int C_LO_ON;
int C_LO_OFF;

//setup for RC command input
unsigned int rcpulse;

//setup for Trapezoidal Commutation
int PWMCOUNTER = 0;
int SPEED = 0; //this is PWMSTEPS number of bits
byte state = 0;

//setup for Sine Commutation
int PWMCOUNTER_SINE = 0;
int SINECOUNTER = 0;
int SPEEDCOUNTER = 0;
int ACB = 1; //anti-crowbar.  be careful if causes one of the triggers to exceed PWMSTEPS
int PHASESTEP = 10; //120 degrees as a fraction of SINESTEPS
int SINEMOD = SINESTEPS-1;


// the setup routine runs once when you press reset:
void setup() {                
  // initialize GPIOs
  pinMode(PHA_HI, OUTPUT); 
  pinMode(PHA_LO, OUTPUT); 
  pinMode(PHB_HI, OUTPUT); 
  pinMode(PHB_LO, OUTPUT); 
  pinMode(PHC_HI, OUTPUT); 
  pinMode(PHC_LO, OUTPUT); 
  pinMode(RC_INT, INPUT);
  pinMode(ZCPIN, OUTPUT);
  pinMode(RCOUT, OUTPUT);
  pinMode(0, INPUT);
  pinMode(1, OUTPUT); //this is the test pin
  
  //init the sinetable
  arraysetup();
  
  rcpulse = 0;  
  
  //ADC Setup
  //ADMUX |= (1 << REFS0); //set to internal reference, comment out for ext AREF
  ADMUX &= ~(1 << REFS0); //set to ext AREF
  ADMUX &= ~(1 << REFS1); //set to ext AREF
  /*analogReference(EXTERNAL);
  ADMUX |= (1 << ADLAR); //left-adjust the result to just read the top 8 bits in ADCH
  ADCSRA |= 4; //set ADC clock rate to 1 MHz
  ADCSRA |= (1 << ADFR); //make the ADC free-running
  delay(1);
  ADCSRA |= (1 << ADEN); //enable the ADC
  ADCSRA |= (1 << ADSC); //start the ADC
  */
  
  //TIMER1 setup
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0; //reset timer
  TCCR1B |= (1 << WGM12) | (3); //set CTC mode and f/1024 prescaler
  OCR1A = 0xFFFF; //set compare register
  TIMSK |= (1 << OCIE1A);  //unmask the OC1A compare interrupt  TIMSK for ATMEGA8
  //TIMSK1 |= (1 << OCIE1A);  //unmask the OC1A compare interrupt TIMSK1 for ATMEGA328
  
  
  //RC pulse interrupt setup
  GICR |= (1 << INT0); //unmask the interrupt 
  MCUCR |= (1 << ISC00) | (1 << ISC01); //interrupt on rising edge
  TCCR0 = 0;
  TCCR0 |= 4; //set prescaler to clk/256 = 16uS/count or 63 counts/ms
  
  
  //final setup of starting state  
  ADMUX = (ADMUX & 0xF8) | PHC_SENSE_MUX; //select sense line C, for starters
    
  //Analog Comparator Setup
  SPEED = 110; //for testing purposes
  ADCSRA &= ~(1 << ADSC); //stop the ADC
  ADCSRA &= ~(1 << ADEN); //disable the ADC
  SFIOR |= (1 << ACME); //enable the comparator mux
  //Note no comparator interrupt here, we're gonna read it synchronously with PWM // ACSR |= (1 << ACIE);
  
  int startup_counter = 0;
  
  //testing area
  TEST_PORT &= ~(1 << TEST_PIN);
  
  //turning on interrupts
  sei();
  
  state = 14; //setup for start, and start in sine commutation mode
}

void loop() {
  
  
  switch(state){ 
    case 0: //start
      
      //TURN OFF EVERYTHING!
      PHA_LO_PORT &= ~_BV(PHA_LO_PIN); //low side phase A off
      PHB_LO_PORT &= ~_BV(PHB_LO_PIN); //low side phase B off
      PHC_LO_PORT &= ~_BV(PHC_LO_PIN); //low side phase C off
      PHA_HI_PORT |= _BV(PHA_HI_PIN);  //High side phase A off
      PHB_HI_PORT |= _BV(PHB_HI_PIN);  //High side phase B off
      PHC_HI_PORT |= _BV(PHC_HI_PIN);  //High side phase C off
      delay(1);
      TCNT1 = 0; //reset the timer
      OCR1A = 0x07FF; //reset the timer length, about 1 second
      ADMUX = (ADMUX & 0xF8) | PHC_SENSE_MUX; //select sense line C, for starters
      
      TIMSK |= (1 << OCIE1A); //enable the timer interrupt
      
      TEST_PORT &= ~(1 << TEST_PIN);//for testing
      
      //advance the state to start trapezoidal commutation
      state = 1;
      break;
      
    case 1: //AB detect
      //wait for zero-crossing
      //turn off the previous high side - PHASE C
      PHC_HI_PORT |= (1 << PHC_HI_PIN);
      //turn on the high side - PHASE A
      PHA_HI_PORT &= ~_BV(PHA_HI_PIN);//digitalWrite(PHA_HI, LOW);
      
      //PWM the low side - PHASE B
      PWMcycleRC_PHB_LO();
      
      //watch for the commutation zero-crossing - PHASE C
      if((TCNT1 > ZC_HOLDOFF) && (ZC_PORT & (1 << ZC_PIN))) { 
        state++;
        OCR1A = TCNT1 << 1; //double the current count
      }
      if(TCNT1 > COMM_TIMEOUT) {
        state = 0;
      }
      break;

    case 2: //AB commute
      //wait for commutation
      //turn off the previous high side - PHASE C
      PHC_HI_PORT |= (1 << PHC_HI_PIN);
      //turn on the high side - PHASE A
      PHA_HI_PORT &= ~_BV(PHA_HI_PIN);//digitalWrite(PHA_HI, LOW);
      
      //PWM the low side - PHASE B
      PWMcycleRC_PHB_LO();
      
      //the interrupt will advance the state here
      if(TCNT1 > COMM_TIMEOUT) {
        state = 0;
      }
      break;
      
    case 3: //AC detect
      //wait for zero-crossing
      //turn off the previous high side - PHASE C
      PHC_HI_PORT |= (1 << PHC_HI_PIN);
      //turn on the high side - PHASE A
      PHA_HI_PORT &= ~_BV(PHA_HI_PIN);//digitalWrite(PHA_HI, LOW);
      
      //PWM the low side - PHASE C
      PWMcycleRC_PHC_LO();
      
      //watch for the commutation zero-crossing - PHASE B
      if((TCNT1 > ZC_HOLDOFF) && (!(ZC_PORT & (1 << ZC_PIN)))) { 
        state++;
        OCR1A = TCNT1 << 1; //double the current count
      }
      if(TCNT1 > COMM_TIMEOUT) {
        state = 0;
      }     
      break;

    case 4: //AC commute
      //wait for commutation
      //turn off the previous high side - PHASE C
      PHC_HI_PORT |= (1 << PHC_HI_PIN);
      //turn on the high side - PHASE A
      PHA_HI_PORT &= ~_BV(PHA_HI_PIN);//digitalWrite(PHA_HI, LOW);
      
      //PWM the low side - PHASE C
      PWMcycleRC_PHC_LO();
      
      //the interrupt will advance the state here
      if(TCNT1 > COMM_TIMEOUT) {
        state = 0;
      }
      break;   
      
    case 5: //BC detect
      //wait for zero-crossing
      //turn off the previous high side - PHASE A
      PHA_HI_PORT |= (1 << PHA_HI_PIN);
      //turn on the high side - PHASE B
      PHB_HI_PORT &= ~_BV(PHB_HI_PIN);//digitalWrite(PHA_HI, LOW);
      
      //PWM the low side - PHASE C
      PWMcycleRC_PHC_LO();
      
      //watch for the commutation zero-crossing - PHASE A
      if((TCNT1 > ZC_HOLDOFF) && (ZC_PORT & (1 << ZC_PIN))) { 
        state++;
        OCR1A = TCNT1 << 1; //double the current count
      }
      if(TCNT1 > COMM_TIMEOUT) {
        state = 0;
      }    
      break;

    case 6: //BC commute
      //wait for commutation
      //turn off the previous high side - PHASE A
      PHA_HI_PORT |= (1 << PHA_HI_PIN);
      //turn on the high side - PHASE B
      PHB_HI_PORT &= ~_BV(PHB_HI_PIN);//digitalWrite(PHA_HI, LOW);
      
      //PWM the low side - PHASE C
      PWMcycleRC_PHC_LO();
      
      //the interrupt will advance the state here
      if(TCNT1 > COMM_TIMEOUT) {
        state = 0;
      }
      break;
      
    case 7: //BA detect
      //wait for zero-crossing
      //turn off the previous high side - PHASE A
      PHA_HI_PORT |= (1 << PHA_HI_PIN);
      //turn on the high side - PHASE B
      PHB_HI_PORT &= ~_BV(PHB_HI_PIN);//digitalWrite(PHA_HI, LOW);
      
      //PWM the low side - PHASE A
      PWMcycleRC_PHA_LO();
      
      //watch for the commutation zero-crossing - PHASE C
      if((TCNT1 > ZC_HOLDOFF) && (!(ZC_PORT & (1 << ZC_PIN)))) { 
        state++;
        OCR1A = TCNT1 << 1; //double the current count
      }
      if(TCNT1 > COMM_TIMEOUT) {
        state = 0;
      } 
      break;

    case 8: //BA commute
      //wait for commutation
      //turn off the previous high side - PHASE A
      PHA_HI_PORT |= (1 << PHA_HI_PIN);
      //turn on the high side - PHASE B
      PHB_HI_PORT &= ~_BV(PHB_HI_PIN);//digitalWrite(PHA_HI, LOW);
      
      //PWM the low side - PHASE A
      PWMcycleRC_PHA_LO();
      
      //the interrupt will advance the state here
      if(TCNT1 > COMM_TIMEOUT) {
        state = 0;
      }
      break;
      
    case 9: //CA detect
      //wait for zero-crossing
      //turn off the previous high side - PHASE B
      PHB_HI_PORT |= (1 << PHB_HI_PIN);
      //turn on the high side - PHASE C
      PHC_HI_PORT &= ~_BV(PHC_HI_PIN);//digitalWrite(PHA_HI, LOW);
      
      //PWM the low side - PHASE A
      PWMcycleRC_PHA_LO();
      
      //watch for the commutation zero-crossing - PHASE B
      if((TCNT1 > ZC_HOLDOFF) && (ZC_PORT & (1 << ZC_PIN))) { 
        state++;
        OCR1A = TCNT1 << 1; //double the current count
      }
      if(TCNT1 > COMM_TIMEOUT) {
        state = 0;
      }    
      break;

    case 10: //CA commute
      //wait for commutation
      //turn off the previous high side - PHASE B
      PHB_HI_PORT |= (1 << PHB_HI_PIN);
      //turn on the high side - PHASE C
      PHC_HI_PORT &= ~_BV(PHC_HI_PIN);//digitalWrite(PHA_HI, LOW);
      
      //PWM the low side - PHASE A
      PWMcycleRC_PHA_LO();
      
      //the interrupt will advance the state here
      if(TCNT1 > COMM_TIMEOUT) {
        state = 0;
      }
      break;
      
    case 11: //CB detect
      //wait for zero-crossing
      //turn off the previous high side - PHASE B
      PHB_HI_PORT |= (1 << PHB_HI_PIN);
      //turn on the high side - PHASE C
      PHC_HI_PORT &= ~_BV(PHC_HI_PIN);//digitalWrite(PHA_HI, LOW);
      
      //PWM the low side - PHASE A
      PWMcycleRC_PHB_LO();
      
      //watch for the commutation zero-crossing - PHASE A
     if((TCNT1 > ZC_HOLDOFF) && (!(ZC_PORT & (1 << ZC_PIN)))) { 
        state++;
        OCR1A = TCNT1 << 1; //double the current count
      }
      if(TCNT1 > COMM_TIMEOUT) {
        state = 0;
      }    
      break;

    case 12: //CB commute
      //wait for commutation
      //turn off the previous high side - PHASE B
      PHB_HI_PORT |= (1 << PHB_HI_PIN);
      //turn on the high side - PHASE C
      PHC_HI_PORT &= ~_BV(PHC_HI_PIN);//digitalWrite(PHA_HI, LOW);
      
      //PWM the low side - PHASE A
      PWMcycleRC_PHB_LO();
      
      //the interrupt will advance the state here
      if(TCNT1 > COMM_TIMEOUT) {
        state = 0;
      }
      break;
      
    case 13:
      state = 1;
      if(SPEED < (TRANSITION_SPEED+1)) {
        TIMSK &= ~(1 << OCIE1A); //disable timer 1 interrupt
        state = 14;
      }
    break;
      
    //case 13 is an empty state.  This keeps the program from interpreting a mode switch as a wrap.
    //If for some odd reason we end up in state 13, it will go to default.  
    
    case 14:  //transition into Pure Sine motor controller state
      //TURN OFF EVERYTHING!
      PHA_LO_PORT &= ~_BV(PHA_LO_PIN); //low side phase A off
      PHB_LO_PORT &= ~_BV(PHB_LO_PIN); //low side phase B off
      PHC_LO_PORT &= ~_BV(PHC_LO_PIN); //low side phase C off
      PHA_HI_PORT |= _BV(PHA_HI_PIN);  //High side phase A off
      PHB_HI_PORT |= _BV(PHB_HI_PIN);  //High side phase B off
      PHC_HI_PORT |= _BV(PHC_HI_PIN);  //High side phase C off
      delay(1);
      //advance the state to start sine commutation
      state ++;
      break;
    
    case 15:
      TEST_PORT |= (1 << TEST_PIN);
      SINECOUNTER = 0;
      SPEEDCOUNTER = 0;
      while(SINECOUNTER < SINESTEPS) {
        
        //note direction has been changed here        
        A_HI_OFF = sinetable[((SINECOUNTER+PHASESTEP+PHASESTEP+PHASEOFFSET)%SINEMOD)] << 1;
        A_LO_ON = A_HI_OFF+ACB;   
        
        B_HI_OFF = sinetable[((SINECOUNTER+PHASESTEP+PHASEOFFSET)%SINEMOD)] << 1;
        B_LO_ON = B_HI_OFF+ACB;
        
        //all phases turn off at the end of a PWM cycle
        PHA_LO_PORT &= ~_BV(PHA_LO_PIN); //low side phase A off
        PHB_LO_PORT &= ~_BV(PHB_LO_PIN); //low side phase B off
        PHC_LO_PORT &= ~_BV(PHC_LO_PIN); //low side phase C off
        
        C_HI_OFF = sinetable[(SINECOUNTER+PHASEOFFSET)%SINEMOD] << 1;
        C_LO_ON = C_HI_OFF+ACB;
               
        SPEEDCOUNTER = SPEEDCOUNTER + SPEED;
        SINECOUNTER = SPEEDCOUNTER >> 9; 
        
        PWMCOUNTER_SINE = 0;
        //all phases turn on at the start of a PWM cycle
        //remember that the HI switch is inverted
        if(SPEED !=0){ //if not in shutoff
          PHA_HI_PORT &= ~_BV(PHA_HI_PIN); //High side phase A on
          PHB_HI_PORT &= ~_BV(PHB_HI_PIN); //High side phase B on
          PHC_HI_PORT &= ~_BV(PHC_HI_PIN); //High side phase C on
        }
        while(PWMCOUNTER_SINE < (PWMSTEPS_SINE+ACB)) {
          if(PWMCOUNTER_SINE==A_HI_OFF){
            PHA_HI_PORT |= _BV(PHA_HI_PIN); //High side phase A off
          }
          if(PWMCOUNTER_SINE==A_LO_ON){  //then low side phase A on
            PHA_LO_PORT |= _BV(PHA_LO_PIN);  
          }
          if(PWMCOUNTER_SINE==B_HI_OFF){  //High side phase B off
            PHB_HI_PORT |= _BV(PHB_HI_PIN);
          }
          if(PWMCOUNTER_SINE==B_LO_ON){  //then low side phase B on
            PHB_LO_PORT |= _BV(PHB_LO_PIN);
          }
          if(PWMCOUNTER_SINE==C_HI_OFF){  //High side phase C off
            PHC_HI_PORT |= _BV(PHC_HI_PIN);
          }
          if(PWMCOUNTER_SINE==C_LO_ON){  //the low side phase C on
            PHC_LO_PORT |= _BV(PHC_LO_PIN);
          }
        PWMCOUNTER_SINE++;  //increment the PWM counter
        }
        
      }
      
      if(SPEED > TRANSITION_SPEED) {
        state=0;
      }  
      break;
    
    default: //just sends you to state 0
      //TURN OFF EVERYTHING!
      /*PHA_LO_PORT &= ~_BV(PHA_LO_PIN); //low side phase A off
      PHB_LO_PORT &= ~_BV(PHB_LO_PIN); //low side phase B off
      PHC_LO_PORT &= ~_BV(PHC_LO_PIN); //low side phase C off
      PHA_HI_PORT |= _BV(PHA_HI_PIN);  //High side phase A off
      PHB_HI_PORT |= _BV(PHB_HI_PIN);  //High side phase B off
      PHC_HI_PORT |= _BV(PHC_HI_PIN);  //High side phase C off
      
      TCNT1 = 0; //reset the timer
      OCR1A = 0x03FF; //reset the timer length, about 1 second
      ADMUX = (ADMUX & 0xF8) | PHC_SENSE_MUX; //select sense line C, for starters*/
      state = 0;
      break;
      
    
  } //end the switch statement
  
}

//zero-crossing interrupt handler
ISR(TIMER1_COMPA_vect) {
  /*switch(state) { //you don't need this unless you need to speed things up, then put the on switching here too
    case 4:
      PHA_HI_PORT |= (1 << PHA_HI_PIN);
    case 8:
      PHB_HI_PORT |= (1 << PHB_HI_PIN);
    case 12:
      PHC_HI_PORT |= (1 << PHB_HI_PIN);
  }*/
  switch(state) {
    case 2:
      ADMUX = (ADMUX & 0xF8) | PHB_SENSE_MUX;
      break;
    case 4:
      ADMUX = (ADMUX & 0xF8) | PHA_SENSE_MUX;
      break;
    case 6:
      ADMUX = (ADMUX & 0xF8) | PHC_SENSE_MUX;
      break;
    case 8:
      ADMUX = (ADMUX & 0xF8) | PHB_SENSE_MUX;
      break;
    case 10:
      ADMUX = (ADMUX & 0xF8) | PHA_SENSE_MUX;
      break;
    case 12:
      ADMUX = (ADMUX & 0xF8) | PHC_SENSE_MUX;
      break;
  }
     
  state++; //advance the state
  OCR1A = 0x07FF; //reset the timer length, in preparation for the next ZC detect
  /*if (state == 13) {  //wrap state
    state = 1;
  }
  if(SPEED < (TRANSITION_SPEED+1)) {
    TIMSK &= ~(1 << OCIE1A); //disable timer 1 interrupt
    state = 14;
  }*/
    
}

//RC pulse interrupt handler
ISR(INT0_vect) {
  (RC_INT_PORT & (1 << RC_INT_PIN)) ? RCOUT_PORT |= (1 << RCOUT_PIN) : RCOUT_PORT &= ~(1 << RCOUT_PIN);
  if((MCUCR & 0x03) == 0x03) { //we detected a rising edge
    TCNT0 = 0;  //reset timer0
    RCOUT_PORT |= (1 << RCOUT_PIN); //update the test pin
    MCUCR &= ~(1 << ISC00); //switch to falling edge detection
  }
  else{ //we detected a falling edge
    rcpulse = TCNT0;
    RCOUT_PORT &= ~(1 << RCOUT_PIN); //update the test pin
    MCUCR |= (1 << ISC00);  //switch to rising edge detection
    if(rcpulse > 62 && rcpulse < 127) {
      SPEED = ((rcpulse - 62) << 1) - 1; //conversion to a 128 step PWM width
    }
    /*if(rcpulse <= 72) { //this doesn't work yet 
      SPEED = 0;
    }*/
  }  
}
  

void PWMcycleRC_PHA_LO() {
  PWMCOUNTER = 0; //reset pwm counter
  A_LO_OFF = SPEED; //CONVERT SPEED (1024ms) TO A_LO_OFF (PWMSTEPS number of cycles)
    
  if(SPEED == 0){ //the shutdown state
    PHA_HI_PORT |= _BV(PHA_HI_PIN);//digitalWrite(PHA_HI, HIGH); //remember that the HI switch is inverted
    PHA_LO_PORT &= ~_BV(PHA_LO_PIN);//digitalWrite(PHA_LO, LOW);
    return;
  }
  
  //redundant safety - make sure hi side is off
  PHA_HI_PORT |= _BV(PHA_HI_PIN);//digitalWrite(PHA_HI, HIGH);
  
  //PWM Cycle
  //turn on low side at start of cycle
  PHA_LO_PORT |= _BV(PHA_LO_PIN);  //digitalWrite(PHA_LO, HIGH);
  while(PWMCOUNTER < PWMSTEPS) {
    
    //Swtich at trigger
    if(PWMCOUNTER==A_LO_OFF){
      (ACSR & (1 << ACO)) ? ZC_PORT |= (1 << ZC_PIN) : ZC_PORT &= ~(1 << ZC_PIN); //update commutation sense at PWM ontime end
      PHA_LO_PORT &= ~_BV(PHA_LO_PIN);  //digitalWrite(PHA_LO, LOW);
    }
   
    PWMCOUNTER++;  //increment the PWM counter  
  }
  //redundant safety - phase turns off at the end of a PWM cycle
  //you need this if A_LO_OFF can ever be equal to PWMSTEPS
  PHA_LO_PORT &= ~_BV(PHA_LO_PIN);//digitalWrite(PHA_LO, LOW);
  
}

void PWMcycleRC_PHB_LO() {
  PWMCOUNTER = 0; //reset pwm counter
  B_LO_OFF = SPEED; //CONVERT SPEED (1024ms) TO A_LO_OFF (PWMSTEPS number of cycles)
    
  if(SPEED == 0){ //the shutdown state
    PHB_HI_PORT |= _BV(PHB_HI_PIN);//digitalWrite(PHA_HI, HIGH); //remember that the HI switch is inverted
    PHB_LO_PORT &= ~_BV(PHB_LO_PIN);//digitalWrite(PHA_LO, LOW);
    return;
  }
  
  //redundant safety - make sure hi side is off
  PHB_HI_PORT |= _BV(PHB_HI_PIN);//digitalWrite(PHA_HI, HIGH);
  
  //PWM Cycle
  //turn on low side at start of cycle
  PHB_LO_PORT |= _BV(PHB_LO_PIN);  //digitalWrite(PHA_LO, HIGH);
  while(PWMCOUNTER < PWMSTEPS) {
    
    //Swtich at trigger
    if(PWMCOUNTER==B_LO_OFF){
      (ACSR & (1 << ACO)) ? ZC_PORT |= (1 << ZC_PIN) : ZC_PORT &= ~(1 << ZC_PIN); //update commutation sense at PWM ontime end
      PHB_LO_PORT &= ~_BV(PHB_LO_PIN);  //digitalWrite(PHA_LO, LOW);
    }
   
    PWMCOUNTER++;  //increment the PWM counter  
  }
  //redundant safety - phase turns off at the end of a PWM cycle
  //you need this if A_LO_OFF can ever be equal to PWMSTEPS
  PHB_LO_PORT &= ~_BV(PHB_LO_PIN);//digitalWrite(PHA_LO, LOW);
  
}



void PWMcycleRC_PHC_LO() {
  PWMCOUNTER = 0; //reset pwm counter
  C_LO_OFF = SPEED; //CONVERT SPEED (1024ms) TO A_LO_OFF (PWMSTEPS number of cycles)
    
  if(SPEED == 0){ //the shutdown state
    PHC_HI_PORT |= _BV(PHC_HI_PIN);//digitalWrite(PHA_HI, HIGH); //remember that the HI switch is inverted
    PHC_LO_PORT &= ~_BV(PHC_LO_PIN);//digitalWrite(PHA_LO, LOW);
    return;
  }
  
  //redundant safety - make sure hi side is off
  PHC_HI_PORT |= _BV(PHC_HI_PIN);//digitalWrite(PHA_HI, HIGH);
  
  //PWM Cycle
  //turn on low side at start of cycle
  PHC_LO_PORT |= _BV(PHC_LO_PIN);  //digitalWrite(PHA_LO, HIGH); 
  while(PWMCOUNTER < PWMSTEPS) {
    
    //Swtich at trigger
    if(PWMCOUNTER==C_LO_OFF){
      (ACSR & (1 << ACO)) ? ZC_PORT |= (1 << ZC_PIN) : ZC_PORT &= ~(1 << ZC_PIN); //update commutation sense at PWM ontime end
      PHC_LO_PORT &= ~_BV(PHC_LO_PIN);  //digitalWrite(PHA_LO, LOW);
    }
   
    PWMCOUNTER++;  //increment the PWM counter  
  }
  //redundant safety - phase turns off at the end of a PWM cycle
  //you need this if A_LO_OFF can ever be equal to PWMSTEPS
  PHC_LO_PORT &= ~_BV(PHC_LO_PIN);//digitalWrite(PHA_LO, LOW);
  
}

void arraysetup(void){
   sinetable[0]=15;  // Put 32 step 5 bit sine table into array.
   sinetable[1]=18;  
   sinetable[2]=21;
   sinetable[3]=24;
   sinetable[4]=26;
   sinetable[5]=28;
   sinetable[6]=29;
   sinetable[7]=30;
   sinetable[8]=31; s
   sinetable[9]=30;
   sinetable[10]=29;
   sinetable[11]=28;
   sinetable[12]=26;
   sinetable[13]=24;
   sinetable[14]=21;
   sinetable[15]=18;
   sinetable[16]=15;
   sinetable[17]=12;
   sinetable[18]=9;
   sinetable[19]=6;
   sinetable[20]=4;
   sinetable[21]=2;
   sinetable[22]=1;
   sinetable[23]=0;
   sinetable[24]=0;
   sinetable[25]=0;
   sinetable[26]=1;
   sinetable[27]=2;
   sinetable[28]=4;
   sinetable[29]=6;
   sinetable[30]=9;
   sinetable[31]=12;
 }
  
