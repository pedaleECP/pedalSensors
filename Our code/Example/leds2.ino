//inputs

int potar=A0;
int sw=2;

 

// outputs
int led_ind=13;
int led_blue=9;
int led_green=10;
int led_red=11;
 
//Program variables
unsigned long alive_period = 100; //unsigned = valeur strictement positif
unsigned long otro_period = 500; //unsigned = valeur strictement positif
void setup(){

  pinMode(potar, INPUT); //configuring input pins
  pinMode(sw, INPUT);
  pinMode(led_ind, OUTPUT); //configuring output pins
  pinMode(led_blue, OUTPUT);
  pinMode(led_green, OUTPUT);
  pinMode(led_red, OUTPUT);

  digitalWrite(led_ind, LOW);  //Give default values to output pins
  digitalWrite(led_blue, LOW);
  digitalWrite(led_green, LOW);
  digitalWrite(led_red, LOW);
  


}

void i_m_alive_loop(){
  static unsigned long last_time = 0;
  static boolean alive_state = true; ;
  unsigned long cur_time = millis();
  if (cur_time - last_time > alive_period) {
    if (alive_state) {
      digitalWrite(led_ind,HIGH);
    }
    else {
      digitalWrite(led_ind,LOW);
    }
    alive_state =!alive_state;
    last_time = cur_time; //Keep track of tim passing by
  }
  
}

  int blink_state=0;

void otro_loop(){
  

  static unsigned long last_time = 0;
  unsigned long cur_time = millis();
  if (cur_time - last_time > otro_period) {
    
      digitalWrite(led_blue, LOW);
  digitalWrite(led_green, LOW);
  digitalWrite(led_red, LOW);
  
  switch (blink_state){
      case 0:
         digitalWrite(led_blue, HIGH);
         blink_state=1;
         break;

      case 1:
         digitalWrite(led_red, HIGH);
         blink_state=2;
         break;

      case 2:
         digitalWrite(led_green, HIGH);
         blink_state=0;
         break;

      default:
         blink_state=0;
         break;
  }
   
    blink_state= (blink_state +1) % 3;
    
    last_time = cur_time; //Keep track of tim passing by
  }
}


void loop(){

 i_m_alive_loop(); 
 otro_loop();

}
