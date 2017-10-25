#include <Arduino.h>
#include "SoftwareSerial.h"

SoftwareSerial mySerial(10, 11); // RX, TX 

#define DEBUG 1
#ifdef DEBUG
#define serial(msg)   Serial.print(msg)
#define serialn(msg) Serial.println(msg)
#else   /* DEBUG 1 */
#define serial(msg)
#define serialn(msg)
#endif  /* DEBUG 1 */

const byte ledPin = 13;
const byte my_led = 3;

const byte interruptPin = 2;
volatile byte button_state = 0;

#define A7_OK         0
#define A7_NOTOK      1
#define A7_TIMEOUT    2
#define A7_FAILURE    3

#define A7_CMD_TIMEOUT   2000
#define A7_AGPS_TIMEOUT  20000
#define A7_SMS_TIMEOUT   5000
#define A7_CALL_TIMEOUT 70000

#define N_COMMON      1
#define N_AGPS        2
#define N_SPECIAL     5
#define N_GPS         5

#define N_WAIT_GPS        1000
#define N_WAIT_SMS        2


struct GPS_A7 {
    String latitude_str;
    String longitude_str;
}; 

struct GPS_A7 GPS_data;

struct SMSmessage {
    String number;
    String date;
    String message;
};
/* PHONE NUMBER */
String my_sim       = "01226779766";
String my_mom       = "0903915863";
String phone_num1   = "01269396359";
String phone_num2   = "01646402358";
String phone_num3   = "0902938745" ;
String Khoa         = "01272701999";


String link_maps;

String inString = "";

/* MY FUNCTION */

void toggle_led(void );
void treo_sms(void);
void treo_gps(void);

void my_delay(unsigned long delay_in_ms);
void blink_led(void);
int  my_set_up(void);
int  interrupt_set_up(void);
int  board_init(void);
void button_interrupt(void);
void treo(void);

int gps_init(void);
void gps_get_data(void);
void gps_done(void);

void send_sos(void);
void sms_init(void);
void send_sms_to(String number);

void call_sos(void);
int dial(String number); 

int a7_command(const char *command, const char *expect_resp1, const char *expect_resp2, long TIMEOUT, int repetitions);
int a7_call(const char *command, const char *expect_resp1, const char *expect_resp2, long TIMEOUT);
int a7_wait(const char *expect_resp1, long TIMEOUT, int repetitions, int wait_times);
/*****************************************************************/
void 
toggle_led(void ){
  digitalWrite(ledPin, !digitalRead(ledPin));
}
void 
treo_sms(void){
  serial("__treo_SMS_");
  while(1){
    digitalWrite(ledPin, HIGH);
    unsigned long start_time = millis();
    while( millis() - start_time < 100 );
    digitalWrite(ledPin, LOW);
    while( millis() - start_time < 200 );
  }
}
void 
treo_gps(void) {
  serial("__treo_GPS_");
  while(1){
    digitalWrite(ledPin, HIGH);
    unsigned long start_time = millis();
    while( millis() - start_time < 50 );
    digitalWrite(ledPin, LOW);
    while( millis() - start_time < 100 );
    digitalWrite(ledPin, HIGH);
    while( millis() - start_time < 600 );
    digitalWrite(ledPin, LOW);
    while( millis() - start_time < 1100 );
  }
}

/*****************************************************************/
void 
my_delay(unsigned long delay_in_ms) {
  serial("__my_delay_");
  unsigned long start_time = millis();
  while( millis() - start_time < delay_in_ms );
  serialn("end_");
}
/*****************************************************************/
void 
blink_led(void){
  byte i = 5;
  serial("__blink_led_");
  while(i--){
    digitalWrite(ledPin, HIGH);
    unsigned long start_time = millis();
    while( millis() - start_time < 200 );
    digitalWrite(ledPin, LOW);
    while( millis() - start_time < 400 );
  }
  serialn("end_");
}
/*****************************************************************/
int 
my_set_up(void ) {
  serialn("__my_set_up_");
  Serial.begin(57600);
  mySerial.begin(57600);
  
  pinMode(my_led, OUTPUT);
  digitalWrite(my_led, HIGH); 
  if (board_init() == A7_OK){
    return A7_OK;
  }
  else return A7_NOTOK;
}
/*****************************************************************/
int 
interrupt_set_up(void) {
  serialn("__interrupt_set_up_");
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), button_interrupt, LOW);
}
/*****************************************************************/
int 
board_init(void) {
  serialn("__board_init_");
  my_delay(5000);

  a7_command("AT+GPSRD=0\r", "AT+GPSRD=0", "OK", A7_CMD_TIMEOUT, N_COMMON);
  serialn(inString);
  a7_command("AT+GPS=1\r", "AT+GPS=1", "OK", A7_CMD_TIMEOUT, N_COMMON);
  serialn(inString);
  a7_command("AT+AGPS=1\r", "AT+AGPS=1", "OK", A7_AGPS_TIMEOUT, N_AGPS);
  serialn(inString);
  a7_command("AT\r", "AT", "OK", A7_CMD_TIMEOUT, N_SPECIAL);
  serialn(inString);
  return A7_OK;
}
/*****************************************************************/
int 
gps_init(void){
  serialn("__gps_init_");
  int i, j;
  int start;
  a7_command("AT+GPS=1\r", "AT", "OK", A7_CMD_TIMEOUT, N_COMMON);
  a7_command("AT+GPSRD=2\r", "AT+GPSRD=2", "OK", A7_CMD_TIMEOUT, N_COMMON);
  if(a7_wait(".000,A", A7_CMD_TIMEOUT, N_GPS, N_WAIT_GPS) == A7_FAILURE){
    treo_gps();
  }
}
/*****************************************************************/
void 
gps_get_data(void) {
  serialn("__gps_get_data_");
  serialn(inString);
  
  int lat1  = inString.indexOf(".000,") + 5; serialn(lat1);
  int lat2  = inString.indexOf(",N,");       serialn(lat2);        
  int long1 = inString.indexOf(",N,") + 3;   serialn(long1);
  int long2 = inString.indexOf(",E,");       serialn(long2);

  GPS_data.latitude_str = inString.substring( lat1, lat2 );   
  serialn(GPS_data.latitude_str);
  GPS_data.longitude_str = inString.substring( long1, long2 );
  serialn(GPS_data.longitude_str);

  int latdot = GPS_data.latitude_str.indexOf(".");    serialn(latdot);
  int longdot = GPS_data.longitude_str.indexOf(".");  serialn(longdot);

  String lat_degree_str = GPS_data.latitude_str.substring(0, latdot-2 );        
  String lat_min_str = GPS_data.latitude_str.substring(latdot-2 );

  String long_degree_str = GPS_data.longitude_str.substring(0, longdot-2);        
  String long_min_str = GPS_data.longitude_str.substring(longdot-2);

  long lat_min =  lat_min_str.toFloat()/60*100000;
  long long_min = long_min_str.toFloat()/60*100000;  

  serialn(lat_min);
  serialn(long_min);
  link_maps = "https://www.google.com/maps?q=";
  link_maps = link_maps + lat_degree_str + '.' + lat_min + ',' + long_degree_str + '.' + long_min;
  serialn(link_maps);
}
/*****************************************************************/
void 
gps_done(void) {
  blink_led();
  serialn("__gps_done_");
  a7_command("AT+GPSRD=0\r", "AT+GPSRD=0", "OK", A7_CMD_TIMEOUT, N_COMMON);
}
/*****************************************************************/
void 
send_sos(void) {
   serialn("__send_sos_");
   sms_init();
   //send_sms_to(my_sim);
   //send_sms_to(my_mom); 
   send_sms_to(Khoa);   
}
/*****************************************************************/
void 
sms_init(void) {
  a7_command("AT+CMGF=1\r", "AT+CMGF=1", "OK", A7_CMD_TIMEOUT, N_COMMON);
}
/****************************************************************/
void 
send_sms_to(String number){  
  char ctrlZ[2] = { 0x1a, 0x00 };
  String sms_num_str = "AT+CMGS=\"" + number + "\"\r\n";
  String sms_str = "SOS, link google_maps: " + link_maps;
  int val_return;

  val_return = a7_command(sms_num_str.c_str(), ">", ">", A7_CMD_TIMEOUT, N_COMMON);

  if(val_return == A7_OK){
    mySerial.println(sms_str);
    mySerial.println(ctrlZ);
    if(a7_wait("OK", A7_SMS_TIMEOUT, N_COMMON, N_WAIT_SMS) == A7_FAILURE){
      treo_sms();
    }
  }  
}
/*****************************************************************/
void 
call_sos(void){
  Serial.println("__call_sos_");
  //dial(my_sim);
  //dial(my_mom);
  dial(Khoa);
  //dial(Thuan);
  /* PHONE NUMBER */
  /*
String my_sim       = "01226779766";
String phone_num1   = "01269396359";
String phone_num2   = "01646402358";
String phone_num3   = "0902938745" ;*/
}
/*****************************************************************/
int 
dial(String number) {
    char buffer[50];
    String call_str = "ATD" + number + "\r\n";

    a7_call(call_str.c_str(), "+CIEV: \"SOUNDER\",0", "+CIEV: \"SOUNDER\",0", A7_CALL_TIMEOUT);
}
/*****************************************************************/
int 
a7_command(const char *command, const char *expect_resp1, const char *expect_resp2, long TIMEOUT, int repetitions){
  serialn("__a7_command_");
  unsigned long time_begin; //= millis();
  int count = 1;
  byte returnValue = A7_NOTOK;

  while (count <= repetitions){
    toggle_led();
    inString = "";
    mySerial.println(command);
    serial("<-- ");
    serialn(command); 
    time_begin = millis();
    /*----------------------------------------------------------------------------------*/
    while( millis() - time_begin < TIMEOUT ){
      if (mySerial.available()) {
        inString += (char) mySerial.read();
      }
    }
  /*----------------------------------------------------------------------------------*/
    if ( ( inString.indexOf(expect_resp1) + 1 ) && ( inString.indexOf(expect_resp2) + 1 ) ) {
      serial("--> OK lan ");
      serialn(count);
      count++;
      returnValue = A7_OK;
    } else {  
      serialn("--> NOT OK.");
    }
  }
  
  return  returnValue;
}
/*****************************************************************/
int 
a7_call(const char *command, const char *expect_resp1, const char *expect_resp2, long TIMEOUT){
  serialn("__a7_call_");
  unsigned long time_begin; //= millis();
  byte returnValue = A7_NOTOK;

  inString = "";
  mySerial.println(command);
  serial("<-- ");
  serialn(command); 
  time_begin = millis();
  /*----------------------------------------------------------------------------------*/
  while( millis() - time_begin < TIMEOUT ){
    if (mySerial.available()) {
      inString += (char) mySerial.read();
    }
  }
  if ( ( inString.indexOf(expect_resp1) + 1 ) && ( inString.indexOf(expect_resp2) + 1 ) ) {
    serialn("--> OK.");
    returnValue = A7_OK;
  } else {  
    serialn("--> NOT OK.");
  }
  /*----------------------------------------------------------------------------------*/
  
  return  returnValue;
}
/*****************************************************************/
int 
a7_wait(const char *expect_resp1, long TIMEOUT, int repetitions, int wait_times){
  //int wait_times = N_WAIT;
  serialn("__a7_wait_");
  unsigned long time_begin; //= millis();
  int count = 1;
  byte returnValue = A7_NOTOK;

  while (count <= repetitions){
    toggle_led();
    inString = "";
    time_begin = millis();
    /*----------------------------------------------------------------------------------*/
    while( millis() - time_begin < TIMEOUT ){
      if (mySerial.available()) {
        inString += (char) mySerial.read();
      }
    }
  /*----------------------------------------------------------------------------------*/
    if (( inString.indexOf(expect_resp1) + 1 )) {
      serial("--> OK lan ");
      serialn(count);
      count++;
      returnValue = A7_OK;
    } else {  
      serial("--> NOT OK. count down = ");
      serialn(wait_times);
      serialn(inString);
    }
    if(wait_times-- == 0){
      return A7_FAILURE;
    }
  }  
  return  returnValue;
}
/*****************************************************/
/*****************************************************************/
/***************************************************************************/
/*****************************************************************/
/*****************************************************/
void 
button_interrupt(void) {
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    static int count = 0;
    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if (interrupt_time - last_interrupt_time > 200) {    
      count++;
      //... do your thing
      Serial.print("__button_interrupt_");
      Serial.println(count);
      button_state = !button_state;        
    }
    last_interrupt_time = interrupt_time;
}
/*****************************************************************/
void 
setup() {
  blink_led();
  // put your setup code here, to run once:
  serial("\n\n --- PROJECT --- \n\n"); 
  my_set_up();
  interrupt_set_up();
  gps_init();
  blink_led();
  blink_led();
  blink_led();
}
/*****************************************************************/
void 
loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(ledPin, HIGH);
  serialn("__loop_ ");
  if(button_state != 0){
    blink_led();
    serialn("__SOS_");
    gps_init();
    //inString = "+GPSRD:$GPGGA,091113.000,1048.49715,N,10638.98760,E,1,06,2.2,50.7,M,,M,,0000*72\n$GPRMC,091113.000,A,1048.49715,N,10638.98760,E,0.00,0.00,010817,,,A*65\n$GPVTG,0.00,T,,M,0.00,N,0.00,K,A*3D";  
    gps_get_data();
    gps_done();    
    send_sos();
    my_delay(500);
    call_sos();
    my_delay(10000);
  }

}