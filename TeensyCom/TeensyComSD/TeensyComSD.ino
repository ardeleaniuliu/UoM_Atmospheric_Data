
#include <ADC.h>
#include <ADC_Module.h>
#include <RingBuffer.h>
#include <RingBufferDMA.h>
#include <SD.h> //Load SD card library
#include<SPI.h> //Load SPI Library

#include <BK_mavlink.h> // Load mavlink with the new uom_atmospheric_data message defined
 
// set this to the hardware serial port you wish to use
#define APSERIAL Serial1 //Autopilot Port RX(pin0) TX(pin1) 
#define TELEMSERIAL Serial3 //Telemetry Serial RX(pin7) TX(pin8)
//#define TELEMSERIAL Serial

const int OP1pin = A2; //pin16
const int OP2pin = A3; //pin17

ADC *adc = new ADC();

int system_id=0;
int component_id=0;
float roll=0;
float pitch=0;
float yaw=0;
float rollspeed=0;
float pitchspeed=0;
float yawspeed=0;
uint32_t time_boot_ms;
int lat=0;
int lon=0;
int alt=0;
int relative_alt=0;
int vx=0;
int vy=0;
int vz=0;
int hdg=0;
float airspeed=0;
float groundspeed=0;
float press_abs=0;
float press_diff=0;
int temperature=0;
float OP1=0;
float OP2=0;
float CO=0;
int firstheartbeat=1;
    
unsigned long previousMillis = 0;        // will store last time atmospheric data packet was sent
// constants won't change :
const long intervaal = 500;           // interval at which to send new atmospheric data packet (milliseconds)

int chipSelect = 4; //chipSelect pin for the SD card Reader
File mySensorData; //Data object you will write your sesnor data to

unsigned long lognum=0;
bool SD_Connected=0;

void setup() {
  // The setup code will run once
  pinMode(OP1pin,INPUT); // configuring OP1pin as input
  pinMode(OP2pin,INPUT); // configuring OP2pin as input

  // Initilizing serial ports
  //Serial.begin(9600);
  TELEMSERIAL.begin(57600);
  APSERIAL.begin(57600);

  pinMode(10, OUTPUT); //Must declare pin10 as an output and reserve it
  SD_Connected=SD.begin(4); //Initialize the SD card reader
  lognum=millis();// can add a lastlog file on sd card to keep track if needed

 ///// ADC0 ////
    // reference can be ADC_REF_3V3, ADC_REF_1V2 (not for Teensy LC) or ADC_REF_EXT.
    //adc->setReference(ADC_REF_1V2, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2

    adc->setAveraging(1); // set number of samples to be taken and averaged to give a reading
    adc->setResolution(12); // set bits of resolution
    adc->setReference( ADC_REF_EXT); // set the external reference pin as the voltage reference

    // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED_16BITS, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
    // see the documentation for more information
    // additionally the conversion speed can also be ADC_ADACK_2_4, ADC_ADACK_4_0, ADC_ADACK_5_2 and ADC_ADACK_6_2,
    // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
    adc->setConversionSpeed(ADC_MED_SPEED); // change the conversion speed
    // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
    adc->setSamplingSpeed(ADC_MED_SPEED); // change the sampling speed
    
    ////// ADC1 /////
    adc->setAveraging(1, ADC_1); // set number of averages
    adc->setResolution(12, ADC_1); // set bits of resolution
    adc->setConversionSpeed(ADC_MED_SPEED, ADC_1); // change the conversion speed
    adc->setSamplingSpeed(ADC_MED_SPEED, ADC_1); // change the sampling speed
    adc->setReference(ADC_REF_EXT, ADC_1);
    
    adc->startSynchronizedContinuous(OP1pin, OP2pin);


}

ADC::Sync_result result;

void loop() {
   // put your main code here, to run repeatedly:  
    int incomingByte;
    int outgoingByte;
    unsigned long currentMillis = millis();

    mavlink_message_t msg;
    mavlink_status_t status1;
    
    
  if (APSERIAL.available() > 0) {
    // Read the first byte in the AP serial port buffer
    incomingByte = APSERIAL.read();

    // if the data is a mavlink packet do the following
    if(mavlink_parse_char(MAVLINK_COMM_0, incomingByte, &msg, &status1)) {

      // Read new mavlink packet
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
      // Send the packet received from the AP to the GCS through the telemetry module
      TELEMSERIAL.write(buf, len);
      // Read the necessary data from the read packet
      switch(msg.msgid)
      {
              case MAVLINK_MSG_ID_HEARTBEAT:
              {
                system_id=msg.sysid;
                component_id=msg.compid;
                
                if (firstheartbeat == 1)
                {
                  mavlink_message_t rdsmsg;
                  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                  mavlink_msg_request_data_stream_pack(255, 0, &rdsmsg, 0, 0, 0, 20, 1);
                  int16_t len = mavlink_msg_to_send_buffer(buf, &rdsmsg);
                  APSERIAL.write(buf, len);
                  
                  firstheartbeat = 0;
                }
                break;
              }
              
              case MAVLINK_MSG_ID_ATTITUDE:
              {
                mavlink_attitude_t attitude;
                mavlink_msg_attitude_decode(&msg, &attitude);
                roll=attitude.roll;
                pitch=attitude.pitch;
                yaw=attitude.yaw;
                rollspeed=attitude.rollspeed;
                pitchspeed=attitude.pitchspeed;
                yawspeed=attitude.yawspeed;
                time_boot_ms=attitude.time_boot_ms;
                break;    
              }
              
              case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
              {
                mavlink_global_position_int_t gps;
                mavlink_msg_global_position_int_decode(&msg, &gps);
                lat=gps.lat;
                lon=gps.lon;
                alt=gps.alt;
                relative_alt=gps.relative_alt;
                vx=gps.vx;
                vy=gps.vy;
                vz=gps.vz;
                time_boot_ms=gps.time_boot_ms;
                break; 
              }
              
              case MAVLINK_MSG_ID_VFR_HUD:
              {
                mavlink_vfr_hud_t vfr;
                mavlink_msg_vfr_hud_decode(&msg, &vfr);
                airspeed=vfr.airspeed;
                groundspeed=vfr.groundspeed;
                hdg=vfr.heading;
                break; 
              }
              
              case MAVLINK_MSG_ID_SCALED_PRESSURE:
              {
                mavlink_scaled_pressure_t pressure;
                mavlink_msg_scaled_pressure_decode(&msg, &pressure);
                press_abs=pressure.press_abs;
                press_diff=pressure.press_diff;
                temperature=pressure.temperature;
                time_boot_ms=pressure.time_boot_ms;
                break; 
              }
              
      }
    }
  }


    result = adc->readSynchronizedContinuous();
    // if using 16 bits and single-ended is necessary to typecast to unsigned,
    // otherwise values larger than 3.3/2 will be interpreted as negative
    result.result_adc0 = (uint16_t)result.result_adc0; 
    result.result_adc1 = (uint16_t)result.result_adc1;

    
    //ADC::Sync_result result = adc->analogSynchronizedRead(OP1pin, OP2pin);
    //Modify the calibration value according to the sensor you are using
    float CalibVal=0.439; //Calibration value to convert from voltage into ppb
    OP1=((((float)result.result_adc0/adc->getMaxValue(ADC_0))*5)/CalibVal); //Converting the raw bit value into ppb equivalent 
    OP2=((((float)result.result_adc1/adc->getMaxValue(ADC_1))*5)/CalibVal); //Converting the raw bit value into ppb equivalent

    CO= OP1 - OP2 ; // Claculating the CO value in ppb
    
    //Serial.println(", OP1 " + String(OP1)+ ", OP2 " + String(OP2)+ ", CO " + String(CO));


  

  if (currentMillis - previousMillis >= intervaal && system_id>0) {
    // save the last time you recorded an atmospheric measurement
    previousMillis = currentMillis;

    // Initialize the required buffers 
    mavlink_message_t atm; 
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    // Pack the message
    mavlink_msg_uom_atmospheric_data_pack( system_id, component_id, &atm,
                    roll, pitch,yaw, rollspeed,  pitchspeed, yawspeed, time_boot_ms, lat, lon, 
                    alt, relative_alt, vx, vy, vz, airspeed, groundspeed, hdg, press_abs, 
                    press_diff, temperature, OP1, OP2, CO);
                    
    // Copy the message to send buffer 
    uint16_t len = mavlink_msg_to_send_buffer(buf, &atm);
    
    // Send the uom_atmospheric_data_pack to the GCS through the telemetry module 
    TELEMSERIAL.write(buf, len);


    // Save the information sent in the mavlink packet on the SD card
    if (SD_Connected){ //log the data only if the SD card is conneted
          mySensorData = SD.open("Log.txt", FILE_WRITE); 
          if (mySensorData) {
          
          mySensorData.println("Roll "+ String(roll) +", Pitch "+ String(pitch) + ", Yaw " + String(yaw) + ", RollSpeed " + String(rollspeed)
                      + ", PitchSpeed " + String(pitchspeed) + ", YawSpeed" + String(yawspeed) + ", Time_boot_ms " + String(time_boot_ms) + ", Lat " + String(lat)
                      + ", Lon " + String(lon) +", Alt " + String(alt) + ", RelativeAlt " + String(relative_alt) + ", vx " + String(vx) + ", vy " + String(vy) 
                      + ", vz" + String(vz) + ", Heading " + String(hdg) + ", Airspeed " + String(airspeed) + ", GroundSpeed " + String(groundspeed)
                      + ", Press_Abs " + String(press_abs) + ", PressDiff " + String(press_diff)+ ", Temperature " + String(temperature) 
                      + ", OP1 " + String(OP1)+ ", OP2 " + String(OP2)+ ", CO " + String(CO));

/*          mySensorData.print("OP1: ");   
          mySensorData.print(OP1);                             //write OP1 data to card
          mySensorData.print(", ");                               //write a commma
          mySensorData.print("OP2: ");   
          mySensorData.print(OP2);                             //write OP2 data to card
          mySensorData.print(", "); 
          mySensorData.print("CO: ");   
          mySensorData.println(CO);                             //write CO data and end the line (println)
*/          
          mySensorData.close();                                  //close the file
          }
    }
    
  }


//Take the GCS inputs from the telemetry module and transfer them to the AP port
  if (TELEMSERIAL.available() > 0){
    outgoingByte = TELEMSERIAL.read();
    APSERIAL.write(outgoingByte);
  }
  



   
}
