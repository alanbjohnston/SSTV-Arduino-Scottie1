/**
 * @author: Fran Acién and David Arias, with the help of Pablo Alvarez and Dani Kholer
 *  SSTV emitter using arduino DUE
 *
**/

#include "SSTV-Arduino-Scottie1-Library.h"
//#include <Arduino.h>
#include "RPi_Pico_TimerInterrupt.h"

RPI_PICO_Timer dds_ITimer0(2);
RPI_PICO_Timer sstv_ITimer1(3);

bool dds_phase = HIGH;
int dds_duration_us = 1000;
int dds_duration_previous_us = 1000;
bool dds_enable = false;
bool sstv_stop;

//volatile uint8_t phase = 0;

char pic_filename[13];
char pic_decoded_filename[13];

uint8_t frameBuf[81920]; //320*256

volatile byte buffE[320]; // Buffer conintating Red values after torch
volatile byte buffR[320]; // Buffer conintating Red values of the line
volatile byte buffG[320]; // Buffer conintating Green values of the line
volatile byte buffB[320]; // Buffer conintating Blue values of the line

volatile byte sEm = 0;    // State of Emition
                    // 0 not emitting
                    // 1 emitting line (NOT HEADER OR VOX)
                    // 2 Change Color

volatile byte sCol = 0;   // Transmitting color Green
                    // Transmitting color Blue
                    // Transmitting color Red

volatile int tp = 0;     // Index of pixel while transmitting with timer
volatile int line;

char charId[13] = "EA4RCT-SSTV-"; // ***** INFORMATION HEADER: MAX 12 CAHARCTERS *****
volatile long syncTime;

short sstv_output_pin;

// #define AUDIO_OUT_PIN 26

void set_sstv_pin(byte pin) {
  sstv_output_pin = pin;
  pinMode(sstv_output_pin, OUTPUT);
}

void sstv_end() {
  sstv_stop = true;
}

/*
void setup() {
  
  Serial.begin(9600);
  
  delay(5000);
  
  Serial.println("Starting");
  
  pinMode(AUDIO_OUT_PIN, OUTPUT);
  
  setup_sstv();
  
}

void loop() {
  
}
*/

bool dds_TimerHandler0(struct repeating_timer *t) {  // DDS timer for waveform
  if (dds_enable) {
    dds_phase = !dds_phase;	  
//    digitalWrite(AUDIO_OUT_PIN, dds_phase);    // ToDo: if no TXC, just turn on PWM carrier
    digitalWrite(sstv_output_pin, dds_phase);    // ToDo: if no TXC, just turn on PWM carrier
  }
  return(true);
}

void dds_begin() {
  
  if (dds_ITimer0.attachInterruptInterval(dds_duration_us, dds_TimerHandler0))	{
    Serial.print(F("Starting dds_ITimer0 OK, micros() = ")); Serial.println(micros());
  }
  else
    Serial.println(F("Can't set dds_ITimer0. Select another Timer, freq. or timer"));
  
  dds_enable = true;
}

void dds_down() {
  
  dds_enable = false;
  Serial.println("Stopping");
}

void dds_setfreq(int freq) {
  
  dds_duration_us = 0.5E6 / (float)freq - 3;  // subtract 3 us of processing delay
//  Serial.println(dds_duration_us);

  if (dds_duration_us != dds_duration_previous_us) {   // only change if frequency is different
    
/*    
    if (dds_ITimer0.setInterval(dds_duration_us, dds_TimerHandler0)) {
      Serial.println(dds_duration_us);
    }
    else
      Serial.println(F("Can't set dds interval"));
*/   
    dds_ITimer0.setInterval(dds_duration_us, dds_TimerHandler0);
    dds_duration_previous_us = dds_duration_us;
  }   
}

// bool sstv_TimerHandler1(struct repeating_timer *t) {
bool sstv_TimerHandler1() {

//void timer1_interrupt(){
//  Serial.println("sstv_TimerHandler1");
  if (sEm == 1){
    if(tp < 320){  // Transmitting pixels
      if(sCol == 0){  // Transmitting color Green
//        DDS.setfreq(1500 + 3.13 * buffG[tp], phase);
        dds_setfreq(1500 + 3.13 * buffG[tp]);
      } else if(sCol == 1){ // Transmitting color Blue
        dds_setfreq(1500 + 3.13 * buffB[tp]);
//        DDS.setfreq(1500 + 3.13 * buffB[tp], phase);
      } else if(sCol == 2){ // Transmitting color Red
//        DDS.setfreq(1500 + 3.13 * buffE[tp], phase);
        dds_setfreq(1500 + 3.13 * buffE[tp]);
      }
    } else if(tp == 320){
      if(sCol == 0){  // Separator pulse after transmit Green
//        DDS.setfreq(1500, phase);
        dds_setfreq(1500);
      } else if(sCol == 1){ // Sync porch
//        DDS.setfreq(1200, phase);
        dds_setfreq(1200);
      } else if(sCol == 2){ // // Separator pulse after transmit Red
//        DDS.setfreq(1500, phase);
        dds_setfreq(1500);
      }
      syncTime = micros();
      sEm = 2;    // State when change color
    }
    tp++;
  }
 return(true);	
}

//void setup_sstv() {
void send_sstv() {
//  delay(5000);
//  pinMode(BUILT_IN_PIN, OUTPUT);
//  pinMode(SD_SLAVE_PIN, OUTPUT);
  
  sstv_stop = false;
  
  Serial.begin(9600);
  Serial.println("Starting");

  // AD9850 initilize
  //DDS.begin(AD9850_CLK_PIN, AD9850_FQ_UPDATE_PIN, AD9850_DATA_PIN, AD9850_RST_PIN);

  dds_begin();
/*  
  delay(2000);
  dds_setfreq(1200);
  Serial.println("1200");
  delay(2000); 
  dds_setfreq(1500);  
  Serial.println("1500");  
  delay(2000);
  dds_setfreq(500);  
  Serial.println("500");  
  delay(2000);
*/  
  
 /* 
  // Sd initialize
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_SLAVE_PIN)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
*/
  
/*  
  // Setup Timer with the emision interval
  // Timer1.attachInterrupt(timer1_interrupt).start(430); // ***** 354(uS/px) +/- SLANT ADJUST *****
//  if (sstv_ITimer1.attachInterruptInterval(430, sstv_TimerHandler1)) {	
  if (sstv_ITimer1.attachInterruptInterval(421, sstv_TimerHandler1)) {	
    Serial.print(F("Starting sstv_ITimer1 OK, micros() = ")); Serial.println(micros());
  }
  else
    Serial.println(F("Can't set sstv_ITimer1. Select another Timer, freq. or timer"));
*/
  ITimer0.setInterval(421);
  
  delay(100);

/*  
  shot_pic();

  Serial.print("Picture taken saved on:");
  Serial.println(pic_filename);

  strcpy(pic_decoded_filename, pic_filename);
  pic_decoded_filename[8] = 'B';
  pic_decoded_filename[9] = 'I';
  pic_decoded_filename[10] = 'N';

  Serial.print("Writting on: ");
  Serial.println(pic_decoded_filename);

  jpeg_decode(pic_filename, pic_decoded_filename);
  */

  scottie1_transmit_file(pic_decoded_filename);
}


/**
 * Get output frequency given a color component (R, G, B) from 0 to 255
 * @param uint8_t c - single color component from 0 to 255
 * @return uint16_t - scottie1 frequency
**/
uint16_t scottie_freq(uint8_t c){
  return 1500 + (c * COLORCORRECTION);
}

void vox_tone(){
  /** VOX TONE (OPTIONAL) **/
  transmit_mili(1900, 100);
  transmit_mili(1500, 100);
  transmit_mili(1900, 100);
  transmit_mili(1500, 100);
  transmit_mili(2300, 100);
  transmit_mili(1500, 100);
  transmit_mili(2300, 100);
  transmit_mili(1500, 100);
}

void scottie1_calibrationHeader(){
  /** CALIBRATION HEADER **/
  transmit_mili(1900, 300);
  transmit_mili(1200, 10);
  transmit_mili(1900, 300);
  transmit_mili(1200, 30);
  transmit_mili(1300, 30);    // 0
  transmit_mili(1300, 30);    // 0
  transmit_mili(1100, 30);    // 1
  transmit_mili(1100, 30);    // 1
  transmit_mili(1100, 30);    // 1
  transmit_mili(1100, 30);    // 1
  transmit_mili(1300, 30);    // 0
  transmit_mili(1300, 30);    // Even parity
  transmit_mili(1200, 30);    // VIS stop bit
}

/**
 * Set a frequency for a certain amount of time
 * @param int freq - Frequency
 * @param float duration - duration in microseconds
 */
void transmit_micro(int freq, float duration){
//  DDS.setfreq(freq, phase);
  dds_setfreq(freq);
  
  delayMicroseconds(duration);
}

/**
 * Set a frequency for a certain amount of time
 * @param int freq - Frequency
 * @param float duration - duration in milliseconds
 */
void transmit_mili(int freq, float duration){
//  DDS.setfreq(freq, phase);
  dds_setfreq(freq);
  delay(duration);
}

void scottie1_transmit_file(char* filename){
  /*
  Be aware that you have to read variables on sync torch due its 9 ms instead 1.5 ms of the sync Pulse
  */

  bool head;
  Serial.println("Transmitting picture");

//  File myFile = SD.open(filename);
  int myFile = true;  
  if (myFile) {
    head = true;

    /** TRANSMIT EACH LINE **/
//    while(myFile.available() || line == 255){
    while ((myFile || line == 255) && !sstv_stop) {
      if(head == true){ // Header
        /** VOX TONE (OPTIONAL) **/
        vox_tone();

        /** CALIBRATION HEADER **/
        scottie1_calibrationHeader();

        // Configure syncTime
        syncTime = micros();

        // Read line and store color values in the buffer
        for(uint16_t i = 0; i < 320; i++){
          buffR[i] =  0; // myFile.read();
          buffG[i] =  128; // myFile.read();
          buffB[i] =  0; // myFile.read();
        }

        Serial.println("+ +");
        Serial.println(micros() - syncTime); //Cheak reading time

        while ((micros() - syncTime < 9000 - 10) && !sstv_stop) {delayMicroseconds(100);}

        Serial.println("Start separator pulse");
        
        // Separator pulse
 //       DDS.setfreq(1500, phase);
        dds_setfreq(1500);
        syncTime = micros();  // Configure syncTime

        line = 0;
        head = false;
      }

      while ((micros() - syncTime < 1500 - 10) && !sstv_stop) {delayMicroseconds(100);} // Separator pulse
      Serial.println("Start green scan"); 
      // Green Scan
      tp = 0; sCol = 0; sEm = 1;
      while((sEm == 1) && !sstv_stop) {delayMicroseconds(100);};

      Serial.println("Start separator pulse");
      // Separator Pulse
 //     DDS.setfreq(1500, phase);
      dds_setfreq(1500);
      while ((micros() - syncTime < 1500 - 10) && !sstv_stop) {delayMicroseconds(100);}

      Serial.println("Start blue scan");
      // Blue Scan
      tp = 0; sCol = 1; sEm = 1;
      while ((sEm == 1) && !sstv_stop) {delayMicroseconds(100);};

//      Serial.println("Start evacuate");
      //Evacuate
      for(uint16_t i = 0; i < 320; i++){
        buffE[i] = buffR[i];
      }

      if(line != 255){
        // Read line and store color values in the buffer
        for(uint16_t i = 0; i < 320; i++){
          buffR[i] = 0; // myFile.read();
          buffG[i] = 128; // myFile.read();
          buffB[i] = 0; // myFile.read();
        }
      }

      Serial.println("--");
      Serial.println(micros() - syncTime); //Cheak reading time

      //Sync pulse
      while ((micros() - syncTime < 9000 - 10) && !sstv_stop) {delayMicroseconds(100);}
      
//      Serial.println("Starting sync porch");  
        
 // Sync porch
//      DDS.setfreq(1500, phase);
      dds_setfreq(1500);
      syncTime = micros();
      while ((micros() - syncTime < 1500 - 10) && !sstv_stop) {delayMicroseconds(100);}

//      Serial.println("Start red scan");  
      // Red Scan
      tp = 0; sCol = 2; sEm = 1;
      while ((sEm == 1) && !sstv_stop) {};

//      Serial.println("increment line");
      line++;
      if(line == 256){
        Serial.println("Finish");
//        DDS.setfreq(2, phase);
        dds_setfreq(2);
//        DDS.down();
        
        dds_down();
        
        sEm = 0;
      }
      else {
//        Serial.println("Start separator pulse");
        // Separator pulse
 //       DDS.setfreq(1500, phase);
        dds_setfreq(1500);
        syncTime = micros();
        sEm = 2;
      }
    }
    // close the file:
//    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  Serial.println("Ending SSTV");
//  sstv_ITimer1.stopTimer();
  Serial.println("SSTV timer stopped");  
}

/*
void jpeg_decode(char* filename, char* fileout){
  uint8 *pImg;
  int x,y,bx,by;
  byte sortBuf[15360]; //320(px)*16(lines)*3(bytes) // Header buffer
  int i,j,k;
  int pxSkip;

  // Open the file for writing
  File imgFile = SD.open(fileout, FILE_WRITE);

  for(i = 0; i < 15360; i++){ // Cleaning Header Buffer array
    sortBuf[i] = 0xFF;
  }

  for(i = 0; i < 12; i++){
    byte fontNumber;
    char ch;
    ch = charId[i];
    for(y = 0; y < 11; y++){
      for(x = 0; x < 8; x++){
        pxSkip = 16 + (320 * (y + 3)) + (3 * 8 * i) + (3 * x); //Width: x3

        uint8_t mask;
        mask = pow(2, 7 - x);

        if(ch >= 65 && ch <= 90){ // A to Z
                fontNumber = ch - 65;
        }
        else if(ch >= 48 && ch <= 57){ //0 to 9
                fontNumber = ch - 22;
        }
        else if(ch == '/'){fontNumber = 36;}
        else if(ch == '-'){fontNumber = 37;}
        else if(ch == '.'){fontNumber = 38;}
        else if(ch == '?'){fontNumber = 39;}
        else if(ch == '!'){fontNumber = 40;}
        else if(ch == ':'){fontNumber = 41;}
        else if(ch == ' '){fontNumber = 42;}
        else              {fontNumber = 42;}

        if((b_fonts[fontNumber][y] & mask) != 0){
          for(j = 0; j < 9; j++){
                  sortBuf[(3 * pxSkip) + j] = 0x00;
          }
        }
      }
    }
  }

  for(k = 0; k < 15360; k++){  // Adding header to the binary file
    imgFile.write(sortBuf[k]);
  }

  writeFooter(&imgFile);  //Writing first 10560 bytes (11*320*3)

  // Decoding start
  JpegDec.decode(filename,0);
  // Image Information
  Serial.print("Width     :");
  Serial.println(JpegDec.width);
  Serial.print("Height    :");
  Serial.println(JpegDec.height);
  Serial.print("Components:");
  Serial.println(JpegDec.comps);
  Serial.print("MCU / row :");
  Serial.println(JpegDec.MCUSPerRow);
  Serial.print("MCU / col :");
  Serial.println(JpegDec.MCUSPerCol);
  Serial.print("Scan type :");
  Serial.println(JpegDec.scanType);
  Serial.print("MCU width :");
  Serial.println(JpegDec.MCUWidth);
  Serial.print("MCU height:");
  Serial.println(JpegDec.MCUHeight);
  Serial.println("");

  Serial.println("Writting bin to SD");

  i = 0;
  j = 0;
  while(JpegDec.read()){
    pImg = JpegDec.pImage ;
    for(by=0; by<JpegDec.MCUHeight; by++){
      for(bx=0; bx<JpegDec.MCUWidth; bx++){
        x = JpegDec.MCUx * JpegDec.MCUWidth + bx;
        y = JpegDec.MCUy * JpegDec.MCUHeight + by;
        if(x<JpegDec.width && y<JpegDec.height){
          if(JpegDec.comps == 1){ // Grayscale
            //sprintf(str,"%u", pImg[0]);
            imgFile.write(pImg, 1);
          }else{ // RGB
            // When saving to the SD, write 16 lines on one time
            // First we write on the array 16 lines and then we save to SD
            pxSkip = ((y - (16 * j)) * 320) + x;
            sortBuf[(3 * pxSkip) + 0] = pImg[0];
            sortBuf[(3 * pxSkip) + 1] = pImg[1];
            sortBuf[(3 * pxSkip) + 2] = pImg[2];

            i++;
            if(i == 5120){ //320(px)x16(lines)
              for(k = 0; k < 15360; k++){
                imgFile.write(sortBuf[k]);
              }
              i = 0;
              j++; //15(sections)
            }
          }
        }
        pImg += JpegDec.comps ;
      }
    }
  }

  Serial.println("Bin has been written on SD");
  imgFile.close();
}

*/

void shot_pic(){
  
  return;
/*  
  // Try to locate the camera
  if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }

  for (int i = 0; i <= 10; i++){
    cam.setImageSize(VC0706_320x240);
  }

  Serial.println("Snap in 3 secs...");
  delay(3000);

  if (! cam.takePicture())
    Serial.println("Failed to snap!");
  else
    Serial.println("Picture taken!");

  // Create an image with the name IMAGExx.JPG`
  strcpy(pic_filename, "IMAGE00.JPG");
  for (int i = 0; i < 100; i++) {
    pic_filename[5] = '0' + i/10;
    pic_filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(pic_filename)) {
      break;
    }
  }

  // Open the file for writing
  File imgFile = SD.open(pic_filename, FILE_WRITE);

  // Get the size of the image (frame) taken
  uint16_t jpglen = cam.frameLength();
  Serial.print("Storing ");
  Serial.print(jpglen, DEC);
  Serial.print(" byte image.");

  int32_t time = millis();
  pinMode(8, OUTPUT);
  // Read all the data up to # bytes!
  byte wCount = 0; // For counting # of writes
  while (jpglen > 0) {
    // read 32 bytes at a time;
    uint8_t *buffer;
    uint8_t bytesToRead = min(32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
    buffer = cam.readPicture(bytesToRead);
    imgFile.write(buffer, bytesToRead);
    if(++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
      Serial.print('.');
      wCount = 0;
    }
    //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
    jpglen -= bytesToRead;
  }
  imgFile.close();

  time = millis() - time;
  Serial.println("done!");
  Serial.print(time); Serial.println(" ms elapsed");
  
*/  
}

/**     Write on a file with 11 lines the values of the GPS
 * @param dst Given an opened File stream then write data to dst.
 * @param latitude Floating point latitude value in degrees/min as received from the GPS (DDMM.MMMM)
 * @param lat N/S
 * @param longitude Floating point longitude value in degrees/min as received from the GPS (DDMM.MMMM)
 * @param lon E/W
 * @param altitude Altitude in meters above MSL
 */

/*
//void writeFooter(File* dst, nmea_float_t latitude, char lat, nmea_float_t longitude, char lon, nmea_float_t altitude){    //Write 16 lines with values
void writeFooter(File* dst){
  int x,y;
  byte sortBuf[10560]; //320(px)*11(lines)*3(bytes) // Header buffer
  int i,j,k;
  int pxSkip;

  char res[51] = "LAT: 1234.1234N     LONG: 1234.1234W     ALT:10000";

  for(i = 0; i < 10560; i++){ // Cleaning Header Buffer array
    sortBuf[i] = 0xFF;
  }

  for(i = 0; i < sizeof(res); i++){
    byte fontNumber;
    char ch;
    ch = res[i];
    for(y = 0; y < 5; y++){
      for(x = 0; x < 4; x++){
        //pxSkip = HORIZONTALOFFSET + VERSTICALOFFSET + (BITSPERWORD * i);
        //pxSkip = 16 + (320 * (y + 3)) + (4 * 2 * i) + (2 * x); Width: x2
        pxSkip = 16 + (320 * (y + 3)) + (4 * i) + x;

        // If ch is pair mask is: 11110000, if no 00001111
        uint8_t sl = (ch % 2)? 3 : 7 ;
        uint8_t mask = pow(2, sl - x);

        if(ch >= 48 && ch <=91){
          fontNumber = (ch-48)/2;
        }
        else {
          fontNumber = 22;
        }

        if((l_fonts[fontNumber][y] & mask) != 0){
          for(j = 0; j < 3; j++){
                  sortBuf[(3 * pxSkip) + j] = 0x00;
          }
        }
      }
    }
  }

  for(k = 0; k < 10560; k++){  // Adding header to the binary file
    dst->write(sortBuf[k]);
  }
}
*/
