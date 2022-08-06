/**
 * @author: Fran Acién and David Arias, with the help of Pablo Alvarez and Dani Kholer
 *  SSTV emitter using arduino DUE
 *
**/

#include "SSTV-Arduino-Scottie1-Library.h"
//#include <Arduino.h>
#include "RPi_Pico_TimerInterrupt.h"
#include <LittleFS.h>
#include <TJpg_Decoder.h>

//#define DEBUG
#define DDS_ALT

RPI_PICO_Timer dds_ITimer0(2);
RPI_PICO_Timer sstv_ITimer1(3);

volatile bool dds_phase = HIGH;
volatile int dds_duration_us = 1000;
volatile int dds_duration = 100;  // 10 us
int dds_duration_previous_us = 1000;
volatile bool dds_enable = false;
volatile long dds_counter = 0;
bool sstv_stop;

//volatile uint8_t phase = 0;

char pic_filename[] = "/cam.jpg";
char pic_decoded_filename[] = "/cam.bin";

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
int blocks = 0;
//int counter = 0;
bool write_complete = false;
File inFile;
File outFile;

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
#ifdef DDS_ALT
    if (dds_counter++ > dds_duration_us) {
      dds_counter = 0;
      dds_phase = !dds_phase;	  
      digitalWrite(sstv_output_pin, dds_phase);    // ToDo: use PWM to approximate sin wave
    }
#else    
    dds_phase = !dds_phase;	  
//    digitalWrite(AUDIO_OUT_PIN, dds_phase);    
    digitalWrite(sstv_output_pin, dds_phase);    
#endif    
  }
  return(true);
}

void dds_begin() {
#ifdef DDS_ALT
  dds_counter = 0;
  if (dds_ITimer0.attachInterruptInterval(10, dds_TimerHandler0))	{
#else
  if (dds_ITimer0.attachInterruptInterval(dds_duration_us, dds_TimerHandler0))	{
#endif
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
#ifdef DDS_ALT  
//  dds_duration_us = (0.5E5 / (float)freq) * 1.064 - 17.8;  // 10 us with calibration
//  dds_duration_us = 0.375E5 / (float)freq;  // 10 us calibrated scaled
  dds_duration_us = 0.355E5 / (float)freq;  // 10 us calibrated scaled
//  dds_duration_us = 0.5E5 / (float)freq - 15;  // 10 us calibrated with delta
#else
  dds_duration_us = 0.5E6 / (float)freq - 3;  // subtract 3 us of processing delay
#endif
  //  Serial.println(dds_duration_us);

  if (dds_duration_us != dds_duration_previous_us) {   // only change if frequency is different
    
/*    
    if (dds_ITimer0.setInterval(dds_duration_us, dds_TimerHandler0)) {
      Serial.println(dds_duration_us);
    }
    else
      Serial.println(F("Can't set dds interval"));
*/   
#ifndef DDS_ALT
    dds_ITimer0.setInterval(dds_duration_us, dds_TimerHandler0);
#endif
    dds_duration_previous_us = dds_duration_us;
  }   
}

bool sstv_TimerHandler1(struct repeating_timer *t) {

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
void send_sstv(char* filename) {
//  delay(5000);
//  pinMode(BUILT_IN_PIN, OUTPUT);
//  pinMode(SD_SLAVE_PIN, OUTPUT);
  
  sstv_stop = false;
  
  Serial.begin(9600);
  Serial.println("Starting");

  // AD9850 initilize
  //DDS.begin(AD9850_CLK_PIN, AD9850_FQ_UPDATE_PIN, AD9850_DATA_PIN, AD9850_RST_PIN);

  dds_begin();
/**/  
  delay(2000);
  dds_setfreq(1200);
  Serial.println("1200");
  delay(2000); 
  dds_setfreq(1500);  
  Serial.println("1500");  
  delay(2000);
  dds_setfreq(2300);  
  Serial.println("2400");  
  delay(2000);
/**/  
  
  LittleFS.begin();
 /* 
  // Sd initialize
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_SLAVE_PIN)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
*/
  // Setup Timer with the emision interval
  // Timer1.attachInterrupt(timer1_interrupt).start(430); // ***** 354(uS/px) +/- SLANT ADJUST *****
//  if (sstv_ITimer1.attachInterruptInterval(430, sstv_TimerHandler1)) {	
  if (sstv_ITimer1.attachInterruptInterval(421, sstv_TimerHandler1)) {	
    Serial.print(F("Starting sstv_ITimer1 OK, micros() = ")); Serial.println(micros());
  }
  else
    Serial.println(F("Can't set sstv_ITimer1. Select another Timer, freq. or timer"));
  
  delay(100);

/*  
  shot_pic();
*/
  strcpy(pic_filename, filename);
  
  Serial.print("Sending the image ");
  Serial.println(pic_filename);

//  strcpy(pic_decoded_filename, pic_filename);
//  pic_decoded_filename[8] = 'B';
//  pic_decoded_filename[9] = 'I';
//  pic_decoded_filename[10] = 'N';

  Serial.print("Writing to: ");
  Serial.println(pic_decoded_filename);
  
//  jpeg_decode(pic_filename, pic_decoded_filename);
  
  raw_decode(pic_filename, pic_decoded_filename);
  
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

  char buff[3];
  bool head;
  Serial.println("Transmitting picture");

//  File myFile = SD.open(filename);
  File myFile = LittleFS.open(pic_decoded_filename, "r");  
  
  Serial.println(myFile);
//  int myFile = true;  
  if (myFile.available()) {
    head = true;
    Serial.println("Sending header");
    
    /** TRANSMIT EACH LINE **/
//    while(myFile.available() || line == 255){
    while(myFile.available() || line == 255){
    while ((myFile.available() || line == 255) && !sstv_stop) {
      if(head == true) { // Header
        /** VOX TONE (OPTIONAL) **/
        vox_tone();

        /** CALIBRATION HEADER **/
        scottie1_calibrationHeader();

        // Configure syncTime
        syncTime = micros();
        
        // Read line and store color values in the buffer
        for(uint16_t i = 0; i < 320; i++){
          
          myFile.readBytes(buff, 3);
          
          buffR[i] =  buff[0];
          buffG[i] =  buff[1];
          buffB[i] =  buff[2];
/*          
          Serial.print(buff[0], HEX);
          Serial.print(" ");
          Serial.print(buff[1], HEX);
          Serial.print(" ");
          Serial.print(buff[2], HEX);
          Serial.println(" ");
*/        }

        Serial.println("++");
        Serial.println(micros() - syncTime); //Cheak reading time

        while ((micros() - syncTime < 9000 - 10) && !sstv_stop) {}

//        Serial.println("Start separator pulse");
        
        // Separator pulse
 //       DDS.setfreq(1500, phase);
        dds_setfreq(1500);
        syncTime = micros();  // Configure syncTime

        line = 0;
        head = false;
      }

      while ((micros() - syncTime < 1500 - 10) && !sstv_stop) {} // Separator pulse
//      Serial.println("Start green scan"); 
      // Green Scan
      tp = 0; sCol = 0; sEm = 1;
      while((sEm == 1) && !sstv_stop) {};

//      Serial.println("Start separator pulse");
      // Separator Pulse
 //     DDS.setfreq(1500, phase);
      dds_setfreq(1500);
      while ((micros() - syncTime < 1500 - 10) && !sstv_stop) {}

//      Serial.println("Start blue scan");
      // Blue Scan
      tp = 0; sCol = 1; sEm = 1;
      while ((sEm == 1) && !sstv_stop) {}

//      Serial.println("Start evacuate");
      //Evacuate
      for(uint16_t i = 0; i < 320; i++){
        buffE[i] = buffR[i];
      }

      if(line != 255){
        // Read line and store color values in the buffer
        for(uint16_t i = 0; i < 320; i++){
         if (myFile.readBytes(buff, 3) == 0) {
//           Serial.println("Problem reading from file");
           buff[0] = 0;
           buff[1] = 255;
           buff[2] = 0;
         } 
          buffR[i] =  buff[0];
          buffG[i] =  buff[1];
          buffB[i] =  buff[2];
#ifdef DEBUG   
          Serial.print(buff[0], HEX);
          Serial.print(" ");
          Serial.print(buff[1], HEX);
          Serial.print(" ");
          Serial.print(buff[2], HEX);
          Serial.println(" ");
#endif
        }
      }

      Serial.println("--");
      Serial.println(micros() - syncTime); //Cheak reading time

      //Sync pulse
      while ((micros() - syncTime < 9000 - 10) && !sstv_stop) {}
      
//      Serial.println("Starting sync porch");  
        
 // Sync porch
//      DDS.setfreq(1500, phase);
      dds_setfreq(1500);
      syncTime = micros();
      while ((micros() - syncTime < 1500 - 10) && !sstv_stop) {}

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
    myFile.close();
  }
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening cam.bin");          
  }
  Serial.println("Ending SSTV");
  sstv_ITimer1.stopTimer();
  Serial.println("SSTV timer stopped");  
}

void print_hex(byte octet) {
      char hexValue[5];
      sprintf(hexValue, "%02X", octet);
      Serial.print(hexValue); 
}

bool get_block(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
/*  
  Serial.println("\nBlock callback");
  Serial.println(x);
  Serial.println(y);
  Serial.println(w);
  Serial.println(h);
  Serial.println(sizeof(*bitmap));
*/
  
//  return 1;

  uint16_t pixel_value;
  uint16_t *pixel;
  bool last_block = ((x == (320 - w)) & (y == (240 - h)));
  char buffer[16 * 8 * 3];
  int counter = 0;
  
/*  
  if (((y % h) == 0) && ((x % w) == 0)) {
    Serial.print("\nStart of row! x = ");
    Serial.print(x);
    Serial.print(" y = ");
    Serial.println(y);
  }
*/
  pixel = bitmap;

  uint32_t total_pixels = w * h;

  while (total_pixels--) {

    pixel_value = *pixel;
/*
    if ((x == 0) && (y == 0)) {
      Serial.print(" ");
      Serial.print(pixel_value, HEX);
      Serial.print(" ");
    }
*/    
//    buffer[counter++] = pixel_value >> 8;
//    buffer[counter++] = pixel_value;

    byte red = (pixel_value & 0b1111100000000000) >> 8;
    byte green = (pixel_value & 0b0000011111100000) >> 3;
    byte blue = (pixel_value & 0b0000000000011111) << 3;
    
    buffer[counter++] = red;
    buffer[counter++] = green;
    buffer[counter++] = blue;
    
#ifdef DEBUG    
    print_hex(red);
    print_hex(green);
    print_hex(blue);
#endif    
 /*
    if (counter >= 155000) {
      Serial.println("Resetting counter****************************************\n");
      counter = 0;
    }
*/
    pixel++;
  }
  
//  Serial.println("\nWriting block to file");
//  Serial.print("Sizeof buffer: ");
//  Serial.println(sizeof(buffer));
  
  if (inFile)
    inFile.write(buffer, sizeof(buffer));  
  else
    Serial.println("Problem writing block");
 
  if (last_block) {
    Serial.println("Complete!\n\n");
  }
/*
    for (int i = 0; i < counter; i++) {
//      Serial.print(buffer[i], HEX);
      char hexValue[5];
      sprintf(hexValue, "%02X", buffer[i]);
      Serial.print(hexValue);
    }
 */ 
    
//    Serial.print("\n\n Size: ");
//    Serial.println(counter);
    
//    write_complete = true;
//  }

//  delay(1000);

  blocks++;

  return 1;
}

void jpeg_decode(char* filename, char* fileout){
  uint8_t *pImg;
//  uint16_t *pImg;
  int x,y,bx,by;
  byte sortBuf[15360]; //320(px)*16(lines)*3(bytes) // Header buffer
  int i,j,k;
  int pxSkip;

  // Open the file for writing
//  File imgFile = SD.open(fileout, FILE_WRITE);
  outFile = LittleFS.open(fileout, "w+");
  
  if (outFile)
    Serial.println("Output opened");
  else
    Serial.println("Failed to open output");
  
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
/*  
  Serial.println("Writing header");
//  for(k = 0; k < 15360; k++){  // Adding header to the binary file
//    imgFile.write(sortBuf[k], sizeof(sortBuf));
    imgFile.write(sortBuf, sizeof(sortBuf));
//  }
*/
  //writeFooter(&imgFile);  //Writing first 10560 bytes (11*320*3)

  // Decoding start
  
  Serial.println("Starting jpeg decode");
  
  uint16_t w = 0, h = 0;
  TJpgDec.getFsJpgSize(&w, &h, "/cam.jpg", LittleFS); // Note name preceded with "/"
  Serial.print("Width = "); 
  Serial.print(w); 
  Serial.print(", height = "); 
  Serial.println(h);
  
  if ((w == 0) && (h == 0)) {
    Serial.println("Failed to open jpeg input");
    return;
  }
//  counter = 0;
//  write_complete = false;
  
  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(false);  
  TJpgDec.setCallback(get_block);  
  TJpgDec.drawFsJpg(0, 0, "/cam.jpg", LittleFS);
  
  Serial.println("Draw complete");
  
//  while (!write_complete) { Serial.println("Waiting..."); delay(500);}

/*  
  JpegDec.decodeFile(filename);
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
*/
  Serial.println("Writing bin to FS");

//  imgFile.write(JpegDec.pImage, sizeof(JpegDec.pImage));
  
/*  
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
            imgFile.write(pImg, sizeof(pImg));
          }else{ // RGB
            // When saving to the SD, write 16 lines on one time
            // First we write on the array 16 lines and then we save to SD
            pxSkip = ((y - (16 * j)) * 320) + x;
            sortBuf[(3 * pxSkip) + 0] = pImg[0];
            sortBuf[(3 * pxSkip) + 1] = pImg[1];
            sortBuf[(3 * pxSkip) + 2] = pImg[2];

            i++;
            if(i == 5120){ //320(px)x16(lines)
//              for(k = 0; k < 15360; k++){
                imgFile.write(sortBuf, sizeof(sortBuf));
//              }
              i = 0;
              j++; //15(sections)
            }
          }
        }
        pImg += JpegDec.comps ;
      }
    }
  }
*/
  Serial.println("Bin has been written to FS");
  outFile.close();
}

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

//  for(k = 0; k < 10560; k++){  // Adding header to the binary file
//    dst->write(sortBuf[k]);
    dst->write(sortBuf, sizeof(sortBuf));
//  }
}
*/

void raw_decode(char* filename, char* fileout){  // used to decode .raw files in RGB565 format

// Open the input file for reading
  inFile = LittleFS.open(filename, "r");
  
  if (inFile)
    Serial.println("Input opened");
  else {
    Serial.println("Failed to open input");
    return;
  }
// Open the output file for writing
  outFile = LittleFS.open(fileout, "w+");
  
  if (outFile)
    Serial.println("Output opened");
  else {
    Serial.println("Failed to open output");
    return;
  }
  char buff[2];
  char buffer[3];
  
  int i = 0;
  int redx = 128;
  int greenx = 128;
  int bluex = 128;
  
//  while (i++ < (320 * 240 * 3)) {
  while (i++ < (320 * 240 * 1.49)) {
    inFile.readBytes(buff, 2);
    
#ifdef DEBUG    
    print_hex(buff[0]);
    print_hex(buff[1]);
#endif
    
    int pixel_value = (buff[0] << 8) + buff[1];  // swap endian back

    byte red = (pixel_value & 0b1111100000000000) >> 8;
    byte green = (pixel_value & 0b0000011111100000) >> 3;
    byte blue = (pixel_value & 0b0000000000011111) << 3;

    int size = 3; // 46;
    int y = (int)( i / 320 );
    int x = (int)( i - y * 320 );
    int box = (int)(x/size) + (int)(y/size); 
//    int box = (int)(x/size);
  /*
    redx += 50;
    if (redx > 255) {
      redx -= 255;
      greenx += 50;
      if (greenx > 255) {
        greenx -= 255;
        bluex += 50;
        if (bluex > 255) {
          bluex -= 255;
        }
      }
    }
*/    
 
    if (y < 10) { // 20) {
      red = 0;
      green = 255;
      blue = 0;
    }   
    else if ( box == ( (int)(box / 2) * 2)) {
//      Serial.println(x);
//      Serial.println(y);
//      Serial.println(box);
//      Serial.println(" ");
      red = 255; //(100 + x) % 256;
      green = 100; // ;
      blue = 255;    
    } else  {
//      Serial.println(x);
//      Serial.println(y);
//      Serial.println(box);
//      Serial.println(" ");
      red = 100;
      green = 25;
      blue = 80; //(100 + y) % 256;
    }  
    
      buffer[0] = red;
      buffer[1] = green;
      buffer[2] = blue;    
/*     
    if (y < 20) { // 20) {
      buffer[0] = 0;
      buffer[1] = 255;
      buffer[2] = 0;
    } else {
    
      buffer[0] = redx;
      buffer[1] = greenx;
      buffer[2] = bluex;
    }
 */
    
    int bytes = outFile.write(buffer, 3);
//    Serial.println(bytes);
    if (bytes < 3) 
      Serial.println("Error writing output file");
    
  #ifdef DEBUG    
    print_hex(red);
    print_hex(green);
    print_hex(blue);
    
//    delay(100);
  #endif    
  }
  inFile.close();
  outFile.close();
}

