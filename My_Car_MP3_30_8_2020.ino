
/***************************************************
 * Requires circa 1 - 10 farad ultra-capacitors on the arduino-pro-mini side for correct function, a power diode stops it bleeding back to the .mp3 which is feeding A0 pin.
 * A0 pin has a 10K Ohm resistor from the MP3 power side, a 100K Ohm pull down resistor connected to th neg' side.
 * 
 * A basic MP3 player for a car, that was never designed for a radio.
 * connect to an amplifier for enhanced sound.  Anything better than the single MOSFET i used should be fine.
 * in this version i attempted to overcome its habit of not playing the next track, but i don't know how to read the 'Finished playing' from the serial... 
 * I derived this code from ...
 * 
 DFPlayer - A Mini MP3 Player For Arduino
 <https://www.dfrobot.com/index.php?route=product/product&product_id=1121>

 ***************************************************
 This example shows the all the function of library for DFPlayer.

 Created 2016-12-07
 By [Angelo qiao](Angelo.qiao@dfrobot.com)

 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
 1.Connection and Diagram can be found here
<https://www.dfrobot.com/wiki/index.php/DFPlayer_Mini_SKU:DFR0299#Connection_Diagram>
 2.This code is tested on Arduino Uno, Leonardo, Mega boards.
 ****************************************************/
#include <EEPROM.h>
#include <Flash.h>
#include "EEPROMAnything.h"
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

# define ACTIVATED LOW

SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);
const int buttonPause = 3;
const int buttonVolPlus = 4;
const int buttonVolMinus = 2;
const int buttonNext = 6;
const int buttonPrev = 7;
const int buttonFolderNext = 8;
const int buttonFolderPrev = 5;
int currentFileNumber;
int readCurrentFileNumber(uint8_t device);
int thisChar;

int buttonPauseState = 0;
int lastPauseButtonState = 0;
int A0State = 0;        // current state of the button
int lastA0State = 0;    // previous state of the button

boolean isPlaying = true;

int trackCount = 0;   
int oldtrackCount = 0;     
int folderCount = 0;   
int oldfolderCount = 0;
int trackAdvance = 0;

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

struct config_t
{
    long thisChar;
    int mode;
} configuration;

void setup()
{

EEPROM_readAnything(0, thisChar);
# define Start_Byte 0x7E
# define Version_Byte 0xFF
# define Command_Length 0x06
# define End_Byte 0xEF
# define Acknowledge 0x00 //Returns info with command 0x41 [0x01: info, 0x00: no info]

  pinMode(A0, INPUT);
  digitalWrite(A0,LOW);
  
  pinMode(buttonPause, INPUT);
  digitalWrite(buttonPause,HIGH);

  pinMode(buttonVolPlus, INPUT);
   digitalWrite(buttonVolPlus,HIGH);

  pinMode(buttonVolMinus, INPUT);
  digitalWrite(buttonVolMinus,HIGH);

  pinMode(buttonNext, INPUT);
  digitalWrite(buttonNext,HIGH);

  pinMode(buttonPrev, INPUT);
  digitalWrite(buttonPrev,HIGH);

  pinMode(buttonFolderNext, INPUT);
  digitalWrite(buttonFolderNext,HIGH);

  pinMode(buttonFolderPrev, INPUT);
  digitalWrite(buttonFolderPrev,HIGH);


  mySoftwareSerial.begin(9600);
  Serial.begin(9600);
  
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  inputString.reserve(200);   // reserve 200 bytes for the inputString:
  
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms

  //----Set volume----
  myDFPlayer.volume(05);  //Set volume value (0~30).
  myDFPlayer.volumeUp(); //Volume Up
  myDFPlayer.volumeDown(); //Volume Down
//  myDFPlayer.play(thisChar);
// This would be 'play(001)' or something but i was trying to make a start-up from where i left off feature
// It didn't work as i'd like, as i cannot overcome the track and folder selection system not liking my tracks & folders, or something.
// It does however, have the stupidest work-arond for that.  The last track it finished playing is recorded as a number  Skipping every track
// and subtracting one digit from the number until it reaches zero. Then it happens to be somewhere around the place it left off.  Usually.
// If you turn the player on but don't let it finish the track, it'll think the last recorded track was track 0, as it erased all the prior 
// digits, one at a time.  I suspect this is hard on the EEPROM, and i have no wear leveling that I know of so.... be careful.
// Don't use this code to run anything important.

  //----Set different EQ----
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
//  myDFPlayer.EQ(DFPLAYER_EQ_POP);
//  myDFPlayer.EQ(DFPLAYER_EQ_ROCK);
//  myDFPlayer.EQ(DFPLAYER_EQ_JAZZ);
//  myDFPlayer.EQ(DFPLAYER_EQ_CLASSIC);
//  myDFPlayer.EQ(DFPLAYER_EQ_BASS);

  //----Set device we use SD as default----
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_U_DISK);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_AUX);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SLEEP);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_FLASH);

  //----Mp3 control----
//  myDFPlayer.sleep();     //sleep
//  myDFPlayer.reset();     //Reset the module
//  myDFPlayer.enableDAC();  //Enable On-chip DAC
//  myDFPlayer.disableDAC();  //Disable On-chip DAC
//  myDFPlayer.outputSetting(true, 15); //output setting, enable the output and set the gain to 15


  //----Read imformation----
  Serial.println(myDFPlayer.readState()); //read mp3 state
  Serial.println(myDFPlayer.readVolume()); //read current volume
  Serial.println(myDFPlayer.readEQ()); //read EQ setting
  Serial.println(myDFPlayer.readFileCounts()); //read all file counts in SD card
  Serial.println(myDFPlayer.readCurrentFileNumber()); //read current play file number
  Serial.println(myDFPlayer.readFileCountsInFolder(3)); //read fill counts in folder SD:/03

  isPlaying = true;
  myDFPlayer.enableLoopAll();
  while (thisChar >= 1){
  keepSkippingUntilZero();
  }
}

void loop()
{
  buttonPauseState =  digitalRead(buttonPause);
  // compare the buttonState to its previous state
  if (buttonPauseState != lastPauseButtonState)
  {
    if (buttonPauseState == LOW)
    {
      if(isPlaying)
      {
       Serial.print( " Paused " );
       myDFPlayer.pause();
       isPlaying = false;
       delay(200);
       }
       else
       {
       Serial.print( " Play " );
       isPlaying = true;
       myDFPlayer.start();
       delay(200);
      }
    }
  }
 lastPauseButtonState = buttonPauseState;
 
 if (digitalRead(buttonVolPlus) == ACTIVATED)
  {
    if(isPlaying)
    {
      Serial.print( " Vol-Up " );
      myDFPlayer.volumeUp();
      delay(50);
    }
  }

  if (digitalRead(buttonVolMinus) == ACTIVATED)
  {
      Serial.print( " Vol-Down " );
      myDFPlayer.volumeDown();
      delay(50);
  }

 if (digitalRead(buttonNext) == ACTIVATED)
  {
    if(isPlaying)
    {
      Serial.print( " Button Next " );
      trackCount++;
      myDFPlayer.next();
      thisChar = ((folderCount + trackCount + 1));
      delay(200);
    }
  }

  if (digitalRead(buttonPrev) == ACTIVATED)
  {
    if(isPlaying)
    {
      Serial.print( " Button Prev' " );
      myDFPlayer.previous();
      trackCount--;
      thisChar = ((folderCount + trackCount - 1));
      delay(200);
    }
  }

 if (digitalRead(buttonFolderNext) == ACTIVATED)
  {
    if(isPlaying)
    {
      Serial.print( " Button folder+ " );
      folderCount++;
      Serial.print(folderCount);
      folderChange();
      thisChar = (((folderCount * 5) + trackCount + 5));
      delay(200);
    }
  }

  if (digitalRead(buttonFolderPrev) == ACTIVATED)
  {
    if(isPlaying)
    {
      Serial.print( " Button folder- " );
      folderCount--;
      Serial.print(folderCount);
      folderChange();
      thisChar = (((folderCount * 5) + trackCount - 5));
      delay(200);
    }
  }

  if (myDFPlayer.available())
  {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }
  while (myDFPlayer.available())
  {
    char inChar = ((char)(myDFPlayer.read()));                    //This method is a work-around for me being too dumb.
    Serial.print(myDFPlayer.readType(), myDFPlayer.read());       //I'm too dumb to figure out how to read the "play finished" output as a string?
    if (inChar != 0)                                              //So i came up with this, Disgusting! it's just using any data while playing to 'auto-next'.
        if(isPlaying)                                             //Even interference from a spark plug could trigger this.  Shield your wires? I used foil tape.
        {
         Serial.print( " Auto-Button Next " );                    //Foil tape wasn't nessisary for me in the end but is good practice anyhow.
         trackCount++;
         myDFPlayer.next();
         delay(20);
         thisChar = inChar;
         inChar = 0;
        }
  }
  A0State = digitalRead(A0);
  if (A0State != lastA0State)
    if (A0State == LOW)
    {
      Serial.print( " AO low, writing file to play " );
      EEPROM_writeAnything(0, (thisChar));
      Serial.print(thisChar);
    }
  lastA0State = A0State;
  delay(10);  //This needs to be a 120 delay or the button presses skip just a bit faster than i'm comfortable with.
}

void keepSkippingUntilZero()
{
  if (thisChar >= 1)
  {
    folderCount++;
    folderChange();            // Who does this?  And i tried folder jump instead of file jump because it's faster... foldernext(); was too slow.
    thisChar --;              // Skipping over & over until it reaches zero? Why do i have to rangi this?
    thisChar --;             // Why is life so hard for me?  Am i actually that dumb? (had 5 "thisChar --" B.T.W.)
    thisChar --;            // If you had the 'folder skip' at ten, you'd use ten of these? Or if it was a folder/file value, just use that. Duh. 
    thisChar --;
    thisChar --;
  }
  else
  {
    myDFPlayer.start();  //time to stop skipping you weirdo.
  }
}

void folderChange()              //Because simply taking the current track & adding 5 or 10 is having strange results lately. 
// trying to edit this to use folder count times 5, or 10, whatever i think the folder size should be, + track count as myDFPlayer.play((# + trackCount));
{
  trackAdvance = ((folderCount) * 5);
  myDFPlayer.play((trackAdvance + trackCount));
  delay(50);
}

void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void execute_CMD(byte CMD, byte Par1, byte Par2)
// Excecute the command and parameters
{
  // Calculate the checksum (2 bytes)
  word checksum = -(Version_Byte + Command_Length + CMD + Acknowledge + Par1 + Par2);
  // Build the command line
  byte Command_line[10] = { Start_Byte, Version_Byte, Command_Length, CMD, Acknowledge,
                            Par1, Par2, highByte(checksum), lowByte(checksum), End_Byte
                          };
  //Send the command line to the module
  for (byte k = 0; k < 10; k++)
  {
    mySoftwareSerial.write( Command_line[k]);
  }
}
