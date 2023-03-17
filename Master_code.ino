/*Include libraries --------------*/
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h> // Library from https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library



/*Defines -----------*/
#define RIGHT_SERVO_PIN 5
#define LEFT_SERVO_PIN 6
#define MIN_PULSE 1300
#define MAX_PULSE 1700
#define STANDSTILL 1500
#define LEFT_QTI  A0
#define MIDDLE_QTI A1
#define RIGHT_QTI A2
#define QTI_THRESHOLD 443
#define BUTTON_PIN 2
#define INTERSECTIONS 6
#define SECTIONS 4 + 1
#define SECTION_DEAD_END INTERSECTIONS + 2
#define SECTION_UNKNOWN INTERSECTIONS + 4
#define START INTERSECTIONS + 1
#define FINISH INTERSECTIONS + 3

/*Global variables -----*/
Servo leftWheel; //servo variables
Servo rightWheel;

LiquidCrystal_I2C lcd(0x27, 16, 2); //LCD variables

int toggle = 0; // flag for toggle 
unsigned long g_last_debounce_time = 0;  // the last time the output pin was toggled
unsigned long g_debounce_delay = 50;    // the debounce time; increase if the output flickers
int g_button_state;             // the current reading from the input pin
int g_last_button_state = LOW;   // the previous reading from the input pin

int intersections[INTERSECTIONS][SECTIONS] = {1}; // memory matrix
int intersection = 0; // current intersection
int section = 1; // current section

int start = 1; // start variable

int finish; // variable where to save intersection that includes finish line

//RoadMapping variables
int wentBack = 0;
int wentBackCounter = 0;
int nextIntersection = 0;

//LCD display variables
byte customChar[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};
byte customChar2[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};
byte customChar3[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};
byte customChar4[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};

byte customCharCursor = 0b00000; // Variable to write values into customChar arrays

int row = 7; // Variable to know, which row to write into

//Variables to know, where to write new value into
int cursorRow = 0;
int cursorCol = 0;

int displayRTB = 0;




/*Private Functions-----------*/
byte ReadQti (byte qti)                                 // Function to get QTI sensor current reading
{                               
  digitalWrite(qti, HIGH);                              // Send an infrared signal
  delayMicroseconds(1000);                               // Wait for 1ms, very important!
  digitalWrite(qti, LOW);                               // Set the pin low again
  return ( analogRead(qti) > QTI_THRESHOLD ? 1 : 0);    // Return the converted result: if analog value more than 443 return 1, else 0
}

void Debug()
{
  int i;
  int j;

  for(i = 0; i < INTERSECTIONS; i++)
  {

    Serial.println("-----------------");


    Serial.print("Intersection: ");
  
    Serial.println(i);
    //Serial.print("Section: ");
    //Serial.println(section);
    //Serial.print("WentBack flag: ");
    //Serial.println(wentBack);

    Serial.println("");
    for(j = 0; j < SECTIONS; j++)
    {
      Serial.print(intersections[i][j]);
      Serial.print(" ");
    }
    Serial.println("");
    Serial.println("-------------------");
  }

}

/**
 * Description: Adds flag value to the intersections matrix based on current
 *              intersection and section.
 *              If there already is a flag in the current position, that is not
 *              the SECTION_UNKNOWN flag, then outputs error and required info
 *              for debugging.
 *
 * Parameters: flag - integer, value to add to current position in matrix.
 *
 */
void AddToMemory(int flag)
{

  if(intersections[intersection][section] == SECTION_UNKNOWN)
  {
    intersections[intersection][section] = flag;
  }
  else
  {
    Serial.print("wentBack: ");
    Serial.println(wentBack);
    Serial.println("WentBackCounter: ");
    Serial.println(wentBackCounter);
    Serial.print("flag: ");
    Serial.println(flag);
    Serial.print("Intersection: ");
    Serial.println(intersection);
    Serial.print("Section: ");
    Serial.println(section);
    Serial.print("Value: ");
    Serial.println(intersections[intersection][section]);
    Serial.println("ERROR! Attempting to rewrite wrong memory");

  } 
  
}

/**
 *Function to return info about current section in current intersection
 */
int ReadSectionFromMemory()
{
  return intersections[intersection][section];
}



/**
 *Function to increment section variable,
 * also increments number of sections passed in current intersection.
 */
void NextSection()
{  
  AddSection();
  if(section == 4)
  {
    section = 1;
  }
  else
  {
    section++;
  }
}


//Function to increment number of sections passed in current intersection, wont allow it to increment higher than number of actual sections in an intersection
/**
 * Function to increment number of sections passed in current intersection,
 * wont allow it to increment higher than number of actual sections in an intersection.
 */
void AddSection()
{
  if(intersections[intersection][0] + 1 <= SECTIONS - 1)
  {
    intersections[intersection][0]++;
  }
  else
  {

    intersections[intersection][0] = SECTIONS - 1;
  }
}


/**
 * Description: Function to navigate the map based on readings from QTI sensors.
 *              Follows left hand rule, as in always turns left to go through every section.
 *              QTI sensor value of 1 means a black surface has been detected by the QTI sensor,
 *               0 means a white surface.
 *
 *
 * Parameters: leftQti - byte, value of left QTI sensor
 *             rightQti - byte, value of right QTI sensor 
 *             middleQti - byte, value of middle QTI sensor
 */
void ReadSurface(byte leftQti, byte rightQti, byte middleQti)
{ 

  if (!leftQti && middleQti && !rightQti)  // robot is on the middle of the line
  {
    Forward(); 
  }
  else if (leftQti && middleQti && rightQti) // robot has reached an intersection
  {
    lcd.print("Exit");
    Left();
    delay(1200);

    if(!ReadQti(LEFT_QTI) && !ReadQti(MIDDLE_QTI) && !ReadQti(RIGHT_QTI)) // robot has detected the finish line
    {

      Standstill();
      delay(1000);

      finish = intersection; // saves the intersection number, where the finish line is at.

      lcd.clear();
      lcd.print("FINISH");

      if(CheckMap() == 0) // if the whole map hasnt been gone through yet
      {
        if (!(!ReadQti(LEFT_QTI) && ReadQti(MIDDLE_QTI) && !ReadQti(RIGHT_QTI))) // robot turns around and gets back on track 
        {

          TurnAround();
          delay(2800);
          Right();
          delay(200);
          Forward();
          delay(600);
          
        }      
      }
      AddToMemory(FINISH); // Adds FINISH flag to memory.
      nextIntersection = 0;

    }
    else
    {
      RoadMapping(); // Calls out the RoadMapping function, to start saving info about the map.
      nextIntersection = 1;      
    }
  
  }
  else if (!leftQti && !middleQti && !rightQti) // robot has reached the end of a line.
  {

    Forward(); // robot goes forward a bit, to ensure the turn around happens correctly.
    delay(500);

    nextIntersection = 0;
    AddToMemory(SECTION_DEAD_END); // Adds SECTION_DEAD_END flag to memory

    lcd.print("Tupik");
      
    while (!(!ReadQti(LEFT_QTI) && ReadQti(MIDDLE_QTI) && !ReadQti(RIGHT_QTI)))
    {
      TurnAround(); // Robot turns around, until back on track
    }
    
  }
  else if (!leftQti && middleQti && rightQti) // robot has gone off too much to the left of the line
  {
    Right();
  }
  else if (leftQti && middleQti && !rightQti) // robot has gone off too much to the right of the line
  {
    Left();
  }

}


//Function to choose next section based on the section that was last used, also manipulates customCharCursor for displaying the map on LCD display

/**
 * Description: Function to choose next section based on the section that was last used.
 *              Also manipulates customCharCursor for displaying the map on the LCD display.
 *              Intersection model is as follows:
 *
 *                               3
 *                             2   4
 *                               1
 */
void ChooseSection()
{
  if(section == 1)
  {

    section = 3;

    if(row != 7) // Moves the customCharCursor down one place
    {
      row++;
    }
  }
  else if(section == 2)
  {
    section = 4;

    if(customCharCursor != 0b10000)
    {
      customCharCursor = customCharCursor << 1; // Moves the customCharCursor to the left by one place
    }
    else
    {
      if(cursorCol == 1)
      {

        cursorCol = 0; // Swaps the value.

      }   
    }

  }
  else if(section == 3)
  {
    section = 1;

    if(row != 0)
    {
      row--; // Moves the customCharCursor up one place
    }


  }
  else if(section == 4)
  {
    section = 2;

    if(customCharCursor != 0b00001)
    {
      customCharCursor = customCharCursor >> 1; // Moves the customCharCursor to the right by one place

    }
    else
    {
      if(cursorCol != 1)// Switch the values.
      {
        cursorCol++;
      }
      else
      {
        cursorCol = 0;
      }
    }

    if(cursorRow == 0 && cursorCol == 0) // Choose the customChar array, where to save value.
    {

      customChar[row] += customCharCursor;   

    }
    else if(cursorRow == 0 && cursorCol == 1)
    {

      customChar2[row] += customCharCursor;

    }
    else if(cursorRow == 1 && cursorCol == 0)
    {

      customChar3[row] += customCharCursor;

    }
    else
    {

      customChar4[row] += customCharCursor;

    }
  }
}


/**
 * Description: Function to have robot go forward.
 */
void Forward()
{

  leftWheel.writeMicroseconds(1525);
  rightWheel.writeMicroseconds(1475);

}

/**
 * Description: Function to have robot stay still.
 */
void Standstill()
{

  leftWheel.writeMicroseconds(STANDSTILL);
  rightWheel.writeMicroseconds(STANDSTILL);

}

/**
 * Description: Function to have robot turn right.
 */
void Right()
{

  leftWheel.writeMicroseconds(1540);
  rightWheel.writeMicroseconds(1520);

}

/**
 * Description: Function to have robot turn left.
 */
void Left()
{

  leftWheel.writeMicroseconds(1480);
  rightWheel.writeMicroseconds(1460);

}

/**
 * Description: Function to have robot turn around in place.
 */
void TurnAround()
{

  leftWheel.writeMicroseconds(1470);
  rightWheel.writeMicroseconds(1470);

}


/**
 * Description: Function to save info on the map, follos the rule, that every turn is a left-hand turn.
 *              Uses the nextIntersection, wentBack and wentBackCounter variables to decide, how to act.
 *
 */
void RoadMapping()
{

  if(nextIntersection == 0)// If not going to next intersection
  {
    if(start == 1)
    {
      intersections[intersection][section] = START;

      start = 0;

      customChar[row] = 0b01000;
      customCharCursor = 0b01000;
    }

    NextSection();


    CustomCharWrite();
  }

  if(nextIntersection == 1)// If going to next intersection
  {
    if(intersection == 0 && wentBack == 0)// If at the first intersection and havent gone back
    {
    AddSection();
    AddToMemory(intersection + 1);
    intersection++;
    ChooseSection();
    AddToMemory(intersection - 1);
    NextSection();


    CustomCharWrite();

    lcd.clear();



    }
    //if going back to previous intersection
    else if(ReadSectionFromMemory() != START && ReadSectionFromMemory() != SECTION_DEAD_END && ReadSectionFromMemory() != FINISH && ReadSectionFromMemory() != SECTION_UNKNOWN && wentBack == 0)
    {

      intersection = ReadSectionFromMemory();//Gets the value of what the intersection is, where robot is going back to
      ChooseSection();
      NextSection();

      CustomCharWrite();//Writes value to a customChar array to display it.

      wentBack = 1;// Saves info, that robot is going back to a previously saved intersection.
      wentBackCounter = 1;

    }
   else if(ReadSectionFromMemory() != START && ReadSectionFromMemory() != SECTION_DEAD_END && ReadSectionFromMemory() != FINISH && ReadSectionFromMemory() != SECTION_UNKNOWN && wentBack == 1)
    {
      
      intersection = ReadSectionFromMemory();
      ChooseSection();
      NextSection();

      wentBackCounter++;

    }
    //if going to next intersection and havent gone back an intersection before 
    else if(ReadSectionFromMemory() == SECTION_UNKNOWN && wentBack == 0)
    {
  
      AddSection();  
      AddToMemory(intersection + 1);
      intersection++;
      ChooseSection();
      AddToMemory(intersection - 1);
      NextSection();

      CustomCharWrite();

    }
    //if going to next intersection but have gone back to an intersection before
    else if(ReadSectionFromMemory() == SECTION_UNKNOWN && wentBack == 1)
    {
      wentBack = 0;

      AddToMemory(intersection + wentBackCounter + 1);//Chooses intersection value based on how many intersections robot has gone back to
      intersection += wentBackCounter + 1;
      ChooseSection();
      AddToMemory(intersection - wentBackCounter - 1);
      NextSection();

      CustomCharWrite();

    }

  }  

}

void FunnyFortnite() //Notes from https://github.com/AnonymousAlly/Arduino-Music-Codes/blob/master/Default.ino
{
  tone(A3,349,149);//F4 
  delay(149); 
  tone(A3,415,149);//Ab/G#4 
  delay(149);
  tone(A3,466,149);//Bb/A#4 
  delay(149); 
  tone(A3,466,446);//Bb/A#4 
  delay(446);  
  tone(A3,415,297);//Ab/G#4 
  delay(1485);
  tone(A3,349,149);//F4 
  delay(149); 
  tone(A3,415,149);//Ab/G#4 
  delay(149);
  tone(A3,466,149);//Bb/A#4 
  delay(149); 
  tone(A3,466,446);//Bb/A#4 
  delay(446);  
  tone(A3,415,297);//Ab/G#4 
  delay(297);
  tone(A3,349,297);//F4 
  delay(297); 
  tone(A3,311,149);//Eb/D#4 
  delay(149); 
  tone(A3,349,149);//F4 
  delay(595); 
  tone(A3,466,149);//Bb/A#4 
  delay(149);
  tone(A3,415,149);//Ab/G#4 
  delay(149);
  tone(A3,349,149);//F4 
  delay(149); 
  tone(A3,311,149);//Eb/D#4 
  delay(149); 
  tone(A3,349,149);//F4 
  delay(1931);
}



/**
 * Description: Function to find finish line in current intersection.
 *
 * Return:      Returns an integer based on what position the FINISH flag is at.
 */
int FindFinish()
{
  int i;

  for(i = 1; i < SECTIONS; i++)
  {
    if(intersections[intersection][i] == FINISH)
    {
      return i;
    }
  }
}



/**
 * Description: Function to navigate robot back to finish.
 *              Uses info saved in intersections array, to decide next action.
 */
void NavigateToFinish()
{
  int counter = section;
  int i;

  if(displayRTB == 0)// Let PrintToDisplay function know, to display RTB.
  {
    displayRTB = 1;
    lcd.print("RTB!");
  }

  if(intersection != finish)// if not in intersection that leads to the finish line.
  {
    for(i = 1; i < SECTIONS; i++)
    {

      if(finish > intersection)// if the intersection that leads to the finish line has a higher value than current intersection.
      {
        if(intersections[intersection][i] > intersection || intersections[intersection][i] == finish)
        {

          counter = i;

        }
      }
      else
      {
        if(intersections[intersection][i] < intersection || intersections[intersection][i] == finish)
        {
          counter = i;

        }
      }
    }

    intersection = intersections[intersection][counter];//Chooses next intersection.

    while(!(ReadQti(LEFT_QTI) && ReadQti(MIDDLE_QTI) && ReadQti(RIGHT_QTI)))//While robot has not reached an intersection, follows the line.
    {
      int leftQti = ReadQti(LEFT_QTI);
      int middleQti = ReadQti(MIDDLE_QTI);
      int rightQti = ReadQti(RIGHT_QTI);

      if (!leftQti && middleQti && !rightQti)//Robot is on the middle of the line
      {
        Forward(); 
      }
      else if (!leftQti && middleQti && rightQti)//Robot has gone off too much to the left
      {
        Right();
      }
      else if(leftQti && middleQti && !rightQti)//Robot has gone off too much to the right
      {
        Left();
      } 
    }

    //Choose which section to turn out of.
    if(counter == section + 1)
    {

      Left();
      delay(1200);
      section++;

    }
    else if(counter == section - 1)
    {
    
      Right();
      delay(1200);
      section--;

    }
    else if(counter == 1 && section == SECTIONS - 1)
    {
      
      Left();
      delay(1200);
      section = 1;

    }
    else if(counter == SECTIONS - 1 && section == 1)
    {

      Right();
      delay(1200);
      section = SECTIONS - 1;

    }
    else
    {

      Forward();
      delay(600);
      ChooseSection();
    }

    //Go to next intersection
    while(!(ReadQti(LEFT_QTI) && ReadQti(MIDDLE_QTI) && ReadQti(RIGHT_QTI)))
    {
      int leftQti = ReadQti(LEFT_QTI);
      int middleQti = ReadQti(MIDDLE_QTI);
      int rightQti = ReadQti(RIGHT_QTI);

      if (!leftQti && middleQti && !rightQti)
      {
        Forward(); 
      }
      else if (!leftQti && middleQti && rightQti)
      {
        Right();
      }
      else if(leftQti && middleQti && !rightQti)
      {
        Left();
      } 
    }

    ChooseSection();

    NavigateToFinish();//Recursively call out function again.

  }
  else// If in intersection that leads to the finish line.
  {
    //Choose, which section to turn into.
    if(section == 1)
    {
      if(FindFinish() == 2)
      {
        Left();
        delay(1200);
      }
      else if(FindFinish() == 3)
      {
        Forward();
        delay(600);
      }
      else if(FindFinish() == 4)
      {
        Right();
        delay(1200);
      }
    }
    else if(section == 2)
    {
      if(FindFinish() == 1)
      {
        Right();
        delay(1200);
      }
      else if(FindFinish() == 3)
      {
        Left();
        delay(1200);
      }
      else if(FindFinish() == 4)
      {
        Forward();
        delay(600);
      }
    }
    else if(section == 3)
    {
      if(FindFinish() == 2)
      {
        Right();
        delay(1200);
      }
      else if(FindFinish() == 1)
      {
        Forward();
        delay(600);
      }
      else if(FindFinish() == 4)
      {
        Left();
        delay(1200);
      }      
    }
    else if(section == 4)
    {
      if(FindFinish() == 1)
      {
        Left();
        delay(1200);
      }
      else if(FindFinish() == 2)
      {
        Forward();
        delay(600);
      }
      else if(FindFinish() == 3)
      {
        Right();
        delay(1200);
      }      
    }


    while(!(ReadQti(LEFT_QTI) && ReadQti(MIDDLE_QTI) && ReadQti(RIGHT_QTI)))
    {
      int leftQti = ReadQti(LEFT_QTI);
      int middleQti = ReadQti(MIDDLE_QTI);
      int rightQti = ReadQti(RIGHT_QTI);

      if (!leftQti && middleQti && !rightQti)
      {
        Forward(); 
      }
      else if (!leftQti && middleQti && rightQti)
      {
        Right();
      }
      else if(leftQti && middleQti && !rightQti) 
      {
        Left();
      } 
    }

    Standstill();    

    while(1)
    {


      lcd.clear();
      lcd.createChar(0, customChar);
      lcd.createChar(1, customChar2);
      lcd.createChar(2, customChar3);
      lcd.createChar(3, customChar4);

      lcd.setCursor(0,0);
      lcd.write(0);
      lcd.setCursor(1,0);
      lcd.write(1);
      lcd.setCursor(0,1);
      lcd.write(2);
      lcd.setCursor(1,1);
      lcd.write(3);

      lcd.print("All Done!");

      //Let user know with music, that robot has reached the finish line.
      FunnyFortnite();

    }    
        
    
  }

}

/**
 * Description: Function to move the customCharCursor one place left.
 *              Also deals with overfilling.
 */
void CustomCharLeft()
{
  if(customCharCursor == 0b10000)
  {

    cursorCol = 0;
    customCharCursor = 0b00001;

  }
  else
  {
    customCharCursor = customCharCursor << 1;
  }
}


/**
 * Description: Function to move the customCharCursor one place right.
 *              Also deals with overfilling.
 */
void CustomCharRight()
{
  if(customCharCursor == 0b00001)
  {

    cursorCol = 1;
    customCharCursor = 0b10000;

  }
  else
  {
    customCharCursor = customCharCursor >> 1;
  }
}


/**
 * Description: Function to move the customCharCursor one place up.
 *              Also deals with overfilling.
 */
void CustomCharUp()
{
  if(row == 0)
  {

    cursorRow = 0;
    row = 7;

  }
  else
  {
    row--;
  }
}



/**
 * Description: Function to move the customCharCursor one place down.
 *              Also deals with overfilling.
 */
void CustomCharDown()
{
  if(row == 7)
  {

    cursorRow = 1;


    if(cursorRow == 0 && cursorCol == 0)
    {
      customChar[0] += customCharCursor;
      customChar[1] += customCharCursor;      
    }
    else if(cursorRow == 0 && cursorCol == 1)
    {

      customChar2[0] += customCharCursor;
      customChar2[1] += customCharCursor;
    }
    else if(cursorRow == 1 && cursorCol == 0)
    {
      customChar3[0] += customCharCursor;
      customChar3[1] += customCharCursor;
    }
    else
    {
      customChar4[0] += customCharCursor;
      customChar4[1] += customCharCursor;
    }
    row = 2;
  }
  else
  {

    if(row != 7)
    {
      row++;
    }
  }
}


/**
 * Description: Function to write values into customChar arrays.
 *              Because LCD display custom characters use binary values,
 *              CustomChar manipulating functions use bitwise operators.
 *              
 */
void CustomCharWrite()
{
  switch(section)
  {
    case 1:

      CustomCharDown();
      CustomCharLeft();
      
      //Choose which array to save value to.
      if(cursorRow == 0 && cursorCol == 0)
      {

        if(customChar[row] != 0b11000 || !(customChar[row] > 0b11000))
        {
          customChar[row] += customCharCursor; 
        }
                
      }
      else if(cursorRow == 0 && cursorCol == 1)
      {

        if(customChar2[row] != 0b11000 || !(customChar2[row] > 0b11000))
        {
          customChar2[row] += customCharCursor; 
        }
      }
      else if(cursorRow == 1 && cursorCol == 0)
      {

        if(customChar3[row] != 0b11000 || !(customChar3[row] > 0b11000))
        {
          customChar3[row] += customCharCursor; 
        }
      }
      else
      {

        if(customChar4[row] != 0b11000 || !(customChar4[row] > 0b11000))
        {
          customChar4[row] += customCharCursor; 
        }
      }

      break;
    case 2:

      CustomCharUp();
      CustomCharLeft();


      if(cursorRow == 0 && cursorCol == 0)
      {

        if(customChar[row] != 0b11010)
        {
          customChar[row] += customCharCursor; 
        }
                
      }
      else if(cursorRow == 0 && cursorCol == 1)
      {

        if(customChar2[row] != 0b11010)
        {
          customChar2[row] += customCharCursor; 
        }
      }
      else if(cursorRow == 1 && cursorCol == 0)
      {

        if(customChar3[row] != 0b11010)
        {
          customChar3[row] += customCharCursor; 
        }
      }
      else
      {

        if(customChar4[row] != 0b11010)
        {
          customChar4[row] += customCharCursor; 
        }
      } 
      break;
    case 3:

      CustomCharUp();
      CustomCharRight();

      if(cursorRow == 0 && cursorCol == 0)
      {

        if(customChar[row] != 0b01011)
        {
          customChar[row] += customCharCursor; 
        }
                
      }
      else if(cursorRow == 0 && cursorCol == 1)
      {

        if(customChar2[row] != 0b01011)
        {
          customChar2[row] += customCharCursor; 
        }
      }
      else if(cursorRow == 1 && cursorCol == 0)
      {

        if(customChar3[row] != 0b01011)
        {
          customChar3[row] += customCharCursor; 
        }
      }
      else
      {

        if(customChar4[row] != 0b01011)
        {
          customChar4[row] += customCharCursor; 
        }
      } 
      break;
    case 4:

      CustomCharDown();
      CustomCharRight();

      if(cursorRow == 0 && cursorCol == 0)
      {

        customChar[row] += customCharCursor;        
      }
      else if(cursorRow == 0 && cursorCol == 1)
      {

        customChar2[row] += customCharCursor;
      }
      else if(cursorRow == 1 && cursorCol == 0)
      {

        customChar3[row] += customCharCursor;
      }
      else
      {

        customChar4[row] += customCharCursor;
      }
      break;
  }
}


/**
 * Description: Function to read when button is bressed.
 *              Uses debouncer.
 *
 * Return:      Returns a byte, value 1 means button is pressed, 0 means not.         
 */
byte ButtonRead() // function to read when button is pressed. With debouncer 
{ 
  int reading = digitalRead(BUTTON_PIN);
  if (reading != g_last_button_state) {
    g_last_debounce_time = millis();
  }
  if ((millis() - g_last_debounce_time) > g_debounce_delay) {
    if (reading != g_button_state) {
      g_button_state = reading;
      if (g_button_state == HIGH) {
        return 1;
      }
    }
  }
  g_last_button_state = reading;
  return 0;
}


/**
 * Description: Function to swap toggle flag to turn orobt on.
 */
void OnOff(byte buttonState)
{
  if(buttonState)
  {
    if(toggle == 1)
    {
      toggle = 0;
      Standstill();
      lcd.clear();
      lcd.print("Vajuta nuppu");
    }
    else
    {
      toggle = 1;
    }
  }
  
}


/**
 * Description: Function to check, if the whole map has been mapped.
 *              
 * Return:      Returns an integer, 1 if whole map has been mapped, 0 if not.
 */
int CheckMap()
{
  int i;
  int j;

  
  for(i = 0; i < INTERSECTIONS; i++)
  {
    if(intersections[i][0] != 4)
    {
      return 0;
    }

    for(j = 1; j < SECTIONS; j++)
    {
      if(intersections[i][j] == SECTION_UNKNOWN)
      {

        
        return 0;
      }
    }

    
  }

  return 1;
}

/**
 * Description: Function to print map info to LCD display.
 *              Very limited.
 */
void PrintToDisplay()
{

  lcd.createChar(0, customChar);
  lcd.createChar(1, customChar2);
  lcd.createChar(2, customChar3);
  lcd.createChar(3, customChar4);

  lcd.clear();

  lcd.setCursor(0,0);
  lcd.write(0);
  lcd.setCursor(1,0);
  lcd.write(1);
  lcd.setCursor(0,1);
  lcd.write(2);
  lcd.setCursor(1,1);
  lcd.write(3);

}


void setup() 
{
  /*attach servo variables to defined pins*/
  leftWheel.attach(LEFT_SERVO_PIN);
  rightWheel.attach(RIGHT_SERVO_PIN);

  /*have robot to stay still to make sure, that servos are still calibrated*/
  Standstill();

  Serial.begin(9600);

  lcd.begin(); //Initialize LCD

  lcd.backlight(); //Turn backlight on 
  lcd.print("Vajuta nuppu");

  int i;

  for(i = 0; i < INTERSECTIONS; i++)
  {
    intersections[i][0] = 0;
  }

  int j;

  for(i = 0; i < INTERSECTIONS; i++)
  {
    for(j = 1; j < SECTIONS; j++)
    {
      intersections[i][j] = SECTION_UNKNOWN;
    }
  }


}

void loop() 
{
  OnOff(ButtonRead()); // Turn robot on or off

  while(CheckMap() == 1)
  {
    NavigateToFinish();
  } 

  //Start navigating, if robot turned on
  if (toggle == 1)
  {
    ReadSurface(ReadQti(LEFT_QTI), ReadQti(RIGHT_QTI), ReadQti(MIDDLE_QTI));
    PrintToDisplay();
  }
  else
  {
    Standstill();
  }

}
