#include <string.h>
#include <stdlib.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

AccelStepper stepper1(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
AccelStepper stepper2(AccelStepper::FULL4WIRE, 8, 9, 10, 11);
MultiStepper steppers;

const float DIST_PER_STEP = 1; // CALIBRATE THIS! - physical distance per step

int incomingByte = 0;
const int CHAR_PER_INSTR = 50; // expect max 35 bytes + terminator
const char END_MARKER = '!';

char rxMessage[CHAR_PER_INSTR];
int currentInstruction[8];

bool newInstruction = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  
  stepper1.setMaxSpeed(100);
  stepper2.setMaxSpeed(100);
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  readyForInstruction();
}

void loop() {
  readInstruction();
  parseInput();
  if(newInstruction) {
    executeInstruction();
    newInstruction = false;
    readyForInstruction();
  }
  delay(5);
}

void readInstruction() {
  char c;
  char toProcess[CHAR_PER_INSTR];
  int i = 0;
  bool done = false;
  if(Serial.available() > 0) {
    while(Serial.available() > 0 && !done) {
      c = Serial.read();
      if (c != END_MARKER) {
        rxMessage[i] = c;
        i++;
        if (i >=  CHAR_PER_INSTR) {
          i--;
          Serial.println("Error: Overflow");
          Serial.flush();
        } // overflow condition
      } else {
        rxMessage[i] = '\0';
        i = 0;
        done = true;
        newInstruction = true;
        return;
      }
    }
  }
  return 0;
}

void parseInput() {
//  for(int i = 0; i < 4; i++)
//    currentInstruction[i] = i;
//  return;
  const char delim[2] = ",";
  char *token;

  token = strtok(rxMessage, delim);
  int i = 0;
  
  while(token != NULL) {
    if (i > 3) {
      Serial.println("Bad input, too many delimiters");
      Serial.flush();
      return 0;
    }
    currentInstruction[i] = atoi(token);
    i++;
    token = strtok(NULL, delim);
  }
  return 0;
}

void executeInstruction() {
//  for(int i = 0; i < 4; i++)
//    Serial.print(currentInstruction[i]);
//  Serial.print("\n");

  long positions[] = {currentInstruction[0], 
                      currentInstruction[1]}; // Array of desired stepper positions
  //steppers.moveTo(positions);
  Serial.print("Position 0 is ");
  Serial.println(positions[0]);
  Serial.print("Position 1 is: ");
  Serial.println(positions[1]);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(10); // can probably be removed
}

void readyForInstruction() {
  Serial.println(">");
  delay(10);
}
