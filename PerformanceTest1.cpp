#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHIO.h>
#include <FEHRPS.h>

#define COUNTS_PER_INCH 33.74084794
#define START_THRESHOLD 2.8
#define JUKEBOX_THRESHOLD 2.6
#define COUNTS_PER_DEGREE 1.95

FEHMotor rightMotor(FEHMotor::Motor1, 9.0);
FEHMotor leftMotor(FEHMotor::Motor0, 9.0);
DigitalEncoder rightEncoder(FEHIO::P0_0);
DigitalEncoder leftEncoder(FEHIO::P3_4);
AnalogInputPin lightSensor(FEHIO::P1_0);
DigitalInputPin rightBump(FEHIO::P0_1);
DigitalInputPin leftBump(FEHIO::P3_5);
AnalogInputPin lineSensor(FEHIO::P2_0);

void moveForward(int percent, int inches)
{

    int counts = COUNTS_PER_INCH * inches;

    //Reset encoder counts
    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();

    //Set both motors to desired percent
    rightMotor.SetPercent(percent);
    leftMotor.SetPercent(percent+1);

    //While the average of the left and right encoder is less than counts,
    //keep running motors
    while((leftEncoder.Counts() + rightEncoder.Counts()) / 2. < counts);

    //Turn off motors
    rightMotor.Stop();
    leftMotor.Stop();
}


void moveBackward(int percent, int inches)
{

    int counts = COUNTS_PER_INCH * inches;

    //Reset encoder counts
    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();

    //Set both motors to desired percent
    rightMotor.SetPercent(-1 * percent - 1);
    leftMotor.SetPercent(-1 * percent);

    //While the average of the left and right encoder is less than counts,
    //keep running motors
    while((leftEncoder.Counts() + rightEncoder.Counts()) / 2. < counts);

    //Turn off motors
    rightMotor.Stop();
    leftMotor.Stop();
}


void turnLeft(int percent, int degrees)
{

    int counts = COUNTS_PER_DEGREE * degrees;

    //Reset encoder counts
    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();

    //Set both motors to desired percent
    rightMotor.SetPercent(percent);
    leftMotor.SetPercent(-1 * percent);

    //While the average of the left and right encoder is less than counts,
    //keep running motors
    while((leftEncoder.Counts() + rightEncoder.Counts()) / 2. < counts);

    //Turn off motors
    rightMotor.Stop();
    leftMotor.Stop();
}

void turnRight(int percent, int degrees)
{

    int counts = COUNTS_PER_DEGREE * 1.125 * degrees;

    //Reset encoder counts
    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();

    //Set both motors to desired percent
    leftMotor.SetPercent(percent);
    rightMotor.SetPercent(-1 * percent);

    //While the average of the left and right encoder is less than counts,
    //keep running motors
    while((leftEncoder.Counts() + rightEncoder.Counts()) / 2. < counts);

    //Turn off motors
    rightMotor.Stop();
    leftMotor.Stop();
}

void jukeboxTask(){
    if (lightSensor.Value() > JUKEBOX_THRESHOLD){
        //blue
        LCD.Clear(FEHLCD::Blue);
        LCD.WriteLine(lightSensor.Value());
        turnLeft(25, 35);
        moveForward(15,2);
        turnRight(25,35);
        rightMotor.SetPercent(20);
        leftMotor.SetPercent(20);
        while(rightBump.Value() && leftBump.Value());
        Sleep(.5);
        rightMotor.Stop();
        leftMotor.Stop();

    } else {
        //red
        LCD.Clear(FEHLCD::Red);
        LCD.WriteLine(lightSensor.Value());
        turnRight(25, 35);
        moveForward(15,2);
        turnLeft(25,35);
        rightMotor.SetPercent(20);
        leftMotor.SetPercent(20);
        while(rightBump.Value() && leftBump.Value());
        Sleep(.5);
        rightMotor.Stop();
        leftMotor.Stop();
    }
}




int main(void)
{


    float x,y;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    LCD.WriteLine("Waiting for touch");
    while(!LCD.Touch(&x,&y));
    LCD.Clear();


    LCD.WriteLine("Waiting for light");

    while(lightSensor.Value() > START_THRESHOLD);
    LCD.Clear();
    LCD.WriteLine("Here i go!");


    moveForward(25,15);

    turnRight(25, 45);

     moveForward(35,30);
     moveBackward(25,27);

     turnLeft(25, 90);


    leftMotor.SetPercent(35);
    rightMotor.SetPercent(35);

    while (rightBump.Value() || leftBump.Value());

    leftMotor.Stop();
    rightMotor.Stop();

    moveBackward(25,3);

    turnLeft(25,95);
    moveBackward(25, 6);
    rightMotor.SetPercent(10);
    leftMotor.SetPercent(10);
    LCD.Clear(FEHLCD::Black);
    LCD.WriteLine("Trying to detect light");
    while(lightSensor.Value() > 3);
    LCD.Clear(FEHLCD::Black);
    LCD.WriteLine("Detected a light");
    Sleep(2.0);

    leftMotor.Stop();
    rightMotor.Stop();

    jukeboxTask();
    LCD.Clear(FEHLCD::Black);




    return 0;
}
