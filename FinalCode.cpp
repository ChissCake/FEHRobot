#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHIO.h>
#include <FEHRPS.h>

#define COUNTS_PER_INCH 33.74084794
#define START_THRESHOLD 3.0
#define JUKEBOX_THRESHOLD 2.45
#define COUNTS_PER_DEGREE 1.95

//PID Variables and Constants
#define P_CONSTANT .75
#define I_CONSTANT .08
#define D_CONSTANT .25
#define SLEEP_TIME .15
#define DISTANCE_PER_COUNT .0296376665




FEHMotor rightMotor(FEHMotor::Motor2, 9.0);
FEHMotor leftMotor(FEHMotor::Motor1, 9.0);
FEHMotor vex(FEHMotor::Motor3, 7.2);
DigitalEncoder rightEncoder(FEHIO::P0_0);
DigitalEncoder leftEncoder(FEHIO::P3_4);
AnalogInputPin lightSensor(FEHIO::P1_0);
DigitalInputPin rightBump(FEHIO::P0_1);
DigitalInputPin leftBump(FEHIO::P3_5);




void startBoxMethod();
void driveToTrash();
void ticketMethod();
void driveUpRamp();
void burgerTask();
void burgerTaskToIceCream();
void driveToBurger();
void burgerTaskPutDown();
void driveToIceCream();
void iceCreamMethod();




class PID{

    public:

        void resetVariables();

        void driveForward(float speed, float inches);
        void driveBackward(float speed, float inches);
        void turnLeft(float speed, float degrees);
        void turnRight(float speed, float degrees);
        void driveUntilWall(float speed);
        void driveUntilTray(float speed);
        void driveUntilWall2(float speed);


        float leftMotorPID(float speed);
        float rightMotorPID(float speed);
        void wiggle(float time);
        void driveBackwardTicket(float speed, float inches);
        void setAngle(float angle, float precision);
        void setY(float y);
        void setY(float y, float precision);
        void setX(float x);




    private:

        float pastErrorLeft;
        float currentErrorLeft;
        float errorSumLeft;
        int pastCountsLeft;
        float pastTimeLeft;
        float oldPowerLeft;


        float pastErrorRight;
        float currentErrorRight;
        float errorSumRight;
        int pastCountsRight;
        float pastTimeRight;
        float oldPowerRight;
};

/********************************************************************************************************************/
/****************************************MAIN FUNCTION***************************************************************/
/********************************************************************************************************************/


int main(void)
{

    RPS.InitializeTouchMenu();


    float x,y;
    PID pid;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);





    //startBoxMethod();
    //driveUpRamp();
    //driveToIceCream();
    //iceCreamMethod();

    ticketMethod();

    return 0;
}


/********************************************************************************************************************/


/******************************************PID Functions***********************************************************/

void PID::resetVariables(){

    pastErrorLeft = 0;
    currentErrorLeft = 0;
    errorSumLeft = 0;
    pastCountsLeft = 0;
    pastTimeLeft = TimeNow();
    oldPowerLeft = 0;

    pastErrorRight = 0;
    currentErrorRight = 0;
    errorSumRight = 0;
    pastCountsRight = 0;
    pastTimeRight = TimeNow();
    oldPowerRight = 0;


    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();



    Sleep(SLEEP_TIME);
}


void PID::driveForward(float speed, float inches){


   resetVariables();

   int counts = inches * COUNTS_PER_INCH;
   rightMotor.SetPercent(25);
   leftMotor.SetPercent(25);

   while (rightEncoder.Counts() + leftEncoder.Counts() < counts * 2 ){

       leftMotor.SetPercent(leftMotorPID(speed));
       rightMotor.SetPercent(rightMotorPID(speed));
       Sleep(SLEEP_TIME);
   }

   rightMotor.Stop();
   leftMotor.Stop();

}

void PID::driveBackward(float speed, float inches){


   resetVariables();

   int counts = inches * COUNTS_PER_INCH;
   rightMotor.SetPercent(-25);
   leftMotor.SetPercent(-25);

   while (rightEncoder.Counts() + leftEncoder.Counts() < counts * 2 ){
       rightMotor.SetPercent(rightMotorPID(speed) * -1);
       leftMotor.SetPercent(leftMotorPID(speed) * -1);
       Sleep(SLEEP_TIME);
   }

   rightMotor.Stop();
   leftMotor.Stop();

}


void PID::driveBackwardTicket(float speed, float inches){


   resetVariables();

   int counts = inches * COUNTS_PER_INCH;
   rightMotor.SetPercent(-25);
   leftMotor.SetPercent(-25);

   while (rightEncoder.Counts() + leftEncoder.Counts() < counts * 2 ){
       rightMotor.SetPercent(rightMotorPID(speed) * -1);
       leftMotor.SetPercent(leftMotorPID(speed) * -1 - 10);
       Sleep(SLEEP_TIME);
   }

   rightMotor.Stop();
   leftMotor.Stop();

}

void PID::driveUntilWall2(float speed){

    resetVariables();

    float time = TimeNow();

    rightMotor.SetPercent(25);
    leftMotor.SetPercent(25);

    while (rightBump.Value() && leftBump.Value()){

        leftMotor.SetPercent(leftMotorPID(speed));
        rightMotor.SetPercent(rightMotorPID(speed));

        Sleep(SLEEP_TIME);
        if (TimeNow() - time > 8.0){
            driveBackward(10, 5);
            turnLeft(10, 15);
            driveUntilWall(speed);
        }
    }
}

void PID::driveUntilWall(float speed){

        resetVariables();



        rightMotor.SetPercent(25);
        leftMotor.SetPercent(25);

        while (rightBump.Value() || leftBump.Value()){

            leftMotor.SetPercent(leftMotorPID(speed));
            rightMotor.SetPercent(rightMotorPID(speed));

            Sleep(SLEEP_TIME);

        }



    rightMotor.Stop();
    leftMotor.Stop();




}

void PID::driveUntilTray(float speed){

    resetVariables();
    float currTime = TimeNow();

    rightMotor.SetPercent(25);
    leftMotor.SetPercent(25);

    while ((TimeNow() - currTime) < 2.0){
        if(!rightBump.Value() || !leftBump.Value())
        {
            break;
        }
        leftMotor.SetPercent(leftMotorPID(speed));
        rightMotor.SetPercent(rightMotorPID(speed));
        Sleep(SLEEP_TIME);
    }

    rightMotor.Stop();
    leftMotor.Stop();




}
void PID::turnLeft(float speed, float degrees){


   resetVariables();

   int counts = degrees * COUNTS_PER_DEGREE;
   rightMotor.SetPercent(25);
   leftMotor.SetPercent(-25);

   while (rightEncoder.Counts() + leftEncoder.Counts() < counts * 2 ){
       rightMotor.SetPercent(rightMotorPID(speed));
       leftMotor.SetPercent(leftMotorPID(speed) * -1);
       Sleep(SLEEP_TIME);
   }

   rightMotor.Stop();
   leftMotor.Stop();

}

void PID::turnRight(float speed, float degrees){


   resetVariables();

   int counts = degrees * COUNTS_PER_DEGREE;
   rightMotor.SetPercent(-25);
   leftMotor.SetPercent(25);

   while (rightEncoder.Counts() + leftEncoder.Counts() < counts * 2 ){
       rightMotor.SetPercent(rightMotorPID(speed) * -1);
       leftMotor.SetPercent(leftMotorPID(speed));
       Sleep(SLEEP_TIME);
   }

   rightMotor.Stop();
   leftMotor.Stop();

}



float PID::leftMotorPID(float speed){

    float pTerm, iTerm, dTerm;

    int deltaCounts = leftEncoder.Counts() - pastCountsLeft;
    float deltaTime = TimeNow() - pastTimeLeft;

    float actualVelocity = ((float) deltaCounts / deltaTime) * DISTANCE_PER_COUNT;
    currentErrorLeft= speed - actualVelocity;
    errorSumLeft += currentErrorLeft;
    pTerm = currentErrorLeft* P_CONSTANT;
    iTerm = errorSumLeft * I_CONSTANT;
    dTerm = (currentErrorLeft - pastErrorLeft) * D_CONSTANT;
    pastTimeLeft = TimeNow();
    pastCountsLeft = leftEncoder.Counts();
    pastErrorLeft = currentErrorLeft;

    float value = (oldPowerLeft + pTerm + iTerm + dTerm);
    oldPowerLeft = value;

   // LCD.Write(value);


    return value;
}

float PID::rightMotorPID(float speed){


    float pTerm, iTerm, dTerm;

    int deltaCounts = rightEncoder.Counts() - pastCountsRight;
    float deltaTime = TimeNow() - pastTimeRight;

    float actualVelocity = ((float) deltaCounts / deltaTime) * DISTANCE_PER_COUNT;
    currentErrorRight= speed - actualVelocity;
    errorSumRight += currentErrorRight;
    pTerm = currentErrorRight* P_CONSTANT;
    iTerm = errorSumRight * I_CONSTANT;
    dTerm = (currentErrorRight - pastErrorRight) * D_CONSTANT;
    pastTimeRight = TimeNow();
    pastCountsRight = rightEncoder.Counts();
    pastErrorRight = currentErrorRight;

    float value = (oldPowerRight + pTerm + iTerm + dTerm);
    oldPowerRight = value;



    //LCD.Write("    ");
   // LCD.WriteLine(value);



    return value;
}
void PID::wiggle(float time){

    float timeStart = TimeNow();
    while(TimeNow() - timeStart < time){
        driveForward(10, .35);
        turnRight(15, 5);

    }

}

void PID::setAngle(float angle, float precision){

    while((RPS.Heading() > -2) && (RPS.Heading() > angle + precision || RPS.Heading() < angle - precision)){
            LCD.WriteLine(RPS.Heading());

            if (RPS.Heading() > 300  && angle < 50){
                turnLeft(10,.7);
            }

            else if ((RPS.Heading() > angle && !(angle < 1 && angle > 0)) || ((angle < 1 && angle > 0) && RPS.Heading() < 180)){
                turnRight(10, .7);
            }
            else if ((RPS.Heading() < 100  && angle > 300)){
                turnRight(10,.7);
            }

            else if ((RPS.Heading() < angle && !(angle < 1 && angle > 0)) || ((angle < 1 && angle > 0) && RPS.Heading() >= 180)){
                turnLeft(10, .7);
            }

        }
}

void PID::setY(float y){

    //check if receiving proper RPS coordinates and whether the robot is within an acceptable range
        while(RPS.Y() > -1.9 && (RPS.Y() < y - .25 || RPS.Y() > y + .25 ))
        {
            if(RPS.Y() > y && RPS.Y() > 0)
            {
                //pulse the motors for a short duration in the correct direction

                 driveBackward(3, .1);

            }
            else if(RPS.Y() < y && RPS.Y() > 0)
            {
                //pulse the motors for a short duration in the correct direction

                 driveForward(3, .1);
            }

            LCD.WriteLine(RPS.Y());
        }

}

void PID::setY(float y, float precision){

    //check if receiving proper RPS coordinates and whether the robot is within an acceptable range
        while(RPS.Y() > -1.9 && (RPS.Y() < y - precision || RPS.Y() > y + precision ))
        {
            if(RPS.Y() > y && RPS.Y() > 0)
            {
                //pulse the motors for a short duration in the correct direction

                 driveBackward(3, precision / 2);

            }
            else if(RPS.Y() < y && RPS.Y() > 0)
            {
                //pulse the motors for a short duration in the correct direction

                 driveForward(3, precision / 2);
            }

            LCD.WriteLine(RPS.Y());
        }

}

void PID::setX(float x){

    //check if receiving proper RPS coordinates and whether the robot is within an acceptable range
        while(RPS.X() > -1.9 && (RPS.X() < x - .2 || RPS.X() > x + .2 ))
        {
            if(RPS.X() > x && RPS.X() > 0)
            {
                //pulse the motors for a short duration in the correct direction

                 driveBackward(3, .15);

            }
            else if(RPS.X() < x && RPS.X() > 0)
            {
                //pulse the motors for a short duration in the correct direction

                 driveForward(3, .15);
            }

            LCD.WriteLine(RPS.X());
        }





}

/********************************************************************************************************************/


/******************************************Task Functions************************************************************/


void startBoxMethod(){

    float x, y;

    LCD.WriteLine("Waiting for touch");
    while(!LCD.Touch(&x,&y));
    LCD.Clear();

    LCD.WriteLine("Waiting for light");
    while(lightSensor.Value() > START_THRESHOLD);
    LCD.Clear();
    LCD.WriteLine("VroomVroom");

}


void driveToTrash(){

    PID pid;

    pid.driveForward(10, 8.25);
    pid.turnLeft(10,44);
    pid.setAngle(178, 1);
    pid.driveUntilWall(8.5);
    pid.driveBackward(10,1.25);
    pid.turnRight(5,84.75);
    pid.setAngle(97, 1);
    pid.driveUntilTray(21);

}

void driveUpRamp(){
    PID pid;


    pid.driveForward(10, 15);
    pid.turnRight(5, 55);
    pid.driveForward(15, 20);
    pid.turnLeft(10, 25);
    pid.setAngle(90, 1);



}

void driveToIceCream(){
    PID pid;

    pid.driveForward(10, 11);
    LCD.WriteLine(RPS.Y());
    pid.setY(56, .5);
}

void iceCreamMethod(){

    PID pid;

    int flavor = RPS.GetIceCream();
    LCD.WriteLine(flavor);

    if (flavor == 0) {
        pid.turnLeft(5,55);
    } else if (flavor == 1) {
        pid.turnLeft(5, 42.5);
    } else {
        pid.turnLeft(5, 29.);
    }

    pid.driveForward(5, 9.25);
    pid.driveBackward(5, 7.5);

    pid.turnRight(8, 175);

    vex.SetPercent(25);
    Sleep(.65);
    vex.Stop();
    Sleep(2.2);

    pid.driveBackward(5, 12.5);


}

void driveToBurger(){

    PID pid;
    pid.turnRight(5, 95);
    pid.setAngle(0, .75);
    vex.SetPercent(-25);
    Sleep(1.0);
    vex.Stop();
    pid.driveUntilWall(10);
    pid.driveBackward(10, 3);
    pid.setX(28.6);

}


void ticketMethod(){
    PID pid;

    pid.driveForward(10, 7.5);
    pid.turnLeft(7, 45);
    pid.driveUntilWall(10);
    pid.driveBackward(7, 1);
    LCD.WriteLine(RPS.X());
    pid.turnLeft(8, 90);
    pid.setAngle(90, 1);
    pid.driveBackward(7, 5);
    LCD.WriteLine(RPS.Y());
    pid.setY(46, .25);
    pid.turnLeft(5, 90);
}



void burgerTask(){
    PID pid;


    vex.SetPercent(95);
    Sleep(.5);
    vex.Stop();
    vex.SetPercent(-25);
    Sleep(0.25);
    vex.Stop();
}
void burgerTaskToIceCream(){

    PID pid;

    pid.turnRight(10, 180);
    pid.driveForward(10,7);
    pid.turnRight(10, 15);
    pid.driveForward(10, 5);
    pid.driveBackward(10, 5);
}
