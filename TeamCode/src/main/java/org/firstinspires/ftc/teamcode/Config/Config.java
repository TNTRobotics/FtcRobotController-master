package org.firstinspires.ftc.teamcode.Config;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**

 This class is used to store and retrieve global variables such as drive motors, linear slide motors,
 servos, and runtime for a robot. It also includes a speed multiplier that can be used to adjust the
 speed of the drive motors and servo position variables. It provides setters and getters for each
 of these variables.

 */


public class Config {


    public int slidesPower;

    public int climbPower;
    // Motors (left front drive, left back drive, right front drive, right back drive)
    DcMotor lfD = null;
    DcMotor lbD = null;
    DcMotor rfD = null;
    DcMotor rbD = null;

    public DcMotor slide1Motor = null; //temp public for testing
    public int pivotPower;
    public DcMotor pivotMotor = null;
    public DcMotor pivot2Motor = null;

    public DcMotor climb = null;
    //DcMotor slide2Motor = null;

    // Runtime
    ElapsedTime rTime = new ElapsedTime();

    // Speed
    double speedMultiplier = 1.0;


    // Position of the slide from encoders
    int slide1Position = 0;
    int pivotPosition = 0;

    // Claw (A1 - Claw, A2 - 180 turn, A3 - Pivot turn)
    public Servo clawServo;
    public Servo clawServo1;
    public Servo rotateServo;
    public Servo plane;
    public CRServo sclimb;


    // Servos Misc
    final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    final double SINCREMENT = .5;
    // targetPos
     int slide1MotorTargetPosition = 0;
   // int slide2MotorTargetPosition = 0;

    // SETTERS AND GETTERS

    public DcMotor getLfD() {
        return lfD;
    }

    public void setLfD(DcMotor lfD) {
        this.lfD = lfD;
    }

    public DcMotor getLbD() {
        return lbD;
    }

    public void setLbD(DcMotor lbD) {
        this.lbD = lbD;
    }

    public DcMotor getRfD() {
        return rfD;
    }

    public void setRfD(DcMotor rfD) {
        this.rfD = rfD;
    }

    public DcMotor getRbD() {
        return rbD;
    }

    public void setRbD(DcMotor rbD) {
        this.rbD = rbD;
    }


    public ElapsedTime getrTime() {
        return rTime;
    }

    public void setrTime(ElapsedTime rTime) {this.rTime = rTime;}



    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public void setSpeedMultiplier(double speedMultiplier) {this.speedMultiplier = speedMultiplier;}


    public int getSlide1Position() {return slide1Position;}

    public void setSlide1Position(int slide1Position) {this.slide1Position = slide1Position;}

    public int getPivotPosition() {return pivotPosition;}

    public void setPivotPosition(int pivotPosition) {this.pivotPosition = pivotPosition;}


    public double getINCREMENT() {return INCREMENT;}
    public double getSINCREMENT() {return SINCREMENT;}

    public int getSlide1MotorTargetPosition() {return slide1MotorTargetPosition;}

}