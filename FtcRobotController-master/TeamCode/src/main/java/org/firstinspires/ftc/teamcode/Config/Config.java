package org.firstinspires.ftc.teamcode.Config;


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
    // Motors (left front drive, left back drive, right front drive, right back drive)
    DcMotor lfD = null;
    DcMotor lbD = null;
    DcMotor rfD = null;
    DcMotor rbD = null;

    DcMotor slide1Motor = null;
    public int pivotPower;
    DcMotor pivotMotor = null;
    //DcMotor slide2Motor = null;

    // Runtime
    ElapsedTime rTime = new ElapsedTime();

    // Speed
    double speedMultiplier = 1.0;


    // Position of the slide from encoders
    int slide1Position = 0;
    int pivotPosition = 0;

    // Claw (A1 - Claw, A2 - 180 turn, A3 - Pivot turn)
   // Servo clawServo;
   // Servo rotateServo;


    // Servos Misc
   // final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle

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

    public DcMotor getSlide1Motor() {
        return slide1Motor;
    }
    
    /*
        public DcMotor getSlide2Motor() {
            return slide2Motor;
        }
    
        public void setSlide2Motor(DcMotor slide2Motor) {this.slide2Motor = slide2Motor;}
    */
    public ElapsedTime getrTime() {
        return rTime;
    }

    public void setrTime(ElapsedTime rTime) {this.rTime = rTime;}



    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public void setSpeedMultiplier(double speedMultiplier) {this.speedMultiplier = speedMultiplier;}

    public void setSlide1Motor(DcMotor slide1Motor) {
        this.slide1Motor = slide1Motor;
    }

    public void setPivotMotor(DcMotor pivotMotor) {
        this.pivotMotor = pivotMotor;
    }

    public int getSlide1Position() {return slide1Position;}

    public void setSlide1Position(int slide1Position) {this.slide1Position = slide1Position;}

    public int getPivotPosition() {return pivotPosition;}

    public void setPivotPosition(int pivotPosition) {this.pivotPosition = pivotPosition;}
/*
    public Servo getClawServo() {
        return clawServo;
    }

    public void setClawServo(Servo clawServo) {
        this.clawServo = clawServo;
    }

    public Servo getRotateServo() {
        return rotateServo;
    }

    public void setRotateServo(Servo rotateServo) {
        this.rotateServo = rotateServo;
    }

   
    public int getSlide2MotorTargetPosition() {return slide2MotorTargetPosition;}

 */
public DcMotor getPivotMotor() {return pivotMotor;}

    
  //  public double getINCREMENT() {return INCREMENT;}

    public int getSlide1MotorTargetPosition() {return slide1MotorTargetPosition;}

}