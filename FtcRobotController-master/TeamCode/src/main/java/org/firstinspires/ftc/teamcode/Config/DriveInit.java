package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This class initializes the drive motors, linear slide motors, and servos for a robot. It uses the
 * HardwareMap class to map the hardware devices to their corresponding variables, sets their directions
 * and behaviors, and initializes their positions. It also includes a speed multiplier that can be used
 * to adjust the speed of the drive motors.
 */


public class DriveInit {

    Config cfg;

    public DriveInit(Config cfg) {
        this.cfg = cfg;
    }

    public void initDrive(HardwareMap hwMap) {
        // Declare OpMode members for each of the 4 motors.
        ElapsedTime runtime = new ElapsedTime();

        DcMotor leftFrontDrive = null;
        DcMotor leftBackDrive = null;
        DcMotor rightFrontDrive = null;
        DcMotor rightBackDrive = null;

        DcMotor slide1Motor;
        DcMotor pivotMotor;
//DcMotor slide2Motor = null;
        Servo clawServo;
        //Servo rotateServo;


        double speedMultiplier = 1;

        //double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
        // ASSIGN DRIVE MOTORS
        leftFrontDrive = (hwMap.get(DcMotor.class, "leftFront"));
        leftBackDrive = hwMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hwMap.get(DcMotor.class, "rightRear");


        // ASSIGN LINEAR SLIDE / ARM MOTOR
        slide1Motor = hwMap.get(DcMotor.class, "s1");//slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        pivotMotor = hwMap.get(DcMotor.class, "pivotMotor");
       /* slide2Motor = hwMap.get(DcMotor.class, "s2");//slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");


        // ASSIGN SERVOS

        rotateServo = hwMap.get(Servo.class, "rotateServo");


        rotateServo.setPosition(0);
        */
        clawServo = hwMap.get(Servo.class, "clawServo");
        clawServo.setPosition(0);

        // DRIVE MOTOR DIRECTION
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // BRAKE
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ARM MOTOR DIRECTION
       slide1Motor.setDirection(DcMotorSimple.Direction.FORWARD); // TEST FORWARD OR BACKWARDS
       // slide2Motor.setDirection(DcMotorSimple.Direction.FORWARD); // TEST FORWARD OR BACKWARDS

        slide1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //slide2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // reset encoder
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
*/
        slide1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  /*      slide2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        slide2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


         */

        // Setters

        //Speed
        cfg.setSpeedMultiplier(speedMultiplier);

        cfg.setrTime(runtime);


        // Motors
        cfg.setLfD(leftFrontDrive);
        cfg.setLbD(leftBackDrive);
        cfg.setRfD(rightFrontDrive);
        cfg.setRbD(rightBackDrive);

        cfg.setSlide1Motor(slide1Motor);
        cfg.setPivotMotor(pivotMotor);
       // cfg.setSlide2Motor(slide2Motor);

        // Servos
        cfg.setClawServo(clawServo);
       // cfg.setRotateServo(rotateServo);
        cfg.slidesPower = 0;
        cfg.pivotPower = 0;



    }
}