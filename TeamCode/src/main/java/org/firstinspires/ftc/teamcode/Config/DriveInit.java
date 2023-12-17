package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.CRServo;
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
        DcMotor pivot2Motor;
        DcMotor climb;
//DcMotor slide2Motor = null;
        Servo clawServo;
        Servo clawServo1;
        Servo rotateServo;
        Servo plane;
        CRServo sclimb;

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
        pivot2Motor = hwMap.get(DcMotor.class, "pivot2Motor");
        climb = hwMap.get(DcMotor.class, "climb");
       // slide2Motor = hwMap.get(DcMotor.class, "s2");//slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        // ASSIGN SERVOS
        clawServo = hwMap.get(Servo.class, "clawServo");
        clawServo.setPosition(1);
        clawServo1 = hwMap.get(Servo.class, "clawServo1");
        clawServo1.setPosition(0);
        rotateServo = hwMap.get(Servo.class, "rotateServo");
        rotateServo.setPosition(.63);
        plane = hwMap.get(Servo.class, "plane");
        plane.setPosition(.8);
        sclimb = hwMap.get(CRServo.class, "sclimb");

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

        pivot2Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        pivot2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        climb.setDirection(DcMotorSimple.Direction.FORWARD);
        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        pivot2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        cfg.slide1Motor=slide1Motor;
        cfg.pivotMotor = pivotMotor;
        cfg.pivot2Motor = pivot2Motor;
        cfg.climb = climb;
       // cfg.setSlide2Motor(slide2Motor);

        // Servos
        cfg.clawServo = clawServo;
        cfg.clawServo1 = clawServo1;
        cfg.rotateServo = rotateServo;
        cfg.plane = plane;
        cfg.slidesPower = 0;
        cfg.pivotPower = 0;
        cfg.climbPower = 0;
        cfg.sclimb = sclimb;



    }
}