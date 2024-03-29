package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
@Autonomous(name = "Blue Close \uD83D\uDFE6")
public class BlueClose extends LinearOpMode {

    private IMU imu;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
//setting up motors for auto
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));





        //motor init for driving
        DcMotorEx leftFrontDrive = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftFront");
        DcMotorEx leftBackDrive = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftRear");
        DcMotorEx rightFrontDrive = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightFront");
        DcMotorEx rightBackDrive = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightRear");
        //motors run with encoder
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
/*
        //slide and pivot motor init
        DcMotorEx slide1Motor = (DcMotorEx) hardwareMap.get(DcMotor.class, "slide1Motor");
        DcMotorEx pivot2Motor = (DcMotorEx) hardwareMap.get(DcMotor.class, "pivot2Motor");
        DcMotorEx pivotMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "pivotMotor");
        // run with encoders
        slide1Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide1Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot2Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

 */

        //servo init

        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        Servo clawServo1 = hardwareMap.get(Servo.class, "clawServo1");
        Servo rotateServo = hardwareMap.get(Servo.class, "rotateServo");
        clawServo.setPosition(1);
        clawServo1.setPosition(0);
        rotateServo.setPosition(.63);

        waitForStart();

        imu.resetYaw();
        telemetry.addData("Robot Angle", Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-90));
        telemetry.update();



        leftFrontDrive.setVelocity(1000);
        rightBackDrive.setVelocity(1000);
        leftBackDrive.setVelocity(1000);
        rightFrontDrive.setVelocity(1000);

        sleep(500);


        while(Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-80)>1){

            leftFrontDrive.setVelocity(1000);
            rightBackDrive.setVelocity(-1000);
            leftBackDrive.setVelocity(1000);
            rightFrontDrive.setVelocity(-1000);

        }

        leftFrontDrive.setVelocity(1500);
        rightBackDrive.setVelocity(1500);
        leftBackDrive.setVelocity(1500);
        rightFrontDrive.setVelocity(1500);

        sleep(800);

        leftFrontDrive.setVelocity(0);
        rightBackDrive.setVelocity(0);
        leftBackDrive.setVelocity(0);
        rightFrontDrive.setVelocity(0);

        rotateServo.setPosition(0);

        sleep(500);



        clawServo.setPosition(.8);
        clawServo1.setPosition(0.2);

        sleep(500);

        leftFrontDrive.setVelocity(-100);
        rightBackDrive.setVelocity(-100);
        leftBackDrive.setVelocity(-100);
        rightFrontDrive.setVelocity(-100);

        sleep(300);

    }
}
