package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Red Close")
public class RedClose extends LinearOpMode {

    private IMU imu;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
//setting up motors for auto
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));


        waitForStart();
        telemetry.addData("Robot Angle", Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-90));
        telemetry.update();

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
        waitForStart();


        leftFrontDrive.setVelocity(600);
        rightBackDrive.setVelocity(600);
        leftBackDrive.setVelocity(600);
        rightFrontDrive.setVelocity(600);

        sleep(500);

        while(Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+84)>1){

            leftFrontDrive.setVelocity(-300);
            rightBackDrive.setVelocity(300);
            leftBackDrive.setVelocity(-300);
            rightFrontDrive.setVelocity(300);

        }

        leftFrontDrive.setVelocity(600);
        rightBackDrive.setVelocity(600);
        leftBackDrive.setVelocity(600);
        rightFrontDrive.setVelocity(600);

        sleep(3500);

        leftFrontDrive.setVelocity(0);
        rightBackDrive.setVelocity(0);
        leftBackDrive.setVelocity(0);
        rightFrontDrive.setVelocity(0);

    }
}
