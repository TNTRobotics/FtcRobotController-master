package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "IMUTestFileJava")
public class IMUTestFileJava extends LinearOpMode {

    private IMU imu;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");

        // Put initialization blocks here.

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.addData("Robot Orientation", imu.getRobotOrientationAsQuaternion());
                telemetry.addData("Robot Angle", imu.getRobotYawPitchRollAngles());
                telemetry.update();
            }
        }
    }
}
