/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Driving;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.Config.PID;
import org.firstinspires.ftc.teamcode.Config.DriveClarityHandler;
import org.firstinspires.ftc.teamcode.Config.Config;
import org.firstinspires.ftc.teamcode.Config.DriveInit;
import org.firstinspires.ftc.teamcode.misc.PID;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

/**

 This is a teleop OpMode for a 4-motor holonomic drive robot. It allows the robot to move in three axes (directions) simultaneously
 using the left joystick for axial and lateral motion and the right joystick for yaw motion. It also includes a speed multiplier that
 can be adjusted with the dpad. The OpMode also includes code for controlling the movement of two linear slide motors using the
 left joystick on the gamepad2. The OpMode also includes code for controlling three servos using gamepad2 buttons.
 The code also includes an option to move linear slides into preset positions using the dpad on the gamepad2.
 */


/*

If I was you I wouldn't probably touch the things here unless something works horribly wrong, else if driver has no issue and everything works let this exist
 */

@TeleOp(name="Drive", group="Driving")

public class Drive extends LinearOpMode {
    public void runOpMode() {
        // Initialize configuration, drive initialization, and drive clarity handler objects
        Config cfg = new Config();
        DriveInit init = new DriveInit(cfg);
        DriveClarityHandler driveClarityHandler = new DriveClarityHandler();

        // Create PID controller for slides
        PID slidesPID = new PID(.02,.0,.02,.008);

        PID pivotPID = new PID(.02, .0, .02, .008);

        // Set initial error for PID controller
        slidesPID.getOutputFromError(0,0);
        pivotPID.getOutputFromError(0,0);

        int pivotMotorTargetPosition = 0;

        // Initialize loop variables
        double loopTime = 0;
        double turnInit = 0;
        double turnInit2 = 0;
        double lastPing = 0;
        boolean closeClaw = false;



        // Initialize drive motors and servos
        init.initDrive(hardwareMap);
        telemetry.addData(">", "Ready");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for start command
        waitForStart();

        // Reset runtime
        cfg.getrTime().reset();

        // BEGIN CODE
        while (opModeIsActive()) {

            // Get joystick values for holonomic drive
            double axial = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Update robot speed based on gamepad input
            driveClarityHandler.updateRobotSpeed(gamepad1, cfg);

            // Set holonomic drive motors based on joystick values
            driveClarityHandler.updateHolonomicDriveMotors(axial, lateral, yaw, cfg.getLfD(), cfg.getRfD(), cfg.getLbD(),cfg.getRbD(), cfg);

            // Update slide motors based on gamepad input
            driveClarityHandler.updateSlideMotors(gamepad2, slidesPID, cfg);

            // Update claw servos based on gamepad input
           // closeClaw = driveClarityHandler.updateGamepadServos(gamepad2, closeClaw, cfg);

         /*   // Update cone servos based on gamepad input
           double[] updatedServoValues = driveClarityHandler.updateConeServos(gamepad2, closeClaw, turnInit, turnInit2, pivotMotorTargetPosition, cfg);
            turnInit = updatedServoValues[0];
            turnInit2 = updatedServoValues[1];
            closeClaw = updatedServoValues[2] == 1;


            // Update servos after delay based on gamepad input
            double[] updatedServoValuesAfterDelay = driveClarityHandler.updateServosAfterDelay(turnInit, turnInit2, lastPing, closeClaw, pivotMotorTargetPosition, cfg);
            turnInit = updatedServoValuesAfterDelay[0];
            turnInit2 = updatedServoValuesAfterDelay[1];
            lastPing = updatedServoValuesAfterDelay[2];
            pivotMotorTargetPosition = (int) updatedServoValuesAfterDelay[3];

            driveClarityHandler.updatePivotMotor(pivotMotorTargetPosition, pivotPID, cfg);
*/
            // Add telemetry data
            telemetry.addData("Status", "Run Time: " + cfg.getrTime().toString());
            telemetry.addLine("Motors");
            telemetry.addData("Front left/Right", axial + lateral + yaw);
            telemetry.addData("Back left/Right", axial - lateral + yaw);
            /*
            telemetry.addLine("Servos");
            telemetry.addData("Close Claw", closeClaw);
            telemetry.addData("Claw, Rotate, Pivot", "%4.2f, %4.2f, %d", cfg.getClawServo().getPosition(), cfg.getRotateServo().getPosition(), cfg.getPivotMotor().getCurrentPosition());
            telemetry.addLine("Motor Rotations (Current vs Set)");
            telemetry.addData("Slide1 Position", "%d, %d", cfg.getSlide1Motor().getCurrentPosition(), cfg.getSlide1MotorTargetPosition());
            telemetry.addData("Slide2 Position", "%d, %d", cfg.getSlide2Motor().getCurrentPosition(), cfg.getSlide2MotorTargetPosition());
            telemetry.addData("Slides Power", "%4.2f", cfg.getSlide1Motor().getPower());
            telemetry.addData("Timers (s)", "%4.2f, %4.2f, %4.2f", turnInit/1000, turnInit2/1000, lastPing/1000);
            telemetry.addData("Loop timer", "%4.2f", cfg.getrTime().milliseconds() - loopTime);
            //telemetry.addData("Sensors (Chassis, Claw) cm", "%4.2f, %4.2f", chassisSensor.getDistance(DistanceUnit.CM), coneSensor.getDistance(DistanceUnit.CM));
            loopTime = cfg.getrTime().milliseconds();
            telemetry.update();

            */
        }
    }
}

