package org.firstinspires.ftc.teamcode.Config;


import org.firstinspires.ftc.teamcode.misc.PID;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/*
 This class handles the control of the holonomic drive motors, robot speed, and linear slide motors
 using input from gamepads. It includes methods to update the motor speeds for the holonomic drive
 based on input from the gamepad's axial, lateral, and yaw values, as well as a method to update the
 robot's speed based on input from the gamepad's buttons. It also includes a method to update the
 linear slide motors based on input from the gamepad's left stick y-axis and dpad, and a method to
 update the servos for the claw, rotate, and pivot based on input from the gamepad's buttons.
 */




/**
This is where all of the actual code is.
 **/

public class DriveClarityHandler {

    double max;
    /**
Set positions for the pivot motor code. Just makes life easier.
     **/
    int pivotPos = 0;
    int[] pivotPositions = {
            0, // Starting
            100, // Half way
            170, // All the way
            50, // Little bit from the ground
    };


    /**
Sets up chassis drive and what each motor will be doing.
     **/
    public void updateHolonomicDriveMotors(double axial, double lateral, double yaw, DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4, Config cfg) {
        // Calculate motor speeds here
        double leftFrontMotor = axial + lateral + yaw;
        double rightFrontMotor = axial - lateral - yaw;
        double leftBackMotor = axial - lateral + yaw;
        double rightBackMotor = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontMotor), Math.abs(rightFrontMotor));
        max = Math.max(max, Math.abs(leftBackMotor));
        max = Math.max(max, Math.abs(rightBackMotor));
        /**
Works on max speed for chassis.
         **/
        if (max > 1.0) {
            leftFrontMotor /= max;
            rightFrontMotor /= max;
            leftBackMotor /= max;
            rightBackMotor /= max;
        }
        /**
Chassis speed stuff.
         **/
        cfg.getLfD().setPower(leftFrontMotor * cfg.getSpeedMultiplier());
        cfg.getRfD().setPower(rightFrontMotor * cfg.getSpeedMultiplier());
        cfg.getLbD().setPower(leftBackMotor * cfg.getSpeedMultiplier());
        cfg.getRbD().setPower(rightBackMotor * cfg.getSpeedMultiplier());
    }

    public void updateRobotSpeed(Gamepad gamepad1, Config cfg) {
        /**
Actual configuration for the speed which can be max, 75%, 50%, and 25%.
         **/
        if (gamepad1.cross) {
            cfg.setSpeedMultiplier(1);
        }
        if (gamepad1.square) {
            cfg.setSpeedMultiplier(.75);
        }
        if (gamepad1.triangle) {
            cfg.setSpeedMultiplier(.5);
        }
        if (gamepad1.circle) {
            cfg.setSpeedMultiplier(.25);
        }

    }

    /**
     What this does- this is set positions that combines the claw pivot, linear slide, and linear slide pivot.
     **/
    public void updateSlideMotors(Gamepad gamepad2, PID slidesPID,  PID pivotPID, boolean closeClaw, Config cfg) throws InterruptedException {
        /**
        This next part is making is so that the pivot motors and the linear slide motor can be controlled by the player through the joysticks. This also prevents either of them
         from going over a set limit so that nothing would break.
        * */
        double pivotPower = -gamepad2.right_stick_y * 2;
        /**
For the pivot motor here, we might want to slow it down ever so slightly (not the joystick part) like the chassis code is doing so that we do not have the whole robot shake. This
         will also simplify the code a lot.
         **/
        double slidesPower = -gamepad2.left_stick_y * 10;
        int pivotPos = (int) (cfg.getPivotPosition() + pivotPower);
        int armNewPos = (int) (cfg.getSlide1Position() + slidesPower);
        if (armNewPos < -1400) {
            armNewPos = -1400;
        }
        if (armNewPos > 0) {
            armNewPos = 0;
        }
        if (pivotPos < -10) {
            pivotPos = -10;
        }
        if (pivotPos > 170) {
            pivotPos = 170;
        }
        /**
        Here, it starts the actual set positions, it initializes everything for that including PID and the motors.
                 **/
        double currentArmPID = slidesPID.getOutputFromError(armNewPos, cfg.getSlide1Motor().getCurrentPosition());
        double currentPivotPID = pivotPID.getOutputFromError(pivotPos,  cfg.getPivotMotor().getCurrentPosition());
        /**
        Dpad up here makes the pivot motor go to preset position 2, then 1 so that the slides don't hit anything on the floor. This also will help with a more peaceful transition
         from the lowest point of the pivot to the highest point. After that, the linear slides extend to the max hight allowed, and the pivot servo turns to the other side
         to be able to drop the pixels on the board.
         **/
        if (gamepad2.dpad_up) {
            wait(2);
            pivotPos = 170;
            armNewPos = -1400;
            if (cfg.getRotateServo().getPosition() <= 0) {
                cfg.getRotateServo().setPosition(0);
            } else {
                cfg.getRotateServo().setPosition(cfg.getRotateServo().getPosition() - cfg.getINCREMENT());
            }
        }
        /**
        This just raises the linear slide to halfway.
         **/
        if (gamepad2.dpad_left) {
            armNewPos = -1000;
        }
        /**
         This moves the slides out ever so slightly.
         **/
        if (gamepad2.dpad_right) {
            armNewPos = -150;
        }
        /**
This moves the linear slides to the minimum positions so that the arm can safley go down. After that, the pivot motor will go to a middle position, break, and go to the lowest
         position. This will keep it from breaking anything. The claw then flips to the other side of the slide to be ready to pick up the pixels again.
         **/
        if (gamepad2.dpad_down) {
            armNewPos = 0;
            wait(2);
            pivotPos = 60;
            wait(2);
            if (cfg.getRotateServo().getPosition() >= .65) {
                cfg.getRotateServo().setPosition(.65);
            } else {
                cfg.getRotateServo().setPosition(cfg.getRotateServo().getPosition() + cfg.getINCREMENT());
            }
        }
        /**
This here, we have yet to figure out exactly what it does. It maybe just configures the motors and servos again saying that they are not needed anymore until they are called.
         This is just a theory though.
         **/
        cfg.getSlide1Motor().setPower(currentArmPID);
        cfg.setSlide1Position(armNewPos);
        cfg.getPivotMotor().setPower(currentPivotPID);
        cfg.getPivot2Motor().setPower(currentPivotPID);
        cfg.setPivotPosition(pivotPos);


    }
    /**
    This originally called for the pivot motor to be moved through the use of the cross and square buttons. It also has joystick and min/max values configured in here.
     **/
   /* public void updatePivotMotor(Gamepad gamepad2, PID pivotPID, Config cfg) {
        double pivotPower = gamepad2.right_stick_y * 2;
        int pivotPos = (int) (cfg.getPivotPosition() + pivotPower);
        if (pivotPos < -10) {
            pivotPos = -10;
        }
        if (pivotPos > 170) {
            pivotPos = 170;
        }
        double currentPivotPID = pivotPID.getOutputFromError(pivotPos,  cfg.getPivotMotor().getCurrentPosition());
        if (gamepad2.cross) {
            pivotPos = 70;
        }
        if (gamepad2.square) {
            pivotPos = 100;
        }
        cfg.getPivotMotor().setPower(currentPivotPID);
        cfg.getPivot2Motor().setPower(currentPivotPID);

        cfg.setPivotPosition(pivotPos);
    }

    */
    /**
    This below updates the claws so that they close and open, left is left claw, and right is right claw.
     **/
    public boolean updateGamepadServos(Gamepad gamepad2, boolean closeClaw, Config cfg) {
        if (gamepad2.left_trigger !=0) {
            cfg.getClawServo().setPosition(0.8);
            closeClaw = true;
        }
        if (gamepad2.left_bumper) {
            cfg.getClawServo().setPosition(1);
            closeClaw = false;
        }
        if(gamepad2.right_trigger !=0){
            cfg.getClawServo1().setPosition(0.2);
            closeClaw = true;
        }
        if(gamepad2.right_bumper){
            cfg.getClawServo1().setPosition(0);
            closeClaw = false;
        }
        // END OF CLAW 1
        /**
    This is the location of additional control for the pivot motor. This is how we will be able to fine tune it and correct it during competition.
         **/
        // START OF CLAW 2 (180 turn around)
        if (gamepad2.circle ) {
            if (cfg.getRotateServo().getPosition() >= .65) {
                cfg.getRotateServo().setPosition(.65);
            } else {
                cfg.getRotateServo().setPosition(cfg.getRotateServo().getPosition() + cfg.getINCREMENT());
            }
        }
        if (gamepad2.triangle) {
            if (cfg.getRotateServo().getPosition() <= 0) {
                cfg.getRotateServo().setPosition(0);
            } else {
                cfg.getRotateServo().setPosition(cfg.getRotateServo().getPosition() - cfg.getINCREMENT());
            }
        }
        // END OF CLAW 2

        return closeClaw;
    }
}