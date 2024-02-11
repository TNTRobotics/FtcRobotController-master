package org.firstinspires.ftc.teamcode.Config;


import org.firstinspires.ftc.robotcore.external.Telemetry;
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

public class Drive1ClarityHandler {


    double max;
    /**
     * Set positions for the pivot motor code. Just makes life easier.
     **/
    int pivotPos = 0;

    boolean clawMoved = false;
    boolean setpos2 = false;

    public enum LIFT_POSITIONS {
        LEVEL_0,
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
        LEVEL_4,
        LEVEL_5,

    }


    /**
     * Sets up chassis drive and what each motor will be doing.
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

    /**
     * What this does- this is set positions that combines the claw pivot, linear slide, and linear slide pivot.
     *
     * @return
     */
    public boolean updateSlideMotors(Gamepad gamepad1, PID slidesPID, PID pivotPID, Gamepad currentGamepad1, Gamepad previousGamepad1, boolean closeClaw, Config cfg) {
        /**
         This next part is making is so that the pivot motors and the linear slide motor can be controlled by the player through the joysticks. This also prevents either of them
         from going over a set limit so that nothing would break.
         * */
        /**
         For the pivot motor here, we might want to slow it down ever so slightly (not the joystick part) like the chassis code is doing so that we do not have the whole robot shake. This
         will also simplify the code a lot.
         **/
        // double slidesPower = -gamepad2.left_stick_y * 10;slidesPower
        //double pivotPower = -gamepad2.right_stick_y * 4;pivotPower
        int pivotPos = (int) (cfg.getPivotPosition());
        int armNewPos = (int) (cfg.getSlide1Position());
        if (armNewPos < -2000) {
            armNewPos = -2000;
        }
        if (armNewPos > 0) {
            armNewPos = 0;
        }
        if (pivotPos < -10) {
            pivotPos = -10;
        }
        if (pivotPos > 1500) {
            pivotPos = 1500;
        }
        /**
         Here, it starts the actual set positions, it initializes everything for that including PID and the motors.
         **/
        double currentArmPID = slidesPID.getOutputFromError(armNewPos, cfg.slide1Motor.getCurrentPosition());
        double currentPivotPID = pivotPID.getOutputFromError(pivotPos, cfg.pivotMotor.getCurrentPosition());
        /**
         Dpad up here makes the pivot motor go to preset position 2, then 1 so that the slides don't hit anything on the floor. This also will help with a more peaceful transition
         from the lowest point of the pivot to the highest point. After that, the linear slides extend to the max hight allowed, and the pivot servo turns to the other side
         to be able to drop the pixels on the board.
         **/
        if (currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button) {
            setpos2 = !setpos2;
            if (setpos2) {
                pivotPos = 1300;
                armNewPos = -1200;
                cfg.rotateServo.setPosition(0);
            } else {
                armNewPos = 0;
                pivotPos = -50;
            }
        }
        if (gamepad1.dpad_up) {
            pivotPos = 1300;
            armNewPos = -1200;
            cfg.rotateServo.setPosition(0);
        }
        /**
         This just raises the linear slide to halfway.
         **/
        if (gamepad1.dpad_right) {
            pivotPos = 1300;
            armNewPos = -800;
            cfg.rotateServo.setPosition(0);
        }
        /**
         This moves the slides out ever so slightly.
         **/
        if (gamepad1.dpad_left) {
            armNewPos = -1800;
            pivotPos = 1300;
            cfg.rotateServo.setPosition(0);
        }
        /**
         This moves the linear slides to the minimum positions so that the arm can safely go down. After that, the pivot motor will go to a middle position, break, and go to the lowest
         position. This will keep it from breaking anything. The claw then flips to the other side of the slide to be ready to pick up the pixels again.
         **/
        if (gamepad1.dpad_down) {
            armNewPos = 0;
            pivotPos = 0;
        }
        /**
         * This will move up the linear slide so that we could knock over the pixel stacks easily without any problems.
         */
        if (gamepad1.circle) {
            cfg.plane.setPosition(1);
        } else if (gamepad1.square) {
            cfg.climb.setPower(-1);
        } else if (gamepad1.triangle) {
            cfg.sclimb.setPower(-.35);
        } else {
            cfg.sclimb.setPower(0);
            cfg.climb.setPower(0);
        }
        if (gamepad1.cross) {
            cfg.rotateServo.setPosition(1.5);
        }
        if (currentGamepad1.right_stick_button && !previousGamepad1.right_stick_button) {
            clawMoved = !clawMoved;
            if (clawMoved) {
                cfg.rotateServo.setPosition(0);
                armNewPos = 0;
                pivotPos = 0;
            } else {
                cfg.rotateServo.setPosition(1.5);
                armNewPos = 0;
                pivotPos = 0;

            }
        }

        /**
         This here, we have yet to figure out exactly what it does. It maybe just configures the motors and servos again saying that they are not needed anymore until they are called.
         This is just a theory though. We also now have motor power controls for the pivot motor so that the motor does not shake too much or go too quickly between
         set positions.
         **/
        cfg.slide1Motor.setPower(currentArmPID);
        cfg.setSlide1Position(armNewPos);
        cfg.pivotMotor.setPower(currentPivotPID * .8);
        cfg.pivot2Motor.setPower(currentPivotPID * .8);
        cfg.setPivotPosition(pivotPos);
        return closeClaw;

    }

    /**
     * This below updates the claws so that they close and open, left is left claw, and right is right claw.
     **/
    public boolean updateGamepadServos(Gamepad gamepad1, Gamepad currentGamepad1, Gamepad previousGamepad1, boolean closeClaw, Config cfg) {
        if (gamepad1.left_trigger != 0) {
            cfg.clawServo.setPosition(.7);
            closeClaw = true;
        }
        if (gamepad1.left_bumper) {
            cfg.clawServo.setPosition(1);
            closeClaw = false;
        }
        if (gamepad1.right_trigger != 0) {
            cfg.clawServo1.setPosition(0.3);
            closeClaw = true;
        }
        if (gamepad1.right_bumper) {
            cfg.clawServo1.setPosition(0);
            closeClaw = false;
            // END OF CLAW 1
            /**
             This is the location of additional control for the pivot servo. This is how we will be able to fine tune it and correct it during competition.
             **/

        }



       /* if (currentGamepad1.cross && !previousGamepad1.cross) {
            clawMoved = !clawMoved;
            if (clawMoved) {
                cfg.rotateServo.setPosition(.63);
            } else {
                cfg.rotateServo.setPosition(0);
            }




            // END OF CLAW 2

        }

        */
        return closeClaw;

    }

    public boolean updateSlideMotorsAuto(PID slidesPID, PID pivotPID, Config cfg, LIFT_POSITIONS targetPos, Telemetry t) {
        /**
         This next part is making is so that the pivot motors and the linear slide motor can be controlled by the player through the joysticks. This also prevents either of them
         from going over a set limit so that nothing would break.
         * */
        /**
         For the pivot motor here, we might want to slow it down ever so slightly (not the joystick part) like the chassis code is doing so that we do not have the whole robot shake. This
         will also simplify the code a lot.
         **/
        // double slidesPower = -gamepad2.left_stick_y * 10;slidesPower
        //double pivotPower = -gamepad2.right_stick_y * 4;pivotPower
        int pivotPos = (int) (cfg.getPivotPosition());
        int armNewPos = (int) (cfg.getSlide1Position());
        if (armNewPos < -2000) {
            armNewPos = -2000;
        }
        if (armNewPos > 0) {
            armNewPos = 0;
        }

            if (pivotPos < -10) {
                pivotPos = -10;
            }
            if (pivotPos > 1500) {
                pivotPos = 1500;
            }

            /**
             Here, it starts the actual set positions, it initializes everything for that including PID and the motors.
             **/
            double currentArmPID = slidesPID.getOutputFromError(armNewPos, cfg.slide1Motor.getCurrentPosition());
            double currentPivotPID = pivotPID.getOutputFromError(pivotPos, cfg.pivotMotor.getCurrentPosition());
            /**
             Dpad up here makes the pivot motor go to preset position 2, then 1 so that the slides don't hit anything on the floor. This also will help with a more peaceful transition
             from the lowest point of the pivot to the highest point. After that, the linear slides extend to the max hight allowed, and the pivot servo turns to the other side
             to be able to drop the pixels on the board.
             **/

            if (targetPos == LIFT_POSITIONS.LEVEL_2) {
                pivotPos = 1350;
                armNewPos = -800;
                cfg.rotateServo.setPosition(0);
            }
            /**
             This just raises the linear slide to halfway.
             **/
            if (targetPos == LIFT_POSITIONS.LEVEL_1) {
                pivotPos = 1350;
                armNewPos = -450;
                cfg.rotateServo.setPosition(0);
            }
        if (targetPos == LIFT_POSITIONS.LEVEL_5) {
            pivotPos = 1350;
            armNewPos = -350;
            cfg.rotateServo.setPosition(0);
        }
            /**
             This moves the slides out ever so slightly.
             **/
            if (targetPos == LIFT_POSITIONS.LEVEL_3) {
                armNewPos = -1800;
                pivotPos = 1500;
                cfg.rotateServo.setPosition(0);
            }
            if (targetPos == LIFT_POSITIONS.LEVEL_4) {
                armNewPos = 0;
                pivotPos = 350;
            }
            /**
             This moves the linear slides to the minimum positions so that the arm can safely go down. After that, the pivot motor will go to a middle position, break, and go to the lowest
             position. This will keep it from breaking anything. The claw then flips to the other side of the slide to be ready to pick up the pixels again.
             **/
            if (targetPos == LIFT_POSITIONS.LEVEL_0) {
                armNewPos = 0;
                pivotPos = 0;
            }


            /**
             This here, we have yet to figure out exactly what it does. It maybe just configures the motors and servos again saying that they are not needed anymore until they are called.
             This is just a theory though. We also now have motor power controls for the pivot motor so that the motor does not shake too much or go too quickly between
             set positions.
             **/
            cfg.slide1Motor.setPower(currentArmPID);
            cfg.setSlide1Position(armNewPos);
            cfg.pivotMotor.setPower(currentPivotPID * 1 );
            cfg.pivot2Motor.setPower(currentPivotPID * 1);
            cfg.setPivotPosition(pivotPos);




        t.addData("pivot 1 power: ", cfg.pivotMotor.getPower());
        t.addData("pivot 2 power: ", cfg.pivot2Motor.getPower());
        t.addData("slides 1 power: ", cfg.slide1Motor.getPower());
        return false;
    }
}
