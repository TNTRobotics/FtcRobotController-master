package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Config.Config;
import org.firstinspires.ftc.teamcode.Config.Drive1ClarityHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.misc.PID;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PlacementPosition;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipelineBlueClose;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipelineRedClose;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.atomic.AtomicReference;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive \uD83D\uDFE5")
public class CamRedClose extends LinearOpMode {

    PropDetectionPipelineBlueClose propDetectionRed;
    String webcamName = "Webcam 1";

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    private VisionPortal visionPortal2;
    private PropDetectionPipelineRedClose propDetector;

    PID slidesPID = new PID(.005,.0,.02,.008);

    PID pivotPID = new PID(.002, .0, .02, .008);

    @Override


    public void runOpMode() throws InterruptedException {
        Config cfg = new Config();

        propDetector = new PropDetectionPipelineRedClose();

        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(propDetector)
                .enableLiveView(true)
                .build();
        Servo clawServo = (Servo) hardwareMap.get(Servo.class, "clawServo");
        Servo clawServo1 = (Servo) hardwareMap.get(Servo.class, "clawServo1");
        Servo rotateServo = (Servo) hardwareMap.get(Servo.class, "rotateServo");

        DcMotor pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        DcMotor pivot2Motor = hardwareMap.get(DcMotor.class, "pivot2Motor");
        DcMotor slide1Motor = hardwareMap.get(DcMotor.class, "s1");


        cfg.slide1Motor = slide1Motor;
        cfg.pivot2Motor = pivot2Motor;
        cfg.pivotMotor = pivotMotor;
        cfg.rotateServo = rotateServo;
        while (opModeInInit()) {
            clawServo.setPosition(1);
            clawServo1.setPosition(0);
            rotateServo.setPosition(0);
            telemetry.addLine("ready");
            telemetry.addData("position", propDetector.getPlacementPosition());
            telemetry.addData("1: ", propDetector.getRedAmount1());
            telemetry.addData("2: ", propDetector.getRedAmount2());
            telemetry.update();
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        Drive1ClarityHandler drive1ClarityHandler = new Drive1ClarityHandler();
        AtomicReference<Drive1ClarityHandler.LIFT_POSITIONS> liftPosition = new AtomicReference<>(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_0);

       // AtomicReference<Drive1ClarityHandler.LIFT_POSITIONS> liftPosition = new AtomicReference<>(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_0);

        PlacementPosition placementPosition = propDetector.getPlacementPosition();

        TrajectorySequence traj;
        TrajectorySequence traj2;
        TrajectorySequence traj3;

        if (placementPosition == PlacementPosition.CENTER) {
            drive.setPoseEstimate(new Pose2d(14, -62, Math.toRadians(90)));
            traj = drive.trajectorySequenceBuilder(new Pose2d(14, -62, Math.toRadians(90)))
//move forward to drop pixel on spike mark
                    .strafeTo(new Vector2d(14, -39))
                    //start claw movement down
                    .addTemporalMarker(0, () -> {
                        rotateServo.setPosition(1);
                    })
                    //open claw
                    .addTemporalMarker(1.5, () -> {
                        clawServo.setPosition(.7);
                    })
                    //move back slightly
                    .strafeTo(new Vector2d(14, -42))
                    //move claw back up
                    .addTemporalMarker(2.5, () -> {
                        rotateServo.setPosition(0);
                    })
                    //turn to face the backdrop
                    .turn(Math.toRadians(90))
                    //start moving pivot to set pos
                    .addTemporalMarker(3, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_1);
                    })
                    //move to backdrop
                    .strafeTo(new Vector2d(48, -33))
                    //open claw to drop pixel
                    .addTemporalMarker(5.3, () -> {
                        clawServo1.setPosition(.3);
                    })
                    //go back and sideways a little
                    .strafeTo(new Vector2d(45, -36))
                    //start set pos
                    .addTemporalMarker(7.5, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_0);
                    })

                    //go to the corner
                    .strafeTo(new Vector2d(47, -60))

                    .build();
            drive.followTrajectorySequenceAsync(traj);
        }
        else if (placementPosition == PlacementPosition.LEFT) {
            drive.setPoseEstimate(new Pose2d(14, -62, Math.toRadians(90)));
            traj3 = drive.trajectorySequenceBuilder(new Pose2d(14, -62, Math.toRadians(90)))
                    //move to spike mark
                    .strafeTo(new Vector2d(14, -42))
                    //turn to face backwards to the backdrop
                    .turn(Math.toRadians(90))
                    //start claw movement down
                    .addTemporalMarker(.8, () -> {
                        rotateServo.setPosition(1);
                    })
                   //open claw
                    .addTemporalMarker(1.9, () -> {
                        clawServo.setPosition(.7);
                    })
                    //move back slightly
                    //.strafeTo(new Vector2d(12, -42))
                    //start turning to backdrop
                    //claw up
                    .addTemporalMarker(1.9, () -> {
                        rotateServo.setPosition(0);
                    })
                    //start going to backdrop
                    .strafeTo(new Vector2d(48, -26))
                    //move back
                    //start set pos for pivot
                    .addTemporalMarker(4.1, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_1);
                    })
                    //open claw broken line of code
                    .addTemporalMarker(4.9, () -> {
                        clawServo1.setPosition(.3);
                    })
                    //reset pivot
                    .addTemporalMarker(5.3, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_0);
                    })
                    //go to corner
                    .strafeTo(new Vector2d(50, -60))
                    .build();
            drive.followTrajectorySequenceAsync(traj3);

        } else {

            drive.setPoseEstimate(new Pose2d(14, -62, Math.toRadians(90)));
            traj2 = drive.trajectorySequenceBuilder(new Pose2d(14, -62, Math.toRadians(90)))

                    .strafeTo(new Vector2d(14, -42))
                    //turn to face backwards to the backdrop
                    .turn(Math.toRadians(-90))
                    //start claw movement down

                    .addTemporalMarker(1.3, () -> {
                        rotateServo.setPosition(1);
                    })


                    //open claw
                    .addTemporalMarker(1.9, () -> {
                        clawServo.setPosition(.7);
                    })

                    //move back slightly
                    //.strafeTo(new Vector2d(12, -42))
                    //start turning to backdrop
                    //claw up
                    .addTemporalMarker(1.9, () -> {
                        rotateServo.setPosition(0);
                    })
                    .turn(Math.toRadians(180))
                    //start going to backdrop
                    .strafeTo(new Vector2d(48, -43))

                    //move back
                    //start set pos for pivot
                    .addTemporalMarker(4.6, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_5);
                    })

                    //open claw broken line of code
                    .addTemporalMarker(6.2, () -> {
                        clawServo1.setPosition(.15);
                    })
                    .waitSeconds(.5)
                    .addTemporalMarker(6.5, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_2);
                    })
                    .strafeTo(new Vector2d(35, -43))
                    //reset pivot
                    .addTemporalMarker(7.3, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_0);
                    })
                    //go to corner
                    .strafeTo(new Vector2d(50, -60))



                    .build();
            drive.followTrajectorySequenceAsync(traj2);

        }

        while (opModeIsActive()) {
            drive.update();

            telemetry.addData("Lift target position: ", liftPosition.get());

            drive1ClarityHandler.updateSlideMotorsAuto(slidesPID, pivotPID, cfg , liftPosition.get(), telemetry);
            telemetry.update();

        }


    }
}
