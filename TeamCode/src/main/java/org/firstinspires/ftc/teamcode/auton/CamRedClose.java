package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PlacementPosition;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipelineBlueClose;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipelineRedClose;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class CamRedClose extends LinearOpMode {

    PropDetectionPipelineBlueClose propDetectionRed;
    String webcamName = "Webcam 1";

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    private VisionPortal visionPortal2;
    private PropDetectionPipelineRedClose propDetector;

    @Override


    public void runOpMode() throws InterruptedException {

        propDetector = new PropDetectionPipelineRedClose();

        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(propDetector)
                .enableLiveView(true)
                .build();

        while (opModeInInit()) {
            telemetry.addLine("ready");
            telemetry.addData("position", propDetector.getPlacementPosition());
            telemetry.addData("1: ", propDetector.getRedAmount1());
            telemetry.addData("2: ", propDetector.getRedAmount2());
            telemetry.update();
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo clawServo = (Servo) hardwareMap.get(Servo.class, "clawServo");
        Servo clawServo1 = (Servo) hardwareMap.get(Servo.class, "clawServo1");
        Servo rotateServo = (Servo) hardwareMap.get(Servo.class, "rotateServo");
        clawServo.setPosition(1);
        clawServo1.setPosition(0);
        rotateServo.setPosition(.63);
        DcMotor pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        DcMotor pivot2Motor = hardwareMap.get(DcMotor.class, "pivot2Motor");
        waitForStart();

        if (isStopRequested()) return;

        PlacementPosition placementPosition = propDetector.getPlacementPosition();

        if (placementPosition == PlacementPosition.CENTER) {
            drive.setPoseEstimate(new Pose2d(14, -62, Math.toRadians(90)));
            TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(14, -62, Math.toRadians(90)))

                    .strafeTo(new Vector2d(14, -39))
                    .addTemporalMarker(1, () -> {
                        rotateServo.setPosition(0);
                    })
                    .addTemporalMarker(2.5, () -> {
                        clawServo.setPosition(.7);
                    })
                    .strafeTo(new Vector2d(14, -42))
                    .addTemporalMarker(3.5, () -> {
                        rotateServo.setPosition(.63);
                    })


                    .turn(Math.toRadians(-88))
                    .strafeTo(new Vector2d(48, -33))
                    .addTemporalMarker(9.5, () -> {
                        rotateServo.setPosition(.22);
                    })
                    .addTemporalMarker(11, () -> {
                        clawServo1.setPosition(.3);
                    })
                    .strafeTo(new Vector2d(45, -36))
                    .strafeTo(new Vector2d(47, -60))

                    .build();
            drive.followTrajectorySequenceAsync(traj);
            while (opModeIsActive()) {
                drive.update();
            }
        }
        else if (placementPosition == PlacementPosition.RIGHT) {


            drive.setPoseEstimate(new Pose2d(14, -62, Math.toRadians(90)));
            TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d(14, -62, Math.toRadians(90)))

                    .strafeTo(new Vector2d(14, -42))
                    .turn(Math.toRadians(-90))
                    .addTemporalMarker(1, () -> {
                        rotateServo.setPosition(0);
                    })
                    .addTemporalMarker(3.1, () -> {
                        clawServo.setPosition(.7);
                    })
                    .strafeTo(new Vector2d(12, -42))
                    .addTemporalMarker(3.5, () -> {
                        rotateServo.setPosition(.63);
                    })
                    .strafeTo(new Vector2d(60, -40))
                    .strafeTo(new Vector2d(50.5, -42))

                    .addTemporalMarker(9.5, () -> {
                        rotateServo.setPosition(.22);
                    })
                    .addTemporalMarker(10, () -> {
                        clawServo1.setPosition(.3);
                    })
                    .strafeTo(new Vector2d(50, -60))

                    .build();
            drive.followTrajectorySequenceAsync(traj3);
            while (opModeIsActive()) {
                drive.update();


            }

        } else {

            drive.setPoseEstimate(new Pose2d(14, -62, Math.toRadians(90)));
            TrajectorySequence traj4 = drive.trajectorySequenceBuilder(new Pose2d(14, -62, Math.toRadians(90)))

                    .strafeTo(new Vector2d(12, -42))
                    .turn(Math.toRadians(90))
                    .addTemporalMarker(1, () -> {
                        rotateServo.setPosition(0);
                    })
                    .addTemporalMarker(2.5, () -> {
                        clawServo.setPosition(.7);
                    })
                    .strafeTo(new Vector2d(14, -42))
                    .addTemporalMarker(3.7, () -> {
                        rotateServo.setPosition(.63);
                    })
                    .strafeTo(new Vector2d(12,-42))
                    .turn(Math.toRadians(180))
                    .strafeTo(new Vector2d(50.5, -24))

                    .addTemporalMarker(9.5, () -> {
                        rotateServo.setPosition(.22);
                    })
                    .addTemporalMarker(14, () -> {
                        clawServo1.setPosition(.3);
                    })
                    .strafeTo(new Vector2d(45, -36))
                    .strafeTo(new Vector2d(47, -60))

                    .build();
            drive.followTrajectorySequenceAsync(traj4);

            while (opModeIsActive()) {
                drive.update();

            }

        }


    }
}
