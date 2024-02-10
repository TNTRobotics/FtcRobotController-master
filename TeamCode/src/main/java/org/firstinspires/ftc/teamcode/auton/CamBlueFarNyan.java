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
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipelineBlueFar;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.atomic.AtomicReference;

/*
 * This is an example of a more complex path to really test the tuning.
 */
//just showing stuff
@Autonomous(group = "drive \uD83D\uDFE6")
public class CamBlueFarNyan extends LinearOpMode {

    PropDetectionPipelineBlueFar propDetectionRed;
    String webcamName = "Webcam 1";

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    private VisionPortal visionPortal2;
    private PropDetectionPipelineBlueFar propDetector;


    PID slidesPID = new PID(.005,.0,.02,.008);

    PID pivotPID = new PID(.002, .0, .02, .008);

    @Override


    public void runOpMode() throws InterruptedException {
        Config cfg = new Config();

        propDetector = new PropDetectionPipelineBlueFar();
        /** Create a new VisionPortal instance. */
        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(propDetector)
                .enableLiveView(true)
                .build();
        /** Servos and motors do exist. */
        Servo clawServo = (Servo) hardwareMap.get(Servo.class, "clawServo");
        Servo clawServo1 = (Servo) hardwareMap.get(Servo.class, "clawServo1");
        Servo rotateServo = (Servo) hardwareMap.get(Servo.class, "rotateServo");

        DcMotor pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        DcMotor pivot2Motor = hardwareMap.get(DcMotor.class, "pivot2Motor");
        DcMotor slide1Motor = hardwareMap.get(DcMotor.class, "s1");

        /** What is what? */
        cfg.slide1Motor = slide1Motor;
        cfg.pivot2Motor = pivot2Motor;
        cfg.pivotMotor = pivotMotor;
        cfg.rotateServo = rotateServo;
        /** Init telemetry and claws to close. */
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
        /** Call the mech drive for rr. */
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        /** Call the drive clarity handler. */
        Drive1ClarityHandler drive1ClarityHandler = new Drive1ClarityHandler();
        /** Config slide and pivot pos. */
        AtomicReference<Drive1ClarityHandler.LIFT_POSITIONS> liftPosition = new AtomicReference<>(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_0);

        // AtomicReference<Drive1ClarityHandler.LIFT_POSITIONS> liftPosition = new AtomicReference<>(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_0);
        /** Get the placement position. */
        PlacementPosition placementPosition = propDetector.getPlacementPosition();
        /** Traj sequences. */
        TrajectorySequence traj;
        TrajectorySequence traj2;
        TrajectorySequence traj3;
        /** Center placement pos. */
        if (placementPosition == PlacementPosition.CENTER) {
            drive.setPoseEstimate(new Pose2d(-14, 62, Math.toRadians(-90)));
            traj = drive.trajectorySequenceBuilder(new Pose2d(-14, 62, Math.toRadians(-90)))
                    /**move forward to drop pixel on spike mark
                     **/
                    .strafeTo(new Vector2d(-14, 40))

                    /**start claw movement down **/
                    .addTemporalMarker(0, () -> {
                        rotateServo.setPosition(1);
                    })

                    /**open claw**/
                    .addTemporalMarker(1.5, () -> {
                        clawServo.setPosition(.7);
                    })
                    /**turn to face the stack**/
                    .turn(Math.toRadians(-90))
                    /**move to stack**/
                    .strafeTo(new Vector2d(-32, 36))
                    /**start moving pivot to set pos for stack pickup**/
                    .addTemporalMarker(1.5, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_4);
                    })
                    /**close claw to pick up 1 pixel**/
                    .addTemporalMarker(4, () -> {
                        clawServo.setPosition(1);
                    })
                    /**pivot down to move to backdrop**/
                    .addTemporalMarker(3.6, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_0);
                    })
                    /**move to backdrop, not too close. saftey first**/
                    .strafeTo(new Vector2d(-34, 59))
                    /**move claw back up**/
                    .addTemporalMarker(4.5, () -> {
                        rotateServo.setPosition(0);
                    })
                    /**start moving pivot to set pos**/
                    .addTemporalMarker(12, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_5);
                    })
                    .addTemporalMarker(14.7, () ->{
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_2);
                    })
                    /**move to backdrop**/
                    .strafeTo(new Vector2d(65, 59))
                    /**turn to go backwards towards center field**/
                    .turn(Math.toRadians(90))
                    /**move to front of the backdrop **/
                    .strafeTo(new Vector2d(65, 42))
                    /**turn for pivot to face backdrop**/
                    .turn(Math.toRadians(-88))
                    /**start moving pivot to set pos for higher linear slide location**/
/*                    .addTemporalMarker(14, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_5);
                    })
                    /**open claw to drop pixels**/
                    .addTemporalMarker(14.4, () -> {
                        clawServo.setPosition(.85);
                        clawServo1.setPosition(.3);

                    })
                    /**go to the backdrop**/
                    .strafeTo(new Vector2d(74, 36))
                    .waitSeconds(1)
                    /**move away slightly from backdrop**/
                    .strafeTo(new Vector2d(70, 32))
                    /**start putting pivot down**/
                    .addTemporalMarker(17 , () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_0);
                    })
                    /**go to the corner**/
                    .strafeTo(new Vector2d(70, 55))
                    .strafeTo(new Vector2d(85, 55))


                    /**build trajectory**/
                    .build();
            drive.followTrajectorySequenceAsync(traj);
        }
        /** left box placement pos. */
        else if (placementPosition == PlacementPosition.LEFT) {
            drive.setPoseEstimate(new Pose2d(-14, 62, Math.toRadians(-90)));
            traj2 = drive.trajectorySequenceBuilder(new Pose2d(-14, 62, Math.toRadians(-90)))

                    .strafeTo(new Vector2d(-14, 40))

                    /**start claw movement down **/
                    .addTemporalMarker(0, () -> {
                        rotateServo.setPosition(1);
                    })

                    /**open claw**/
                    .addTemporalMarker(1.9, () -> {
                        clawServo.setPosition(.7);
                    })
                    /**turn to face the stack**/
                    .turn(Math.toRadians(90))
                    .strafeTo(new Vector2d(-18,40))
                    .turn(Math.toRadians(180))
                    /**move to stack**/
                    .strafeTo(new Vector2d(-31.5, 37))
                    /**start moving pivot to set pos for stack pickup**/
                    .addTemporalMarker(2, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_4);
                    })
                    /**close claw to pick up 1 pixel**/
                    .addTemporalMarker(6.4, () -> {
                        clawServo.setPosition(1);
                    })
                    /**pivot down to move to backdrop**/
                    .addTemporalMarker(6.1, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_0);
                    })
                    /**move to backdrop, not too close. saftey first**/
                    .strafeTo(new Vector2d(-34, 59))
                    /**move claw back up**/
                    .addTemporalMarker(6.5, () -> {
                        rotateServo.setPosition(0);
                    })
                    /**start moving pivot to set pos**/
                    .addTemporalMarker(14, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_1);
                    })
                    .addTemporalMarker(16.8, () ->{
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_2);
                    })
                    /**move to backdrop**/
                    .strafeTo(new Vector2d(63, 59))
                    /**turn to go backwards towards center field**/
                    .turn(Math.toRadians(90))
                    /**move to front of the backdrop **/
                    .strafeTo(new Vector2d(63, 42))
                    /**turn for pivot to face backdrop**/
                    .turn(Math.toRadians(-87))
                    /**start moving pivot to set pos for higher linear slide location**/
/*                    .addTemporalMarker(14, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_5);
                    })
                    /**open claw to drop pixels**/
                    .addTemporalMarker(16.5, () -> {
                        clawServo1.setPosition(.15);

                    })
                    .addTemporalMarker(16.5, () -> {
                        clawServo.setPosition(.7);

                    })
                    /**go to the backdrop**/
                    .strafeTo(new Vector2d(74, 41))
                    /**move away slightly from backdrop**/
                    .strafeTo(new Vector2d(70, 41))
                    /**start putting pivot down**/
                    .addTemporalMarker(19, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_0);
                    })
                    /**go to the corner**/
                    .strafeTo(new Vector2d(70, 55))
                    .strafeTo(new Vector2d(85, 55))
                    .waitSeconds(2)
                    /**build trajectory**/
                    .build();




            drive.followTrajectorySequenceAsync(traj2);


        } else {

            drive.setPoseEstimate(new Pose2d(-14, 62, Math.toRadians(-90)));
            traj3 = drive.trajectorySequenceBuilder(new Pose2d(-14, 62, Math.toRadians(-90)))
                    /**move forward to drop pixel on spike mark
                     **/
                    .strafeTo(new Vector2d(-25, 62))
                    .strafeTo(new Vector2d(-25, 36))
                    .strafeTo(new Vector2d( -25, 40))

                    /**start claw movement down **/
                    .addTemporalMarker(0, () -> {
                        rotateServo.setPosition(1);
                    })

                    /**open claw**/
                    .addTemporalMarker(1.7, () -> {
                        clawServo.setPosition(.7);
                    })
                    /**turn to face the stack**/
                    .turn(Math.toRadians(-90))
                    /**move to stack**/
                    .strafeTo(new Vector2d(-32, 37))

                    /**start moving pivot to set pos for stack pickup**/
                    .addTemporalMarker(1.5, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_4);
                    })
                    .addTemporalMarker(1.8, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_0);
                    })                    .addTemporalMarker(2.3, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_4);
                    })


                    /**close claw to pick up 1 pixel**/
                    .addTemporalMarker(5, () -> {
                        clawServo.setPosition(1);
                    })
                    /**pivot down to move to backdrop**/
                    .addTemporalMarker(5, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_0);
                    })
                    /**move to backdrop, not too close. saftey first**/
                    .strafeTo(new Vector2d(-34, 59.5))
                    /**move claw back up**/
                    .addTemporalMarker(5.5, () -> {
                        rotateServo.setPosition(0);
                    })
                    /**start moving pivot to set pos**/
                    .addTemporalMarker(14, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_1);
                    })
                    .addTemporalMarker(15.9, () ->{
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_2);
                    })
                    /**move to backdrop**/
                    .strafeTo(new Vector2d(65, 59.5))
                    /**turn to go backwards towards center field**/
                    .turn(Math.toRadians(90))
                    /**move to front of the backdrop **/
                    .strafeTo(new Vector2d(65, 30))
                    /**turn for pivot to face backdrop**/
                    .turn(Math.toRadians(-88))
                    /**start moving pivot to set pos for higher linear slide location**/
/*                    .addTemporalMarker(14, () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_5);
                    })
                    /**open claw to drop pixels**/
                    .addTemporalMarker(15.8, () -> {
                        clawServo1.setPosition(.15);

                    })
                    .addTemporalMarker(17.5, () -> {
                        clawServo.setPosition(.7);

                    })
                    /**go to the backdrop**/
                    .strafeTo(new Vector2d(74, 30))
                    .strafeTo(new Vector2d(74, 36))
                    .waitSeconds(1)
                    /**move away slightly from backdrop**/
                    .strafeTo(new Vector2d(68, 26))
                    /**start putting pivot down**/
                    .addTemporalMarker(17.9 , () -> {
                        liftPosition.set(Drive1ClarityHandler.LIFT_POSITIONS.LEVEL_0);
                    })
                    /**go to the corner**/
                    .strafeTo(new Vector2d(70, 55))
                    .strafeTo(new Vector2d(85, 55))
                    /**build trajectory**/
                    .waitSeconds(2)
                    .build();

            drive.followTrajectorySequenceAsync(traj3);

        }

        while (opModeIsActive()) {
            drive.update();

            telemetry.addData("Lift target position: ", liftPosition.get());

            drive1ClarityHandler.updateSlideMotorsAuto(slidesPID, pivotPID, cfg , liftPosition.get(), telemetry);
            telemetry.update();

        }


    }
}
