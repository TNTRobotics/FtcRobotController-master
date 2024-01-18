package com.example.meepmeeptesting;

import static javax.swing.text.StyleConstants.setBackground;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity BlueCloseBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(20, 20, Math.toRadians(180), Math.toRadians(180), 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14, 62, Math.toRadians(270)))
                                .strafeTo(new Vector2d(14, 36))
                                .strafeTo(new Vector2d(14, 40))
                                .turn(Math.toRadians(90))
                                .strafeTo(new Vector2d(44, 36))
                                .strafeTo(new Vector2d(47, 60))
                                .build()
                );
        RoadRunnerBotEntity BlueFarBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(20, 20, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 62, Math.toRadians(270)))
                                .strafeTo(new Vector2d(-36, 36))
                                .strafeTo(new Vector2d(-36, 40))
                                .turn(Math.toRadians(90))
                                .strafeTo(new Vector2d(-36, 36))
                                .strafeTo(new Vector2d(44, 36))
                                .strafeTo(new Vector2d(47, 13))
                                .build()
                );
        RoadRunnerBotEntity RedFarBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(20, 20, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(90)))
                                .strafeTo(new Vector2d(-36, -36))
                                .strafeTo(new Vector2d(-36, -40))
                                .turn(Math.toRadians(-90))
                                .strafeTo(new Vector2d(-36, -36))
                                .strafeTo(new Vector2d(44, -36))
                                .strafeTo(new Vector2d(47, -13))
                                .build()
                );
        RoadRunnerBotEntity RedCloseBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(20, 20, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14, -62, Math.toRadians(90)))
                                .strafeTo(new Vector2d(14, -36))
                                .strafeTo(new Vector2d(14, -40))
                                .turn(Math.toRadians(-90))
                                .strafeTo(new Vector2d(44, -36))
                                .strafeTo(new Vector2d(47, -60))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(BlueCloseBot)
                .addEntity(BlueFarBot)
                .addEntity(RedCloseBot)
                .addEntity(RedFarBot)
                .start();
    }
}