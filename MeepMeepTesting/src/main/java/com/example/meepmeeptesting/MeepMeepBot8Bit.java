package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBot8Bit {
  public static void main(String[] args) {

    // jwitt (needed for display in linux)
    System.setProperty("sun.java2d.opengl", "true");

    MeepMeep meepMeep = new MeepMeep(800);


    RoadRunnerBotEntity B8B_BZ2_PIXEL = new DefaultBotBuilder(meepMeep)
            .setColorScheme(new ColorSchemeBlueDark())
            .setConstraints(50,50,Math.toRadians(180), Math.toRadians(180), 14.1732)
            .followTrajectorySequence ( drive ->
                    drive.trajectorySequenceBuilder(new Pose2d(-36.13, 58.55, Math.toRadians(90.00)))
                            .lineToConstantHeading(new Vector2d(-36.13, 11.40))
                            .lineToSplineHeading(new Pose2d(-26.01, 9.22, Math.toRadians(180.00)))
                            .lineToConstantHeading(new Vector2d(35.23, 11.02))
                            .lineToConstantHeading(new Vector2d(34.98, 34.59))
                            .lineToConstantHeading(new Vector2d(48.81, 34.85))
                            .build()
            );


    RoadRunnerBotEntity BotBZ1_BACK = new DefaultBotBuilder(meepMeep)
    .setColorScheme(new ColorSchemeBlueDark())
    .setConstraints(45,45,Math.toRadians(120), Math.toRadians(120), 17.66)
    .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, 63.5, Math.toRadians(-90.00)))
    .splineToConstantHeading(new Vector2d(26, 40), Math.toRadians(-90))
    .lineToLinearHeading(new Pose2d(36, 60, Math.toRadians(178.80)))
            .lineToLinearHeading(new Pose2d(36, 35, Math.toRadians(180.00))) // view location
            .lineTo(new Vector2d(45.14, 42.58)) // TODO: APRIL TAG LOC
    .lineTo(new Vector2d(51.53, 61.58))
    .lineToLinearHeading(new Pose2d(62.50, 62.50, Math.toRadians(180.00)))
    .build());


    meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
    .setDarkMode(true)
    .setBackgroundAlpha(0.95f)
    .addEntity(B8B_BZ2_PIXEL)
    .start();
  }
}