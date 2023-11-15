package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBZ1 {
  public static void main(String[] args) {
    MeepMeep meepMeep = new MeepMeep(800);

 /*   RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17.66)
    .followTrajectorySequence(drive ->
    drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
    .forward(30)
    .turn(Math.toRadians(90))
    .forward(30)
    .turn(Math.toRadians(90))
    .forward(30)
    .turn(Math.toRadians(90))
    .forward(30)
    .turn(Math.toRadians(90))
    .build()
    );
*/
    RoadRunnerBotEntity BotBZ1_BACK = new DefaultBotBuilder(meepMeep)
    .setColorScheme(new ColorSchemeBlueDark())
    .setConstraints(52,52,Math.toRadians(180), Math.toRadians(180), 17.66)
    .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12.97, 62.31, Math.toRadians(-90.00)))
    .splineToConstantHeading(new Vector2d(23.76, 41.12), Math.toRadians(266.67))


    .lineToLinearHeading(new Pose2d(34.72, 43.68, Math.toRadians(180.00)))
    .lineTo(new Vector2d(45.14, 42.58))
    .lineTo(new Vector2d(51.53, 61.58))
    .lineToLinearHeading(new Pose2d(60.67, 61.40, Math.toRadians(180.00)))
    .build());


    RoadRunnerBotEntity BotBZ1_PIXEL = new DefaultBotBuilder(meepMeep)
    .setColorScheme(new ColorSchemeBlueDark())
    .setConstraints(52, 52, Math.toRadians(180), Math.toRadians(180), 17.66)
    .followTrajectorySequence(drive ->
    drive.trajectorySequenceBuilder(new Pose2d(-35.63, 62.50, Math.toRadians(-90.00)))
    .lineToLinearHeading(new Pose2d(-36.38, 31.80, Math.toRadians(360.00)))
    .addDisplacementMarker(() -> {
      // Deploy Purple
      /*bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
      sleep(750);
      armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
      armmotor.setPower(1);
      sleep(500);
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
      setWristFoldPosition();*/
    })
    .lineToConstantHeading(new Vector2d(-52.08, 32.16))
    .lineToSplineHeading(new Pose2d(-60.67, 61.58, Math.toRadians(180.00)))
    .lineTo(new Vector2d(35.63, 62.13))
    .lineTo(new Vector2d(33.62, 41.85))
    .addDisplacementMarker(()->{
      // REVERSE CAMERA ACQUIRE APRIL TAGS
      /*camServo.setPosition(BotConstants.CAM_SERVO_REAR);

      double[] xyArray = acquireTagLocation();
      telemetry.addLine(String.format("(%.3f,%.3f",xyArray[0],xyArray[1]));
      telemetry.update();
*/
    })
    .waitSeconds(1)
    .lineTo(new Vector2d(42.21, 41.48))
    .lineTo(new Vector2d(49.0, 8.59))
    .build());

/*    Bot3
      toPurplePixel = drive.trajectoryBuilder(new Pose2d())
      .lineToSplineHeading(new Pose2d(34, 0, Math.toRadians(90)))
      .build();

    drive.followTrajectory(toPurplePixel);

    // Open Bottom Finger to deposit Purple Pixel on Strike mark
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    sleep(1000);
    armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
    armmotor.setPower(1);
    sleep(1000);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    setWristFoldPosition();


    // Navigate to Backdrop (Yellow Pixel)
    Trajectory backToStart = drive.trajectoryBuilder(toPurplePixel.end())
    .lineToSplineHeading(new Pose2d(12, 0, drive.getRawExternalHeading()))
    .lineToSplineHeading(new Pose2d(5, 0, Math.toRadians(-90)))
    .build();

    // Execute Drive trajectory
    drive.followTrajectory(backToStart);

    Trajectory ThroughTruss = drive.trajectoryBuilder(backToStart.end())
    .lineToSplineHeading(new Pose2d(5, 55, drive.getRawExternalHeading())) // was 72
    .splineToConstantHeading(new Vector2d(44, 48), Math.toRadians(0)) // was 75 was 35
    .build();

    drive.followTrajectory(ThroughTruss);


    // Acquire April Tag by zone (Red: 4,5,6  blue: 1,2,3)
    // TODO: Hard Coding BLUE team for now  Need to SET this value (Also ID Offset is hard-coded in botconstants... FIX before 18NOV)
    camServo.setPosition(BotConstants.CAM_SERVO_REAR);
    Vector2d driveToTag = new Vector2d(0,0);
    driveToTag = acquireTagLocation(new Vector2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY()), 0);
    while (!gamepad1.a)
    {
      telemetry.addLine(String.format("AprilTag: (%.3f,%.3f), tagfound: %b, id: %d",driveToTag.getX(),driveToTag.getY(),tagfound,tagIdfound));
      telemetry.update();
    }

    Trajectory toAprilTag = drive.trajectoryBuilder(ThroughTruss.end())
    .splineTo(driveToTag, drive.getExternalHeading(), // -9.5 distance from camera Y to AprilTag Y
    SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
    )
    .build();

    drive.followTrajectory(toAprilTag);
*/
    meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
    .setDarkMode(true)
    .setBackgroundAlpha(0.95f)
    .addEntity(BotBZ1_PIXEL)
    .addEntity(BotBZ1_BACK)
    .start();
  }
}