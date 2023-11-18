package org.firstinspires.ftc.teamcode.autonomous;

import android.annotation.SuppressLint;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.common.BotConstants;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "Competition", name = "Autonomous")

public class AutonomousSoftwareOpMode extends LinearOpMode {

  public int team     = BotConstants.RED_TEAM;
  public int startLoc = BotConstants.START_SIDE_PIXEL;

  private Blinker control_Hub;
  private Servo bottomArmServo;
  private IMU imu;
  private Servo topArmServo;
  private Servo wristPanServo;
  private Servo camServo;
  private Servo droneServo = null;
  private Servo hookServo = null;
  private final ElapsedTime runtime = new ElapsedTime();
  private DcMotor motor1 = null; //front left
  private DcMotor motor2 = null; //front right
  private DcMotor motor3 = null; //back left
  private DcMotor motor4 = null; //back right
  private DcMotor armmotor = null;
  int zone;
  int tagIdFound =0;
  boolean tagFound =false;
  Vector2d driveToTag = new Vector2d();
  String teamStr;
  String startLocStr;

  // ALL CONSTANTS MOVED TO Common.BotConstants class

  // Vision portal Replaces EasyOpenCV method
  private VisionPortal visionPortal;
  Contours_Extraction contoursExtraction = new Contours_Extraction();

  // JW AprilTag
  private AprilTagProcessor aprilTag;

  private static final boolean USE_WEBCAM = true;

  @Override
  public void runOpMode() {
    // Export telemetry to FTC Dashboard
    Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    telemetry.addData("Status", "Initializing");
    telemetry.update();

    initHardware();

    if (team ==BotConstants.RED_TEAM)
      teamStr = "*RED*";
    else
      teamStr = "*BLUE*";
    if (startLoc == BotConstants.START_SIDE_PIXEL)
      startLocStr = "*PIXEL*";
    else
      startLocStr = "*BACKDROP*";

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    while (!isStarted()){
      telemetry.addLine(String.format("Team: %s  Side: %s",teamStr,startLocStr));
      zone = detectZone();
    }
    //waitForStart();

    if (team == BotConstants.BLUE_TEAM) {
      if (startLoc == BotConstants.START_SIDE_PIXEL) {
        switch (zone) {
          case 1:
            RR_BZ1_Pixel();
            break;
          case 2:
            RR_BZ2_Pixel();
            break;
          case 3:
            RR_BZ3_Pixel();
            break;
        } // end switch
      } // end Pixel
      else {  // Backdrop Side
        switch (zone) {
          case 1:
            RR_BZ1_Backdrop();
            break;
          case 2:
            RR_BZ2_Backdrop();
            break;
          case 3:
            RR_BZ3_Backdrop();
            break;
        } // end switch
      } // End Backdrop

    } // End Blue
    else { // RED TEAM
      if (startLoc == BotConstants.START_SIDE_PIXEL) {
        switch (zone) {
          case 1:
            RR_RZ1_Pixel();
            break;
          case 2:
            RR_RZ2_Pixel();
            break;
          case 3:
            RR_RZ3_Pixel();
            break;
        } // end switch
      } // end Pixel
      else {  // Backdrop Side
        switch (zone) {
          case 1:
            RR_RZ1_Backdrop();
            break;
          case 2:
            RR_RZ2_Backdrop();
            break;
          case 3:
            RR_RZ3_Backdrop();
            break;
        } // end switch
      } // End Backdrop
    } // End Red


    while (opModeIsActive())
    {
      // do nothing after Automation until end
    }
  }


  @SuppressLint("DefaultLocale")
  public void RR_BZ1_Pixel()
  {
    double[] xyArray = new double[2];

    // Lower Wrist
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FLOOR);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    visionPortal.setProcessorEnabled(contoursExtraction,false);

    // BZ1_PIXEL
    TrajectorySequence BZ1_PixelSide =
    drive.trajectorySequenceBuilder(new Pose2d(-35.63, 62.50, Math.toRadians(-90.00)))
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))


    //.lineToLinearHeading(new Pose2d(-36.38, 31.80, Math.toRadians(360.00)))
    .splineToLinearHeading(new Pose2d(-34.5, 31.80, Math.toRadians(0.00)), Math.toRadians(0.00)) // tweak 17nov 5pm was x-32.5

    .addDisplacementMarker(() -> {

      // Deploy Purple on strike 1
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
      sleep(750);
      armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
      armmotor.setPower(1);
      sleep(500);
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
      setWristFoldPosition();
    })
    .lineToConstantHeading(new Vector2d(-52.08, 32.16))
    .lineToSplineHeading(new Pose2d(-60.67, 59, Math.toRadians(180.00))) // tweak 17Nov23 truss
    .lineTo(new Vector2d(35.63, 59)) // tweak 17nov23
    .lineTo(new Vector2d(32, 36)) // View Location New Point for April Tag. (tweak 17nov23 was x:30 encroach)
    .build();
    drive.setPoseEstimate(BZ1_PixelSide.start());
    drive.followTrajectorySequence(BZ1_PixelSide);

    // TODO: Possibly move cam swivel earlier
    // TODO: try different focus modes for better picture?
    camServo.setPosition(BotConstants.CAM_SERVO_REAR);
    sleep(2000);
    xyArray[0] = drive.getPoseEstimate().getX();
    xyArray[1] = drive.getPoseEstimate().getY();

    driveToTag = acquireTagLocation(new Vector2d(xyArray[0],xyArray[1]), team);
    telemetry.addLine(String.format("AprilTag: (%.3f,%.3f), tagFound: %b, id: %d",driveToTag.getX(),driveToTag.getY(), tagFound, tagIdFound));
    telemetry.addLine(String.format("PoseEstimate: (%.3f,%.3f)", xyArray[0], xyArray[1]));
    telemetry.update();

    TrajectorySequence BZ1_PixelSideB =
    drive.trajectorySequenceBuilder(BZ1_PixelSide.end())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))
    .lineTo(driveToTag) // Go to April Tag
    .build();
    drive.setPoseEstimate(BZ1_PixelSideB.start());
    drive.followTrajectorySequence(BZ1_PixelSideB);

    // Drop Yellow Pixel
    setArmPosition(BotConstants.ARM_POS_AUTO_DEPLOY, BotConstants.ARM_POWER);
    setWristDeployPosition();
    // Put a blocking call after Arm and Wrist, will allow both to move at same time.
    while (armmotor.isBusy()) {
      sleep(10);
    }
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);
    sleep(400);

    // Close up for parking
    setArmPosition(BotConstants.ARM_POS_DRIVE, BotConstants.ARM_POWER);
    sleep(100);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
    setWristFoldPosition();

    // PARK
    TrajectorySequence BZ1_PixelSideC =
    drive.trajectorySequenceBuilder(BZ1_PixelSideB.end())
    .lineTo(new Vector2d(49.0, 8.59))
    .build();

    drive.followTrajectorySequence(BZ1_PixelSideC);
  }

  @SuppressLint("DefaultLocale")
  public void RR_BZ2_Pixel()
  {
    double[] xyArray = new double[2];

    // Lower Wrist
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FLOOR);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    visionPortal.setProcessorEnabled(contoursExtraction,false);

    // BZ2_PIXEL
    TrajectorySequence BZ2_PixelSide =
    drive.trajectorySequenceBuilder(new Pose2d(-35.63, 62.50, Math.toRadians(-90.00)))
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))


    //.lineToLinearHeading(new Pose2d(-36.38, 31.80, Math.toRadians(360.00)))
    .lineToConstantHeading(new Vector2d(-36.00, 36)) // tweak 17n 530p was y:38
    .addDisplacementMarker(() -> {
      // Deploy Purple on strike 1
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
      sleep(250);
      armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
      armmotor.setPower(1);
      sleep(250);
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
      setWristFoldPosition();
    })
    .lineToConstantHeading(new Vector2d(-56, 38)) // tweaked // -54 32
    .lineToSplineHeading(new Pose2d(-60.67, 61.58, Math.toRadians(180.00)))
    .lineTo(new Vector2d(33.25, 59))
    .lineTo(new Vector2d(30, 31)) // View Location New Point for April Tag. // tweak 17n 5:22p was y:33 y:36
    .build();
    drive.setPoseEstimate(BZ2_PixelSide.start());
    drive.followTrajectorySequence(BZ2_PixelSide);

    camServo.setPosition(BotConstants.CAM_SERVO_REAR);
    sleep(2000);
    xyArray[0] = drive.getPoseEstimate().getX();
    xyArray[1] = drive.getPoseEstimate().getY();

    driveToTag = acquireTagLocation(new Vector2d(xyArray[0],xyArray[1]), team);
    telemetry.addLine(String.format("AprilTag: (%.3f,%.3f), tagFound: %b, id: %d",driveToTag.getX(),driveToTag.getY(), tagFound, tagIdFound));
    telemetry.addLine(String.format("PoseEstimate: (%.3f,%.3f)", xyArray[0], xyArray[1]));
    telemetry.update();

    TrajectorySequence BZ2_PixelSideB =
    drive.trajectorySequenceBuilder(BZ2_PixelSide.end())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))
    .lineTo(driveToTag) // Go to April Tag
    .build();
    drive.setPoseEstimate(BZ2_PixelSideB.start());
    drive.followTrajectorySequence(BZ2_PixelSideB);

    // Drop Yellow Pixel
    setArmPosition(BotConstants.ARM_POS_AUTO_DEPLOY, BotConstants.ARM_POWER);
    setWristDeployPosition();
    // Put a blocking call after Arm and Wrist, will allow both to move at same time.
    while (armmotor.isBusy()) {
      sleep(10);
    }
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);
    sleep(400);

    // Close up for parking
    setArmPosition(BotConstants.ARM_POS_DRIVE, BotConstants.ARM_POWER);
    sleep(100);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
    setWristFoldPosition();

    // PARK
    TrajectorySequence BZ2_PixelSideC =
    drive.trajectorySequenceBuilder(BZ2_PixelSideB.end())
    .lineTo(new Vector2d(44, 0))
    .build();

    drive.followTrajectorySequence(BZ2_PixelSideC);
  }

  @SuppressLint("DefaultLocale")
  public void RR_BZ3_Pixel()
  {
    double[] xyArray = new double[2];

    // Lower Wrist
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FLOOR);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    visionPortal.setProcessorEnabled(contoursExtraction,false);

    // BZ3_PIXEL
    TrajectorySequence BZ3_PixelSide =
    drive.trajectorySequenceBuilder(new Pose2d(-35.63, 62.50, Math.toRadians(-90.00)))
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))

    // .lineToConstantHeading(new Pose2d(-36.38, 31.80, Math.toRadians(-180)))
    .lineToConstantHeading(new Vector2d(-50, 42))
    .addDisplacementMarker(() -> {

      // Deploy Purple on strike 1
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
      sleep(750);
      armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
      armmotor.setPower(1);
      sleep(500);
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
      setWristFoldPosition();
    })
    //.lineToConstantHeading(new Vector2d(-52.08, 32.16))
    .lineToConstantHeading(new Vector2d(-59, 40))
    .lineToSplineHeading(new Pose2d(-60.67, 60, Math.toRadians(180.00)))
    .lineToConstantHeading(new Vector2d(35.63, 58))
    .lineToConstantHeading(new Vector2d(30, 28)) // View Location New Point for April Tag.
    .build();
    drive.setPoseEstimate(BZ3_PixelSide.start());
    drive.followTrajectorySequence(BZ3_PixelSide);

    camServo.setPosition(BotConstants.CAM_SERVO_REAR);
    sleep(2000);
    xyArray[0] = drive.getPoseEstimate().getX();
    xyArray[1] = drive.getPoseEstimate().getY();

    driveToTag = acquireTagLocation(new Vector2d(xyArray[0],xyArray[1]), team);
    telemetry.addLine(String.format("AprilTag: (%.3f,%.3f), tagFound: %b, id: %d",driveToTag.getX(),driveToTag.getY(), tagFound, tagIdFound));
    telemetry.addLine(String.format("PoseEstimate: (%.3f,%.3f)", xyArray[0], xyArray[1]));
    telemetry.update();

    TrajectorySequence BZ3_PixelSideB =
    drive.trajectorySequenceBuilder(BZ3_PixelSide.end())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))
    .lineToConstantHeading(driveToTag) // Go to April Tag
    .build();
    drive.setPoseEstimate(BZ3_PixelSideB.start());
    drive.followTrajectorySequence(BZ3_PixelSideB);

    // Drop Yellow Pixel
    setArmPosition(BotConstants.ARM_POS_AUTO_DEPLOY, BotConstants.ARM_POWER);
    setWristDeployPosition();
    // Put a blocking call after Arm and Wrist, will allow both to move at same time.
    while (armmotor.isBusy()) {
      sleep(10);
    }
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);
    sleep(400);

    // Close up for parking
    setArmPosition(BotConstants.ARM_POS_DRIVE, BotConstants.ARM_POWER);
    sleep(100);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
    setWristFoldPosition();

    // PARK
    TrajectorySequence BZ3_PixelSideC =
    drive.trajectorySequenceBuilder(BZ3_PixelSideB.end())
    .lineToConstantHeading(new Vector2d(48.0, 0)) // tweak
    .build();

    drive.followTrajectorySequence(BZ3_PixelSideC);
  }





  public void RR_BZ1_Backdrop()
  {
    double[] xyArray = new double[2];

    // Lower Wrist
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FLOOR);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    visionPortal.setProcessorEnabled(contoursExtraction,false);

    // BZ1_BackdropSide (14Nov)
    TrajectorySequence BZ1_BackdropSide =
    drive.trajectorySequenceBuilder(new Pose2d(12, 62.50, Math.toRadians(-90.00)))
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))

    .splineToConstantHeading(new Vector2d(26, 40), Math.toRadians(-90)) // Drop Pixel P
    .addDisplacementMarker(() -> {

      // Deploy Purple on strike 1
      // contoursExtraction: REPLACE SLEEP with something else
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
      sleep(250);
      armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
      armmotor.setPower(1);
      sleep(250);
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
      setWristFoldPosition();
    })

    .lineToLinearHeading(new Pose2d(36, 60, Math.toRadians(178.70)))
    .lineToLinearHeading(new Pose2d(36, 35, Math.toRadians(180.00))) // view location
    .build();
    drive.setPoseEstimate(BZ1_BackdropSide.start());
    drive.followTrajectorySequence(BZ1_BackdropSide);

    camServo.setPosition(BotConstants.CAM_SERVO_REAR);
    sleep(2000);
    xyArray[0] = drive.getPoseEstimate().getX();
    xyArray[1] = drive.getPoseEstimate().getY();

    driveToTag = acquireTagLocation(new Vector2d(xyArray[0],xyArray[1]), team);
    telemetry.addLine(String.format("AprilTag: (%.3f,%.3f), tagFound: %b, id: %d",driveToTag.getX(),driveToTag.getY(), tagFound, tagIdFound));
    telemetry.addLine(String.format("PoseEstimate: (%.3f,%.3f)", xyArray[0], xyArray[1]));
    telemetry.update();

    TrajectorySequence BZ1_BackdropSideB =
    drive.trajectorySequenceBuilder(BZ1_BackdropSide.end())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))

    .lineTo(driveToTag) // Go to April Tag location
    .build();
    drive.followTrajectorySequence(BZ1_BackdropSideB);

    // Drop Yellow Pixel
    setArmPosition(BotConstants.ARM_POS_AUTO_DEPLOY, BotConstants.ARM_POWER);
    setWristDeployPosition();
    // Put a blocking call after Arm and Wrist, will allow both to move at same time.
    while (armmotor.isBusy()) {
      sleep(10);
    }
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);
    sleep(400);

    // Close up for parking
    setArmPosition(BotConstants.ARM_POS_DRIVE, BotConstants.ARM_POWER);
    sleep(100);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
    setWristFoldPosition();

    // PARK
    TrajectorySequence BZ1_BackdropSideC =
    drive.trajectorySequenceBuilder(BZ1_BackdropSideB.start())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))

    .lineTo(new Vector2d(51.53, 61.58))
    .lineToLinearHeading(new Pose2d(62.50, 62.50, Math.toRadians(180.00)))
    .build();
    drive.followTrajectorySequence(BZ1_BackdropSideC);
  }

  public void RR_BZ2_Backdrop()
  {

    double[] xyArray = new double[2];

    // Lower Wrist
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FLOOR);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    visionPortal.setProcessorEnabled(contoursExtraction,false);

    // BZ2_BACKDROP
    TrajectorySequence BZ2_BackdropSide = drive.trajectorySequenceBuilder(new Pose2d(12, 62.50, Math.toRadians(-90.00)))
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
                    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))


    .lineToConstantHeading(new Vector2d(12.61, 35)) // tweak 17n 6:20 was 36.55
    .addDisplacementMarker(() -> {

      // Deploy Purple on strike 1
      // TODO: REPLACE SLEEP with something else
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
      sleep(250);
      armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
      armmotor.setPower(1);
      sleep(250);
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
      setWristFoldPosition();
    })
    .lineToLinearHeading(new Pose2d(40, 35, Math.toRadians(180.00))) // view location  // Tweak was 36
    .build();

    drive.setPoseEstimate(BZ2_BackdropSide.start());
    drive.followTrajectorySequence(BZ2_BackdropSide);

    camServo.setPosition(BotConstants.CAM_SERVO_REAR);
    sleep(2000);
    xyArray[0] = drive.getPoseEstimate().getX();
    xyArray[1] = drive.getPoseEstimate().getY();

    driveToTag = acquireTagLocation(new Vector2d(xyArray[0],xyArray[1]), team);
    telemetry.addLine(String.format("AprilTag: (%.3f,%.3f), tagFound: %b, id: %d",driveToTag.getX(),driveToTag.getY(), tagFound, tagIdFound));
    telemetry.addLine(String.format("PoseEstimate: (%.3f,%.3f)", xyArray[0], xyArray[1]));
    telemetry.update();

    TrajectorySequence BZ2_BackdropSideB =
    drive.trajectorySequenceBuilder(BZ2_BackdropSide.end())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))

    .lineToConstantHeading(driveToTag) // Go to April Tag location
    .build();
    drive.followTrajectorySequence(BZ2_BackdropSideB);

    // Drop Yellow Pixel
    setArmPosition(BotConstants.ARM_POS_AUTO_DEPLOY, BotConstants.ARM_POWER);
    setWristDeployPosition();
    // Put a blocking call after Arm and Wrist, will allow both to move at same time.
    while (armmotor.isBusy()) {
      sleep(10);
    }
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);
    sleep(400);

    // Close up for parking
    setArmPosition(BotConstants.ARM_POS_DRIVE, BotConstants.ARM_POWER);
    sleep(100);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
    setWristFoldPosition();

    // PARK
    TrajectorySequence BZ2_BackdropSideC =
    drive.trajectorySequenceBuilder(BZ2_BackdropSideB.start())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))

    .lineToConstantHeading(new Vector2d(51, 63)) // tweak
    .lineToLinearHeading(new Pose2d(64, 63, Math.toRadians(180.00))) // tweak
    .build();
    drive.followTrajectorySequence(BZ2_BackdropSideC);
  }

  public void RR_BZ3_Backdrop()
  {

    double[] xyArray = new double[2];

    // Lower Wrist
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FLOOR);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    visionPortal.setProcessorEnabled(contoursExtraction,false);

    // BZ3_BACKDROP
    TrajectorySequence BZ3_BackdropSide = drive.trajectorySequenceBuilder(new Pose2d(12, 62.50, Math.toRadians(-90.00)))
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))


    //.lineToConstantHeading(new Vector2d(12.61, 36.55))
    //.lineToLinearHeading(new Pose2d(12.97, 32, Math.toRadians(180.00))) // 34
    .splineToLinearHeading(new Pose2d(10.5, 32, Math.toRadians(180.00)), Math.toRadians(180.00))
    .addDisplacementMarker(() -> {
      // Deploy Purple on strike 1
      // TODO: REPLACE SLEEP with something else
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
      sleep(250);
      armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
      armmotor.setPower(1);
      sleep(250);
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
      setWristFoldPosition();
    })
    .lineToConstantHeading(new Vector2d(40, 35)) // view location  // Tweak was 36
    .build();

    drive.setPoseEstimate(BZ3_BackdropSide.start());
    drive.followTrajectorySequence(BZ3_BackdropSide);

    camServo.setPosition(BotConstants.CAM_SERVO_REAR);
    sleep(2000);
    xyArray[0] = drive.getPoseEstimate().getX();
    xyArray[1] = drive.getPoseEstimate().getY();

    driveToTag = acquireTagLocation(new Vector2d(xyArray[0],xyArray[1]), team);
    telemetry.addLine(String.format("AprilTag: (%.3f,%.3f), tagFound: %b, id: %d",driveToTag.getX(),driveToTag.getY(), tagFound, tagIdFound));
    telemetry.addLine(String.format("PoseEstimate: (%.3f,%.3f)", xyArray[0], xyArray[1]));
    telemetry.update();

    TrajectorySequence BZ3_BackdropSideB =
    drive.trajectorySequenceBuilder(BZ3_BackdropSide.end())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))

    .lineToConstantHeading(driveToTag) // Go to April Tag location
    .build();
    drive.followTrajectorySequence(BZ3_BackdropSideB);

    // Drop Yellow Pixel
    setArmPosition(BotConstants.ARM_POS_AUTO_DEPLOY, BotConstants.ARM_POWER);
    setWristDeployPosition();
    // Put a blocking call after Arm and Wrist, will allow both to move at same time.
    while (armmotor.isBusy()) {
      sleep(10);
    }
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);
    sleep(400);

    // Close up for parking
    setArmPosition(BotConstants.ARM_POS_DRIVE, BotConstants.ARM_POWER);
    sleep(100);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
    setWristFoldPosition();

    // PARK
    TrajectorySequence BZ3_BackdropSideC =
    drive.trajectorySequenceBuilder(BZ3_BackdropSideB.start())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))

    .lineToConstantHeading(new Vector2d(51, 65)) // tweak
    .lineToLinearHeading(new Pose2d(64, 65, Math.toRadians(180.00))) // tweak
    .build();
    drive.followTrajectorySequence(BZ3_BackdropSideC);
  }


  // OK 17Nov23 3:48p
  public void RR_RZ3_Pixel()
  {
    double[] xyArray = new double[2];

    // Lower Wrist
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FLOOR);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    visionPortal.setProcessorEnabled(contoursExtraction,false);

    // RZ3_PIXEL
    TrajectorySequence RZ3_PixelSide =
    drive.trajectorySequenceBuilder(new Pose2d(-35.63, -62.50, Math.toRadians(90.00)))
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))
    .lineToLinearHeading(new Pose2d(-36.38, -31.80, Math.toRadians(0)))
    .addDisplacementMarker(() -> {

      // Deploy Purple on strike 1
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
      sleep(750);
      armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
      armmotor.setPower(1);
      sleep(500);
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
      setWristFoldPosition();
    })
    .lineToConstantHeading(new Vector2d(-52.08, -32.16))
    .lineToSplineHeading(new Pose2d(-60.67, -59, Math.toRadians(-180))) // Tweak 17Nov 3pm (Truss)
    .lineToConstantHeading(new Vector2d(35.63, -59)) // tweak 17Nov 3Pm (Truss)
    .lineToConstantHeading(new Vector2d(30, -36)) // View Location New Point for April Tag Red "3"
    .build();
    drive.setPoseEstimate(RZ3_PixelSide.start());
    drive.followTrajectorySequence(RZ3_PixelSide);

    // TODO: Possibly move cam swivel earlier
    // TODO: try different focus modes for better picture?
    camServo.setPosition(BotConstants.CAM_SERVO_REAR);
    sleep(2000);
    xyArray[0] = drive.getPoseEstimate().getX();
    xyArray[1] = drive.getPoseEstimate().getY();

    driveToTag = acquireTagLocation(new Vector2d(xyArray[0],xyArray[1]), team);
    telemetry.addLine(String.format("AprilTag: (%.3f,%.3f), tagFound: %b, id: %d",driveToTag.getX(),driveToTag.getY(), tagFound, tagIdFound));
    telemetry.addLine(String.format("PoseEstimate: (%.3f,%.3f)", xyArray[0], xyArray[1]));
    telemetry.update();

    TrajectorySequence RZ3_PixelSideB =
    drive.trajectorySequenceBuilder(RZ3_PixelSide.end())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))
    .lineTo(driveToTag) // Go to April Tag
    .build();
    drive.setPoseEstimate(RZ3_PixelSideB.start());
    drive.followTrajectorySequence(RZ3_PixelSideB);

    // Drop Yellow Pixel
    setArmPosition(BotConstants.ARM_POS_AUTO_DEPLOY, BotConstants.ARM_POWER);
    setWristDeployPosition();
    // Put a blocking call after Arm and Wrist, will allow both to move at same time.
    while (armmotor.isBusy()) {
      sleep(10);
    }
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);
    sleep(400);

    // Close up for parking
    setArmPosition(BotConstants.ARM_POS_DRIVE, BotConstants.ARM_POWER);
    sleep(100);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
    setWristFoldPosition();

    // PARK
    TrajectorySequence RZ3_PixelSideC =
    drive.trajectorySequenceBuilder(RZ3_PixelSideB.end())
    .lineToConstantHeading(new Vector2d(50, 0)) // tweak 17Nov23 (little farther)
    .build();

    drive.followTrajectorySequence(RZ3_PixelSideC);
  }

  // OK (ish) 17Nov23 (Rightward bounce at tag possible) -> offset left a bit 7.5"
  @SuppressLint("DefaultLocale")
  public void RR_RZ2_Pixel()
  {
    double[] xyArray = new double[2];

    // Lower Wrist
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FLOOR);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    visionPortal.setProcessorEnabled(contoursExtraction,false);

    // RZ2_PIXEL
    TrajectorySequence RZ2_PixelSide =
    drive.trajectorySequenceBuilder(new Pose2d(-35.63, -62.50, Math.toRadians(90.00)))
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))
    .lineToConstantHeading(new Vector2d(-36.00, -36.55))
    .addDisplacementMarker(() -> {
      // Deploy Purple on strike 1
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
      sleep(250);
      armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
      armmotor.setPower(1);
      sleep(250);
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
      setWristFoldPosition();
    })
    .lineToConstantHeading(new Vector2d(-56, -38)) // tweaked // -54 32
    .lineToSplineHeading(new Pose2d(-60.67, -59, Math.toRadians(-180.00))) // pretweak  17Nov23 3:50p
    .lineTo(new Vector2d(33.25, -59))
    .lineTo(new Vector2d(30, -36)) // View Location New Point for April Tag.
    .build();
    drive.setPoseEstimate(RZ2_PixelSide.start());
    drive.followTrajectorySequence(RZ2_PixelSide);

    camServo.setPosition(BotConstants.CAM_SERVO_REAR);
    sleep(2000);
    xyArray[0] = drive.getPoseEstimate().getX();
    xyArray[1] = drive.getPoseEstimate().getY();

    driveToTag = acquireTagLocation(new Vector2d(xyArray[0],xyArray[1]), team);
    telemetry.addLine(String.format("AprilTag: (%.3f,%.3f), tagFound: %b, id: %d",driveToTag.getX(),driveToTag.getY(), tagFound, tagIdFound));
    telemetry.addLine(String.format("PoseEstimate: (%.3f,%.3f)", xyArray[0], xyArray[1]));
    telemetry.update();

    TrajectorySequence RZ2_PixelSideB =
    drive.trajectorySequenceBuilder(RZ2_PixelSide.end())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))
    .lineTo(driveToTag) // Go to April Tag
    .build();
    drive.setPoseEstimate(RZ2_PixelSideB.start());
    drive.followTrajectorySequence(RZ2_PixelSideB);

    // Drop Yellow Pixel
    setArmPosition(BotConstants.ARM_POS_AUTO_DEPLOY, BotConstants.ARM_POWER);
    setWristDeployPosition();
    // Put a blocking call after Arm and Wrist, will allow both to move at same time.
    while (armmotor.isBusy()) {
      sleep(10);
    }
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);
    sleep(400);

    // Close up for parking
    setArmPosition(BotConstants.ARM_POS_DRIVE, BotConstants.ARM_POWER);
    sleep(100);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
    setWristFoldPosition();

    // PARK
    TrajectorySequence RZ2_PixelSideC =
    drive.trajectorySequenceBuilder(RZ2_PixelSideB.end())
    .lineTo(new Vector2d(44, 0))
    .build();

    drive.followTrajectorySequence(RZ2_PixelSideC);
  }

  // OK 17Nov23 with 7.5" cam dist
  @SuppressLint("DefaultLocale")
  public void RR_RZ1_Pixel()
  {
    double[] xyArray = new double[2];

    // Lower Wrist
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FLOOR);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    visionPortal.setProcessorEnabled(contoursExtraction,false);

    // RZ1_PIXEL
    TrajectorySequence RZ1_PixelSide =
    drive.trajectorySequenceBuilder(new Pose2d(-35.63, -62.50, Math.toRadians(90.00)))
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))

    // .lineToConstantHeading(new Pose2d(-36.38, 31.80, Math.toRadians(-180)))
    .lineToConstantHeading(new Vector2d(-50, -42))
    .addDisplacementMarker(() -> {

      // Deploy Purple on strike 1
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
      sleep(250);
      armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
      armmotor.setPower(1);
      sleep(250);
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
      setWristFoldPosition();
    })
    //.lineToConstantHeading(new Vector2d(-52.08, 32.16))
    .lineToConstantHeading(new Vector2d(-59, -40))
    .lineToSplineHeading(new Pose2d(-60.67, -59, Math.toRadians(-180.00))) // tweak Truss 17Nov 4pm
    .lineToConstantHeading(new Vector2d(35.63, -59)) // tweak truss
    .lineToConstantHeading(new Vector2d(30, -28)) // View Location New Point for April Tag.
    .build();
    drive.setPoseEstimate(RZ1_PixelSide.start());
    drive.followTrajectorySequence(RZ1_PixelSide);

    camServo.setPosition(BotConstants.CAM_SERVO_REAR);
    sleep(2000);
    xyArray[0] = drive.getPoseEstimate().getX();
    xyArray[1] = drive.getPoseEstimate().getY();

    driveToTag = acquireTagLocation(new Vector2d(xyArray[0],xyArray[1]), team);
    telemetry.addLine(String.format("AprilTag: (%.3f,%.3f), tagFound: %b, id: %d",driveToTag.getX(),driveToTag.getY(), tagFound, tagIdFound));
    telemetry.addLine(String.format("PoseEstimate: (%.3f,%.3f)", xyArray[0], xyArray[1]));
    telemetry.update();

    TrajectorySequence RZ1_PixelSideB =
    drive.trajectorySequenceBuilder(RZ1_PixelSide.end())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))
    .lineToConstantHeading(driveToTag) // Go to April Tag
    .build();
    drive.setPoseEstimate(RZ1_PixelSideB.start());
    drive.followTrajectorySequence(RZ1_PixelSideB);

    // Drop Yellow Pixel
    setArmPosition(BotConstants.ARM_POS_AUTO_DEPLOY, BotConstants.ARM_POWER);
    setWristDeployPosition();
    // Put a blocking call after Arm and Wrist, will allow both to move at same time.
    while (armmotor.isBusy()) {
      sleep(10);
    }
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);
    sleep(400);

    // Close up for parking
    setArmPosition(BotConstants.ARM_POS_DRIVE, BotConstants.ARM_POWER);
    sleep(100);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
    setWristFoldPosition();

    // PARK
    TrajectorySequence RZ1_PixelSideC =
    drive.trajectorySequenceBuilder(RZ1_PixelSideB.end())
    .lineToConstantHeading(new Vector2d(48.0, 0)) // tweak
    .build();

    drive.followTrajectorySequence(RZ1_PixelSideC);
  }

// OK 17Nov23
  public void RR_RZ3_Backdrop()
  {
    double[] xyArray = new double[2];

    // Lower Wrist
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FLOOR);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    visionPortal.setProcessorEnabled(contoursExtraction,false);

    // RZ3_BackdropSide (14Nov)
    TrajectorySequence RZ3_BackdropSide =
    drive.trajectorySequenceBuilder(new Pose2d(12, -62.50, Math.toRadians(90.00)))
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))

    .splineToConstantHeading(new Vector2d(26, -40), Math.toRadians(90)) // Drop Pixel P
    .addDisplacementMarker(() -> {

      // Deploy Purple on strike 1
      // contoursExtraction: REPLACE SLEEP with something else
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
      sleep(250);
      armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
      armmotor.setPower(1);
      sleep(250);
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
      setWristFoldPosition();
    })

    .lineToLinearHeading(new Pose2d(36, -60, Math.toRadians(-178.70)))
    .lineToLinearHeading(new Pose2d(36, -35, Math.toRadians(-180.00))) // view location
    .build();
    drive.setPoseEstimate(RZ3_BackdropSide.start());
    drive.followTrajectorySequence(RZ3_BackdropSide);

    camServo.setPosition(BotConstants.CAM_SERVO_REAR);
    sleep(2000);
    xyArray[0] = drive.getPoseEstimate().getX();
    xyArray[1] = drive.getPoseEstimate().getY();

    driveToTag = acquireTagLocation(new Vector2d(xyArray[0],xyArray[1]), team);
    telemetry.addLine(String.format("AprilTag: (%.3f,%.3f), tagFound: %b, id: %d",driveToTag.getX(),driveToTag.getY(), tagFound, tagIdFound));
    telemetry.addLine(String.format("PoseEstimate: (%.3f,%.3f)", xyArray[0], xyArray[1]));
    telemetry.update();

    TrajectorySequence RZ3_BackdropSideB =
    drive.trajectorySequenceBuilder(RZ3_BackdropSide.end())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))

    .lineTo(driveToTag) // Go to April Tag location
    .build();
    drive.followTrajectorySequence(RZ3_BackdropSideB);

    // Drop Yellow Pixel
    setArmPosition(BotConstants.ARM_POS_AUTO_DEPLOY, BotConstants.ARM_POWER);
    setWristDeployPosition();
    // Put a blocking call after Arm and Wrist, will allow both to move at same time.
    while (armmotor.isBusy()) {
      sleep(10);
    }
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);
    sleep(400);

    // Close up for parking
    setArmPosition(BotConstants.ARM_POS_DRIVE, BotConstants.ARM_POWER);
    sleep(100);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
    setWristFoldPosition();

    // PARK
    TrajectorySequence RZ3_BackdropSideC =
    drive.trajectorySequenceBuilder(RZ3_BackdropSideB.start())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))

    .lineTo(new Vector2d(51.53, -61.58))
    .lineToLinearHeading(new Pose2d(62.50, -62.50, Math.toRadians(-180.00)))
    .build();
    drive.followTrajectorySequence(RZ3_BackdropSideC);
  }


  // OK 17Nov23 (4:38p) 7.5" cam
  public void RR_RZ2_Backdrop()
  {

    double[] xyArray = new double[2];

    // Lower Wrist
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FLOOR);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    visionPortal.setProcessorEnabled(contoursExtraction,false);

    // BZ2_BACKDROP
    TrajectorySequence RZ2_BackdropSide = drive.trajectorySequenceBuilder(new Pose2d(12, -62.50, Math.toRadians(90.00)))
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))


    .lineToConstantHeading(new Vector2d(12.61, -34.55)) // tweak was -36.55
    .addDisplacementMarker(() -> {

      // Deploy Purple on strike 1
      // TODO: REPLACE SLEEP with something else
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
      sleep(250);
      armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
      armmotor.setPower(1);
      sleep(250);
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
      setWristFoldPosition();
    })
    .lineToLinearHeading(new Pose2d(40, -35, Math.toRadians(-180.00))) // view location  // Tweak was 36
    .build();

    drive.setPoseEstimate(RZ2_BackdropSide.start());
    drive.followTrajectorySequence(RZ2_BackdropSide);

    camServo.setPosition(BotConstants.CAM_SERVO_REAR);
    sleep(2000);
    xyArray[0] = drive.getPoseEstimate().getX();
    xyArray[1] = drive.getPoseEstimate().getY();

    driveToTag = acquireTagLocation(new Vector2d(xyArray[0],xyArray[1]), team);
    telemetry.addLine(String.format("AprilTag: (%.3f,%.3f), tagFound: %b, id: %d",driveToTag.getX(),driveToTag.getY(), tagFound, tagIdFound));
    telemetry.addLine(String.format("PoseEstimate: (%.3f,%.3f)", xyArray[0], xyArray[1]));
    telemetry.update();

    TrajectorySequence RZ2_BackdropSideB =
    drive.trajectorySequenceBuilder(RZ2_BackdropSide.end())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))

    .lineToConstantHeading(driveToTag) // Go to April Tag location
    .build();
    drive.followTrajectorySequence(RZ2_BackdropSideB);

    // Drop Yellow Pixel
    setArmPosition(BotConstants.ARM_POS_AUTO_DEPLOY, BotConstants.ARM_POWER);
    setWristDeployPosition();
    // Put a blocking call after Arm and Wrist, will allow both to move at same time.
    while (armmotor.isBusy()) {
      sleep(10);
    }
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);
    sleep(400);

    // Close up for parking
    setArmPosition(BotConstants.ARM_POS_DRIVE, BotConstants.ARM_POWER);
    sleep(100);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
    setWristFoldPosition();

    // PARK
    TrajectorySequence RZ2_BackdropSideC =
    drive.trajectorySequenceBuilder(RZ2_BackdropSideB.start())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))

    .lineToConstantHeading(new Vector2d(51, -63)) // tweak
    .lineToLinearHeading(new Pose2d(64, -63, Math.toRadians(-180.00))) // tweak (16nov)
    .build();
    drive.followTrajectorySequence(RZ2_BackdropSideC);
  }


  public void RR_RZ1_Backdrop()
  {

    double[] xyArray = new double[2];

    // Lower Wrist
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FLOOR);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    visionPortal.setProcessorEnabled(contoursExtraction,false);

    // RZ1_BACKDROP
    TrajectorySequence RZ1_BackdropSide = drive.trajectorySequenceBuilder(new Pose2d(12, -62.50, Math.toRadians(90.00)))
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))


    //.lineToLinearHeading(new Pose2d(12.97, -32, Math.toRadians(-180.00))) // 34
    .splineToLinearHeading(new Pose2d(12.97, -32, Math.toRadians(-180.00)), Math.toRadians(-180.00)) // TWEAK 17Nov23 (Avoid truss bump)
    .addDisplacementMarker(() -> {
      // Deploy Purple on strike 1
      // TODO: REPLACE SLEEP with something else
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
      sleep(250);
      armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
      armmotor.setPower(1);
      sleep(250);
      bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
      setWristFoldPosition();
    })
    .lineToConstantHeading(new Vector2d(40, -35)) // view location  // Tweak was 36
    .build();

    drive.setPoseEstimate(RZ1_BackdropSide.start());
    drive.followTrajectorySequence(RZ1_BackdropSide);

    camServo.setPosition(BotConstants.CAM_SERVO_REAR);
    sleep(2000);
    xyArray[0] = drive.getPoseEstimate().getX();
    xyArray[1] = drive.getPoseEstimate().getY();

    driveToTag = acquireTagLocation(new Vector2d(xyArray[0],xyArray[1]), team);
    telemetry.addLine(String.format("AprilTag: (%.3f,%.3f), tagFound: %b, id: %d",driveToTag.getX(),driveToTag.getY(), tagFound, tagIdFound));
    telemetry.addLine(String.format("PoseEstimate: (%.3f,%.3f)", xyArray[0], xyArray[1]));
    telemetry.update();

    TrajectorySequence RZ1_BackdropSideB =
    drive.trajectorySequenceBuilder(RZ1_BackdropSide.end())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))

    .lineToConstantHeading(driveToTag) // Go to April Tag location
    .build();
    drive.followTrajectorySequence(RZ1_BackdropSideB);

    // Drop Yellow Pixel
    setArmPosition(BotConstants.ARM_POS_AUTO_DEPLOY, BotConstants.ARM_POWER);
    setWristDeployPosition();
    // Put a blocking call after Arm and Wrist, will allow both to move at same time.
    while (armmotor.isBusy()) {
      sleep(10);
    }
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);
    sleep(400);

    // Close up for parking
    setArmPosition(BotConstants.ARM_POS_DRIVE, BotConstants.ARM_POWER);
    sleep(100);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
    setWristFoldPosition();

    // PARK
    TrajectorySequence RZ1_BackdropSideC =
    drive.trajectorySequenceBuilder(RZ1_BackdropSideB.start())
    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,45,17.66),
    SampleMecanumDrive.getAccelerationConstraint(30))
    .setTurnConstraint(Math.toRadians(120),Math.toRadians(120))

    .lineToConstantHeading(new Vector2d(51, -65)) // tweak
    .lineToLinearHeading(new Pose2d(64, -65, Math.toRadians(-180.00))) // tweak
    .build();
    drive.followTrajectorySequence(RZ1_BackdropSideC);
  }


  // TODO: ********************************** END OF TRAJECTORIES ******************************
  public void RRRunAutomation() {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    // Lower Wrist
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FLOOR);

    // Build Trajectory
    Trajectory toPurplePixel = null;
    if (zone == 1) {
      toPurplePixel = drive.trajectoryBuilder(new Pose2d())
      .lineToSplineHeading(new Pose2d(34, 0, Math.toRadians(90)))
      .build();

    } else if (zone == 2) {
      toPurplePixel = drive.trajectoryBuilder(new Pose2d())
      .lineToSplineHeading(new Pose2d(31, 0, Math.toRadians(0)))
      .build();

    } else {
      toPurplePixel = drive.trajectoryBuilder(new Pose2d())
      .lineToSplineHeading(new Pose2d(34, 0, Math.toRadians(-90)))
      .build();
    }

    drive.followTrajectory(toPurplePixel);

    // Open Bottom Finger to deposit Purple Pixel on Strike mark
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    sleep(1000);
    armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
    armmotor.setPower(1);
    sleep(750);
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
    camServo.setPosition(BotConstants.CAM_SERVO_REAR);
    Vector2d driveToTag;
    double[] xyArray = new double[2];
    xyArray[0] = drive.getPoseEstimate().getX();
    xyArray[1] = drive.getPoseEstimate().getY();

    driveToTag = acquireTagLocation(new Vector2d(xyArray[0],xyArray[1]), 0);

      Trajectory toAprilTag = drive.trajectoryBuilder(ThroughTruss.end())
      .splineTo(driveToTag, drive.getExternalHeading(), // -9.5 distance from camera Y to AprilTag Y
      SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
      )
      .build();

      drive.followTrajectory(toAprilTag);

    // Deploy Yellow Pixel
    setArmPosition(BotConstants.ARM_POS_AUTO_DEPLOY, BotConstants.ARM_POWER);
    setWristDeployPosition();

    // Put a blocking call after Arm and Wrist, will allow both to move at same time.
    while (armmotor.isBusy()) {
      // block
    }
    // sleep(BotConstants.WRIST_DEPLOY_SLEEP);

    // Open the fingers and drop the pixel(s)
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);
    sleep(750);


    // Raise Arm and close fingers, fold wrist
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
    setWristFoldPosition();
    setArmPosition(BotConstants.ARM_POS_DRIVE, BotConstants.ARM_POWER);

  }

  public void PrintSome_telemetry()
  {
    double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    telemetry.addData("imu", "yaw: %.2f", botHeading);
    telemetry.update();
  }
  public void forwardSecs(double power, double seconds) {
    motor1.setPower(power);
    motor2.setPower(-power);
    motor3.setPower(power);
    motor4.setPower(-power);
    sleep(Math.round(seconds * 1000));
    motor1.setPower(0);
    motor2.setPower(0);
    motor3.setPower(0);
    motor4.setPower(0);
  }

  public void leftTurnSecs(double power, double seconds) {
    motor1.setPower(-power);
    motor2.setPower(-power);
    motor3.setPower(-power);
    motor4.setPower(-power);
    sleep(Math.round(seconds * 1000));
    motor1.setPower(0);
    motor2.setPower(0);
    motor3.setPower(0);
    motor4.setPower(0);
  }

  public void rightTurnSecs(double power, double seconds) {
    motor1.setPower(power);
    motor2.setPower(power);
    motor3.setPower(power);
    motor4.setPower(power);
    sleep(Math.round(seconds * 1000));
    motor1.setPower(0);
    motor2.setPower(0);
    motor3.setPower(0);
    motor4.setPower(0);
  }


  public void initHardware() {

    //map hardware
    control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
    bottomArmServo = hardwareMap.get(Servo.class, "bottomArmServo");
    topArmServo = hardwareMap.get(Servo.class, "topArmServo");
    wristPanServo = hardwareMap.get(Servo.class, "wristPanServo");
    camServo = hardwareMap.get(Servo.class, "camServo");
    droneServo = hardwareMap.get(Servo.class, "droneServo");
    hookServo = hardwareMap.get(Servo.class, "hookServo");

    motor1 = hardwareMap.get(DcMotor.class, "motor1");
    motor2 = hardwareMap.get(DcMotor.class, "motor2");
    motor3 = hardwareMap.get(DcMotor.class, "motor3");
    motor4 = hardwareMap.get(DcMotor.class, "motor4");
    armmotor = hardwareMap.get(DcMotor.class, "armcontrol");

    //set motor behavior
    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    armmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    armmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    //Set initial positions for automation

    // init Wrist (FOLDED)
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FOLDED);

    // init Hook (RETRACT)
    hookServo.setPosition(BotConstants.HOOK_POS_RETRACT);

    // init drone Servo (ARMED)
    droneServo.setPosition(BotConstants.DRONE_POSITION_ARMED);


    // Close Both fingers around purple and yellow pixels
    topArmServo.setDirection(Servo.Direction.REVERSE);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);

    // init imu
    imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
    ));
    imu.initialize(parameters);

    // Set initial heading to ZERO
    imu.resetYaw();

    // Raise arm off ground (flat-hand level)
    armmotor.setTargetPosition(BotConstants.ARM_POS_FLOOR);
    armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armmotor.setPower(1);

    // Set Camera Servo facing FRONT
    camServo.setPosition(BotConstants.CAM_SERVO_SPIKE);


    // initialize camera
    VisionPortal.Builder builder = new VisionPortal.Builder();

    // Select the Camera
    if (USE_WEBCAM) {
      builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    } else {
      builder.setCamera(BuiltinCameraDirection.BACK);
    }

    // Choose a camera resolution. Not all cameras support all resolutions.
    //builder.setCameraResolution(new Size(640, 480));

    // JW April Tag
    aprilTag = new AprilTagProcessor.Builder()
      .build();
    aprilTag.setDecimation(1);

    // Add both Processors to the Portal
    builder.addProcessors(contoursExtraction, aprilTag);


    // Build the Vision Portal, using the above settings.
    visionPortal = builder.build();

    while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){
      // wait until Camera is live
    }

    // ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
    // exposure.isExposureSupported();
    
  }


  @SuppressLint("DefaultLocale")
  public int detectZone() {

    int[] result = contoursExtraction.getResult();


    if (result[0] < BotConstants.Z1_RECT_BRX && result[0] > BotConstants.Z1_RECT_TLX) {
      telemetry.addData("Detect Zone", "1");
      zone = 1;
    }
    else if (result[0] < BotConstants.Z2_RECT_BRX && result[0] > BotConstants.Z2_RECT_TLX) {
      telemetry.addData("Detect Zone", "2");
      zone = 2;
    }
    else
    {
      zone = 3;
      if (result[0] < BotConstants.Z3_RECT_BRX && result[0] > BotConstants.Z3_RECT_TLX) {
        telemetry.addData("Detect Zone", "3");
      }
      else {telemetry.addData("xDetect Zone", "3");}
    }

    telemetry.addLine(String.format("%d,%d", result[0], result[1]));
    //telemetryAprilTag();
    telemetry.update();
    return zone;
  }

  @SuppressLint("DefaultLocale")
  public Vector2d acquireTagLocation(Vector2d Current, int myTeam) {
    boolean found=false;
    int tries=0;
    Vector2d Future;

    // These are the field locations of the tags
    Vector2d[][] fallbackLoc = {
    {new Vector2d(61.22,41.66), new Vector2d(61.22,35.63),new Vector2d(61.22,29.24)},  // BLUE tags
    {new Vector2d(61.22,-30.15),new Vector2d(61.22,-36.37),new Vector2d(61.22,-42.4)}   // RED tags
    };

    int targetId;

    // Team: BLUE=0, RED = 1
    // Set a 'Fallback' location of tags as defined in the rules.
    if (myTeam == BotConstants.BLUE_TEAM) {
      targetId = zone+BotConstants.BLUE_TEAM_ID_OFFSET;
      Future = new Vector2d( fallbackLoc[0][zone-1].getX()+BotConstants.APRIL_POSE_YOFFSET,
                                fallbackLoc[0][zone-1].getY());
    } else {
      targetId = zone+BotConstants.RED_TEAM_ID_OFFSET;
      Future = new Vector2d (fallbackLoc[1][zone-1].getX()+BotConstants.APRIL_POSE_YOFFSET,
                                fallbackLoc[1][zone-1].getY());
    }

    // Main loop will check 5 times for April Tag detections, first choice is OUR target tag
    // next choice is any other tag,  final choice is where tag should be on the field.
    while (tries<5) {
      List<AprilTagDetection> currentDetections = aprilTag.getDetections();
      telemetry.addData("# AprilTags Detected", currentDetections.size());

      // Step through the list of detections and calculate drop location
      for (AprilTagDetection detection : currentDetections) {
        if (detection.metadata != null) {
            // ftcPose camera to tag looks like "+Y" move towards backdrop, "X" move to the right, "z" is ignored
            // Reverse GetY and GetX from Field pose
            Future = new Vector2d(Current.getX() +  Math.cos(BotConstants.CAM_TILT_ANGLE_RAD)*detection.ftcPose.y + BotConstants.APRIL_POSE_YOFFSET,
                                  Current.getY() - (detection.ftcPose.x + (targetId - detection.id)*BotConstants.TAG_TO_TAG_DIST) + BotConstants.APRIL_POSE_XOFFSET);
            found = true;
            tagIdFound = targetId;
            tagFound = found;
            if (detection.id == targetId) {
              telemetry.addData("Found ours: ", targetId);
              telemetry.update();
              return Future;
            }

          telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
          telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
          telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
          telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
        }
      }   // end for() loop
      if (found)
      {
        return Future;
      }

      telemetry.update();
      sleep(1000); // wait a bit
      tries += 1;
    } // end while loop until any tag detected or n-tries expires
    tagIdFound = -1;
    tagFound = false;
    return Future;
  }
  @SuppressLint("DefaultLocale")
  private void telemetryAprilTag() {

    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    telemetry.addData("# AprilTags Detected", currentDetections.size());

    // Step through the list of detections and display info for each one.
    for (AprilTagDetection detection : currentDetections) {
      if (detection.metadata != null) {
        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
      } else {
        telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
        telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
      }
    }   // end for() loop

    // Add "key" information to telemetry
    telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    telemetry.addLine("RBE = Range, Bearing & Elevation");

  }   // end method telemetryAprilTag()

  public void setArmDrivePosition() {
    armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
    armmotor.setPower(BotConstants.ARM_POWER);
  }

  public void setArmPosition(int position, double speed) {
    armmotor.setTargetPosition(position);
    armmotor.setPower(speed);
  }

  public void setWristFoldPosition() {
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FOLDED);
  }
  public void setWristDeployPosition() {
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_AUTO_DEPLOY);
  }


  public class Contours_Extraction implements VisionProcessor {
    /*
     * Our working image buffers
     */
    Mat hsv = new Mat();
    Mat thresh1 = new Mat();
    Mat thresh2 = new Mat();
    Mat kernel = new Mat();
    Mat red_thresh = new Mat();
    Mat blue_thresh = new Mat();
    Mat thresh = new Mat();
    Mat morph = new Mat();
    Mat mask = new Mat();

    Scalar rLower1 = new Scalar(0, 70, 50);
    Scalar rUpper1 = new Scalar(10, 255, 255);
    Scalar rLower2 = new Scalar(170, 70, 50);
    Scalar rUpper2 = new Scalar(180, 255, 255);
    Scalar bLower = new Scalar(BotConstants.BLUE_HUE_LOW, BotConstants.BLUE_SAT_LOW, BotConstants.BLUE_VAL_LOW);
    Scalar bUpper = new Scalar(BotConstants.BLUE_HUE_HIGH, BotConstants.BLUE_SAT_HIGH, BotConstants.BLUE_VAL_HIGH);
    MatOfPoint big_contour = null;
    int big_contourIdx = 0;
    int cX, cY = 0;
    int[] xyArray = new int[2];


    @Override
    public void init(int width, int height, CameraCalibration calibration) {
      // Code executed on the first frame dispatched into this VisionProcessor
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {

      // EasyOpenCV uses "RGB" order instead of BGR (like OpenCV does).
      Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);


      // red Hue goes from ~170-10 (wraps around 0). need two ranges
      Core.inRange(hsv, rLower1, rUpper1, thresh1);
      Core.inRange(hsv, rLower2, rUpper2, thresh2);

      // red_thresh = thresh1 | thresh2
      Core.bitwise_or(thresh1, thresh2, red_thresh);

      // Blue thresholds  hue: ~98-140
      Core.inRange(hsv, bLower, bUpper, blue_thresh); // blue_thresh

      // thresh = red_thresh | blue_thresh
      Core.bitwise_or(red_thresh, blue_thresh, thresh); //blue thresh

      // Make a 3x3 'elliptical' shape kernel for matrix convolution
      kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(3, 3));

      // apply close and open morphology (removes noise) (erode/dilate) and save to 'morph'
      Imgproc.morphologyEx(thresh, morph, Imgproc.MORPH_CLOSE, kernel);
      Imgproc.morphologyEx(morph, morph, Imgproc.MORPH_OPEN, kernel);

      // Create black mask image
      mask.create(morph.rows(), morph.cols(), morph.type());

      // fill white rectangle in area where prop will be
      Imgproc.rectangle(mask,
              new Point(BotConstants.ROI_RECT_TOP_LEFT_X,BotConstants.ROI_RECT_TOP_LEFT_Y),
              new Point(BotConstants.ROI_RECT_BOTTOM_RIGHT_X,BotConstants.ROI_RECT_BOTTOM_RIGHT_Y),
              new Scalar(255,255,255),
              Imgproc.FILLED);

      // ignore all bits of MORPH outside
      Core.bitwise_and(morph,mask,morph);

      // Get the contours in the morph image buffer
      ArrayList<MatOfPoint> contoursList = findContours(morph);

      // Find the largest contour and it's index (by area) and store it in big_contour
      double maxVal = 0;
      big_contourIdx =0;
      for (int contourIdx = 0; contourIdx < contoursList.size(); contourIdx++) {
        double contourArea = Imgproc.contourArea(contoursList.get(contourIdx));
        if (maxVal < contourArea) {
          maxVal = contourArea;
          big_contour = contoursList.get(contourIdx);
          big_contourIdx = contourIdx;
        }
      }

      // Draw that contour (Filled) A clean buffer
      Imgproc.drawContours(input, contoursList, big_contourIdx, new Scalar(255, 255, 0), Imgproc.FILLED);

      // Draw 3 Zone Rectangles
      Imgproc.rectangle(input,
              new Point(BotConstants.Z1_RECT_TLX,BotConstants.Z1_RECT_TLY),
              new Point (BotConstants.Z1_RECT_BRX,BotConstants.Z1_RECT_BRY),
              new Scalar(255,0,0) );
      Imgproc.rectangle(input,
              new Point(BotConstants.Z2_RECT_TLX,BotConstants.Z2_RECT_TLY),
              new Point (BotConstants.Z2_RECT_BRX,BotConstants.Z2_RECT_BRY),
              new Scalar(255,0,0) );
      Imgproc.rectangle(input,
              new Point(BotConstants.Z3_RECT_TLX,BotConstants.Z3_RECT_TLY),
              new Point (BotConstants.Z3_RECT_BRX,BotConstants.Z3_RECT_BRY),
              new Scalar(255,0,0) );

      //Imgproc.rectangle(morph, new Rect(0, 0, 320, 80), new Scalar(255,0,0));
      //Imgproc.rectangle(morph, new Rect(0, 80, 160, 240), new Scalar(255,0,0));
      //Imgproc.rectangle(morph, new Rect(160, 80, 320, 240), new Scalar(255,0,0));

      // Find the Center point of largest contour if there is one.. using moments and store it
      if (big_contour != null) {
        Moments M = Imgproc.moments(big_contour);
        cX = (int) (M.get_m10() / M.get_m00());
        cY = (int) (M.get_m01() / M.get_m00());
      }


      Imgproc.putText(input, String.format("%d,%d", cX, cY), new Point(cX+10,cY+10), Imgproc.FONT_HERSHEY_PLAIN, 2.5, new Scalar (255,0,0));

      // return null (input has our image)
      return null;

    }

    ArrayList<MatOfPoint> findContours(Mat input) {
      // A list to store the contours we find
      ArrayList<MatOfPoint> contoursList = new ArrayList<>();

      // look for the contours
      Imgproc.findContours(input, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

      return contoursList;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }


    // Returns the Largest Contour Prop location (Zone 1, 2, 3)
    int[] getResult() {

      xyArray[0] = cX;
      xyArray[1] = cY;
      return xyArray;
    }
  }
}