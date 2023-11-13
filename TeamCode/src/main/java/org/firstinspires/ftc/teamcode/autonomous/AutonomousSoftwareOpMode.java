package org.firstinspires.ftc.teamcode.autonomous;

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
  private Blinker control_Hub;
  private Servo bottomArmServo;
  private IMU imu;
  private Servo topArmServo;
  private Servo wristPanServo;
  private Servo camServo;
  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor motor1 = null; //front left
  private DcMotor motor2 = null; //front right
  private DcMotor motor3 = null; //back left
  private DcMotor motor4 = null; //back right
  private DcMotor armmotor = null;
  int zone;

  // ALL CONSTANTS MOVED TO Common.BotConstants class



  // Vision portal Replaces EasyOpenCV method
  private VisionPortal visionPortal;
  Contours_Extraction pipeline = new Contours_Extraction();

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

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    while (!isStarted()){
      detectZone();
    }
    //waitForStart();

    // Set the Wrist to 'floor' mode for first pixel drop
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FLOOR);

    zone = detectZone();

    RRRunAutomation();


    while (opModeIsActive())
    {
      // do nothing after Automation until end
    }

  }


  public void RRRunAutomation() {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    Trajectory toPurplePixel = null;
    if (zone == 1) {
      toPurplePixel = drive.trajectoryBuilder(new Pose2d())
        .lineToSplineHeading(new Pose2d(34, 0, Math.toRadians(90)))
        .build();

    }
    else if (zone == 2) {
      toPurplePixel = drive.trajectoryBuilder(new Pose2d())
        .lineToSplineHeading(new Pose2d(34, 0, Math.toRadians(0)))
        .build();

    }
    else {
      toPurplePixel = drive.trajectoryBuilder(new Pose2d())
        .lineToSplineHeading(new Pose2d(34, 0, Math.toRadians(-90)))
        .build();
    }

    drive.followTrajectory(toPurplePixel);

    // Open Bottom Finger to deposit Purple Pixel on Strike mark
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    armmotor.setTargetPosition(BotConstants.ARM_POS_DRIVE);
    armmotor.setPower(1);
    sleep(1000);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    setWristFoldPosition();


    // Navigate to Backdrop (Yellow Pixel)
    // TODO: CHECK / The Pose and Trajectory may be centered on Control Hub or Robot
    // TODO: Change Poses to FIELD CENTRIC COORDINATES
    Trajectory backToStart = drive.trajectoryBuilder(toPurplePixel.end())
            .lineToSplineHeading(new Pose2d(12,0, drive.getRawExternalHeading()))
            .lineToSplineHeading(new Pose2d(5, 0, Math.toRadians(-90)))
            .build();

    // Execute Drive trajectory
    drive.followTrajectory(backToStart);

    Trajectory ThroughTruss = drive.trajectoryBuilder(backToStart.end())
            .lineToSplineHeading(new Pose2d(5, 55, drive.getRawExternalHeading())) // was 72
            .splineToConstantHeading(new Vector2d(44, 48), Math.toRadians(0)) // was 75 was 35
            .build();

    drive.followTrajectory(ThroughTruss);


    // Todo: Acquire April Tag by zone (Red: 4,5,6  blue: 1,2,3)
    camServo.setPosition(BotConstants.CAM_SERVO_REAR);

    // Todo: Navigate to TagID (red: 3+[id] or  blue: [id]])
    while (!gamepad1.a)
    {
      telemetryAprilTag();
      telemetry.update();
    }
    double[] xyArray = acquireTagLocation();

    while (!gamepad1.b)
    {
      double x = drive.getPoseEstimate().getX();
      double y = drive.getPoseEstimate().getY();
      double head = drive.getExternalHeading();
      telemetry.addLine(String.format("offset: (%f,%f), pose: (%f,%f), heading=%f", xyArray[0],xyArray[1],x,y,head));
      telemetry.update();
    }

    if (xyArray[0] != 999.0) {

      Trajectory toAprilTag = drive.trajectoryBuilder(ThroughTruss.end())
      .splineTo(new Vector2d(drive.getPoseEstimate().getX() + (xyArray[0]-4), // was 6
      drive.getPoseEstimate().getY() + (xyArray[1] - 9.5)), drive.getExternalHeading(),
      SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
      )
      .build();

      drive.followTrajectory(toAprilTag);
    }
    // Deploy Yellow Pixel
    // TODO: TEST YELLOW DEPLOY
    setArmDeployPosition(BotConstants.ARM_POS_AUTO_DEPLOY, BotConstants.ARM_POWER);
    while (armmotor.isBusy()) {
      // PrintSome_telemetry();
    }
//    sleep(BotConstants.ARM_DEPLOY_SLEEP);
    setWristDeployPosition();
    sleep(BotConstants.WRIST_DEPLOY_SLEEP);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);


    // Todo: Park

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


    //set initial positions for automation

    // Fold Wrist
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FOLDED); // 0.5

    // Set Reverse and Close both fingers to ARM around purble and yellow pixel.
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
    // TODO: (this will be carried over to TeleOp, so don't re-init)
    imu.resetYaw();

    // Raise arm off ground (flat-hand level)
    armmotor.setTargetPosition(BotConstants.ARM_POS_FLOOR);
    armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armmotor.setPower(1);

    // Set Camera Servo facing FRONT
    camServo.setPosition(BotConstants.CAM_SERVO_STRIKE);



    // initialize camera
    VisionPortal.Builder builder = new VisionPortal.Builder();

    // Set the camera (webcam vs. built-in RC phone camera).
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

    // Set the stream format; MJPEG uses less bandwidth than default YUY2.
    //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

    // Add both Processors to the Portal
    builder.addProcessors(pipeline, aprilTag);

    // Build the Vision Portal, using the above settings.
    visionPortal = builder.build();

  }


  public int detectZone() {

    int[] result = pipeline.getResult();

    // Zone1 = yDetectLoc < 1.5 xDetectLoc
    // Zone2 = yDetectLoc < -1.5 xDetectLoc + 960
    // Zone3

    /*
    if (result[1] > 1.5*result[0]) {
      telemetry.addData("zone", "1");
      zone = 1;
    }
    else if (result[1] < -1.5 * result[0] + 960) {
      telemetry.addData("zone", "2");
      zone = 2;
    }
    else {
      telemetry.addData("zone", "3");
      zone = 3;
    }

     */

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
      else {
        telemetry.addData("xDetect Zone", "3");
      }

    }

    telemetry.addLine(String.format("%d,%d", result[0], result[1]));
    telemetryAprilTag();
    telemetry.update();
    return zone;
  }

  public double[] acquireTagLocation() {
    boolean found=false;
    double[] xyArray = new double[2];

    while (!found) {
      List<AprilTagDetection> currentDetections = aprilTag.getDetections();
      telemetry.addData("# AprilTags Detected", currentDetections.size());

      // Step through the list of detections and display info for each one.
      for (AprilTagDetection detection : currentDetections) {
        if (detection.metadata != null) {
          if (detection.id == zone + 3) {
            // TODO: pretend we are red team for now JW Fix
            xyArray[0] = detection.ftcPose.x;
            xyArray[1] = detection.ftcPose.y;
            found = true;
            return xyArray;
          } else {
            xyArray[0] = 999.0;
            xyArray[1] = 999.0;
          }
          telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
          telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
          telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
          telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
        }
      }   // end for() loop
      telemetry.update();
    } // end while loop until detect or TODO: Fail for park on timeout
    return xyArray;
  }
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

  public void setArmDeployPosition(int position, double speed) {
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
    // Scalar bLower = new Scalar(98, 38, 10);
    // Scalar bUpper = new Scalar(140, 255, 255);
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
      // Cool feature: This method is used for drawing annotations onto
      // the displayed image, e.g outlining and indicating which objects
      // are being detected on the screen, using a GPU and high quality
      // graphics Canvas which allow for crisp quality shapes.

    }


    // Returns the Largest Contour Prop location (Zone 1, 2, 3)
    int[] getResult() {

      xyArray[0] = cX;
      xyArray[1] = cY;
      return xyArray;
    }
  }
}