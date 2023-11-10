package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Canvas;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;


@Autonomous(group = "Testing", name = "RR auto")

public class RRAutoSoftware extends LinearOpMode {
  private Blinker control_Hub;
  private Servo bottomArmServo;
  private IMU imu;
  private Servo topArmServo;
  private Servo wristPanServo;
  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor motor1 = null; //front left
  private DcMotor motor2 = null; //front right
  private DcMotor motor3 = null; //back left
  private DcMotor motor4 = null; //back right
  private DcMotor armmotor = null;
  int zone;
  double forwardTime;
  double lTurnTime;
  double rTurnTime;
  double BOTTOM_ARM_SERVO_CLOSE = 0.10;
  double BOTTOM_ARM_SERVO_OPEN = 0.30;
  double TOP_ARM_SERVO_CLOSE = 0.10;
  double TOP_ARM_SERVO_OPEN = 0.30;
  double WRIST_PAN_SERVO_FOLDED = 0.6;
  double WRIST_PAN_SERVO_FLOOR = 0;
  double WRIST_PANSERVO_AUTO_DEPLOY = 0.32;
  // Arm Constants
  int ARM_POS_FLOOR = 150;
  int ARM_POS_90 = 400;
  int ARM_POS_AUTO_DEPLOY = 7718;

  // Vision portal Replaces EasyOpenCV method
  private VisionPortal visionPortal;
  Contours_Extraction pipeline = new Contours_Extraction();
  private static final boolean USE_WEBCAM = true;

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initializing");
    telemetry.update();

    initHardware();

    setupVariables();

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    waitForStart();

    zone = detectZone();


    // Set the Wrist to 'floor' mode for first pixel drop
    wristPanServo.setPosition(WRIST_PAN_SERVO_FLOOR);


    RRRunAutomation();


  }

  public void runAutomation() {
    // automatic movement
    // Todo: replace with RR trajectory

    forwardSecs(0.5, forwardTime);
    if (zone == 1) {
      leftTurnSecs(0.5, lTurnTime);
    } else if (zone == 3) {
      rightTurnSecs(0.5, rTurnTime);
    }

    // open Purple pixel Servo
    bottomArmServo.setPosition(BOTTOM_ARM_SERVO_OPEN);

    double temp = bottomArmServo.getPosition();
    telemetry.addData("Bottom Servo", "Command: %.3f, Value: %.3f", BOTTOM_ARM_SERVO_OPEN, temp);
    telemetry.update();

    // Todo: move arm up to 90 and out of way

    // Todo: back up to start position and orient to Backdrop "North"

    // Todo: Navigate to Backdrop

    // Todo: Acquire April Tag by zone (Red: 1,2,3  blue: 4,5,6)

    // Todo: Navigate to TagID

    // Todo: Position Arm and Wrist for deploy

    // Todo: Deploy Yellow Pixel

    // Todo: Park and wait

    sleep(20000);

  }

  public void RRRunAutomation() {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    Trajectory toPurplePixel = null;
    if (zone == 1) {
      toPurplePixel = drive.trajectoryBuilder(new Pose2d())
        .lineToSplineHeading(new Pose2d(32.6, 0, Math.toRadians(90)))
        .build();

    }
    else if (zone == 2) {
      toPurplePixel = drive.trajectoryBuilder(new Pose2d())
        .lineToSplineHeading(new Pose2d(32.6, 0, Math.toRadians(0)))
        .build();

    }
    else {
      toPurplePixel = drive.trajectoryBuilder(new Pose2d())
        .lineToSplineHeading(new Pose2d(32.6, 0, Math.toRadians(-90)))
        .build();
    }

    drive.followTrajectory(toPurplePixel);

    bottomArmServo.setPosition(BOTTOM_ARM_SERVO_OPEN);

    armmotor.setTargetPosition(ARM_POS_90);

    Trajectory toYellowPixel = drive.trajectoryBuilder(toPurplePixel.end()) // continue from old pose
      //.back(3)
      .splineToSplineHeading(new Pose2d(3, 0, Math.toRadians(90)), Math.toRadians(0)) // back to start turn left 90
      //.lineToSplineHeading(new Pose2d(3, 0, Math.toRadians(90)))
      .splineToSplineHeading(new Pose2d(3, 48, Math.toRadians(90)), Math.toRadians(0))  // move forward (Y) 48 old -48 0 90
      .splineToSplineHeading(new Pose2d(32.6, 82, Math.toRadians(-90)), Math.toRadians(0)) // old -32 -28
      .build();

    drive.followTrajectory(toYellowPixel);

    armmotor.setTargetPosition(ARM_POS_AUTO_DEPLOY);

    wristPanServo.setPosition(WRIST_PANSERVO_AUTO_DEPLOY);



    topArmServo.setPosition(TOP_ARM_SERVO_OPEN);

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

  public void setupVariables() {
    //setup variables
    zone = (int) Math.floor((Math.random() * 3 + 1)); // replace with recognition code
    forwardTime = 1;
    lTurnTime = 0.9;
    rTurnTime = 1;
  }

  public void initHardware() {
    //map hardware

    control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
    bottomArmServo = hardwareMap.get(Servo.class, "bottomArmServo");
    topArmServo = hardwareMap.get(Servo.class, "topArmServo");
    wristPanServo = hardwareMap.get(Servo.class, "wristPanServo");
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


    //set initial positions

    // Fold Wrist
    wristPanServo.setPosition(WRIST_PAN_SERVO_FOLDED); // 0.5

    // Set Reverse and Close both fingers
    topArmServo.setDirection(Servo.Direction.REVERSE);
    bottomArmServo.setPosition(BOTTOM_ARM_SERVO_CLOSE);
    topArmServo.setPosition(TOP_ARM_SERVO_CLOSE);

    // init imu
    imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
    ));
    imu.initialize(parameters);

    // Raise arm off ground (flat hand level)
    armmotor.setTargetPosition(ARM_POS_FLOOR);
    armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armmotor.setPower(1);

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

    // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
    //builder.enableLiveView(true);

    // Set the stream format; MJPEG uses less bandwidth than default YUY2.
    //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

    // Choose whether or not LiveView stops if no processors are enabled.
    // If set "true", monitor shows solid orange screen if no processors enabled.
    // If set "false", monitor shows camera view without annotations.
    //builder.setAutoStopLiveView(false);

    // Set and enable the processor.
    builder.addProcessor(pipeline);

    // Build the Vision Portal, using the above settings.
    visionPortal = builder.build();

  }


  public int detectZone() {

    int[] result = pipeline.getResult();

    if (result[1] < 80 && result[0] > 120 && result[0] < 200) {
      telemetry.addData("zone", "2");
      zone = 2;
    } else if (result[0] < 120) {
      telemetry.addData("zone", "1");
      zone = 1;
    } else if (result[0] > 200) {
      telemetry.addData("zone", "3");
      zone = 3;
    } else {
      zone = 2;
    }

    telemetry.addLine(String.format("%d,%d", result[0], result[1]));
    telemetry.update();
    return zone;
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
    Scalar bLower = new Scalar(98, 38, 10);
    Scalar bUpper = new Scalar(140, 255, 255);
    MatOfPoint big_contour = null;
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
      Core.inRange(input, bLower, bUpper, blue_thresh);

      // thresh = red_thresh | blue_thresh
      Core.bitwise_or(red_thresh, blue_thresh, thresh);

      // Make a 3x3 'elliptical' shape kernel for matrix convolution
      kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(3, 3));

      // apply close and open morphology (removes noise) (erode/dilate) and save to 'morph'
      Imgproc.morphologyEx(thresh, morph, Imgproc.MORPH_CLOSE, kernel);
      Imgproc.morphologyEx(thresh, morph, Imgproc.MORPH_OPEN, kernel);

      // Get the contours in the morph image buffer
      ArrayList<MatOfPoint> contoursList = findContours(morph);

      // Find the largest contour and it's index (by area) and store it in big_contour
      double maxVal = 0;
      int maxValIdx = 0;
      for (int contourIdx = 0; contourIdx < contoursList.size(); contourIdx++) {
        double contourArea = Imgproc.contourArea(contoursList.get(contourIdx));
        if (maxVal < contourArea) {
          maxVal = contourArea;
          maxValIdx = contourIdx;
          big_contour = contoursList.get(contourIdx);
        }
      }

      // Draw that contour (Filled) A clean buffer
      Imgproc.drawContours(input, contoursList, maxValIdx, new Scalar(255, 255, 0), Imgproc.FILLED);

      //Imgproc.rectangle(morph, new Rect(0, 0, 320, 80), new Scalar(255,0,0));
      //Imgproc.rectangle(morph, new Rect(0, 80, 160, 240), new Scalar(255,0,0));
      //Imgproc.rectangle(morph, new Rect(160, 80, 320, 240), new Scalar(255,0,0));

      // Find the Center point of largest contour if there is one.. using moments and store it
      if (big_contour != null) {
        Moments M = Imgproc.moments(big_contour);
        cX = (int) (M.get_m10() / M.get_m00());
        cY = (int) (M.get_m01() / M.get_m00());
      }

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

      // TODO: Triangle it is inside of
      // TODO: zone1: (x: 320, y: 480), (0,0), (0,480)  "left"
      // TODO: zone2: (x: 320, y: 480), (640,0), (0,0)  "center"
      // TODO: zone3: (x: 320, y: 480), (640,480), (640,0)  "right"
      xyArray[0] = cX;
      xyArray[1] = cY;
      return xyArray;
    }
  }
}