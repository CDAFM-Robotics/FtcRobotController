package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(group = "Testing", name = "autonomous")

public class AutonomousSoftwareOpMode extends LinearOpMode {
  private Blinker control_Hub;
  private Servo bottomArmServo;
  private Gyroscope imu;
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
  double bottomArmServoClose = 0;
  double bottomArmServoOpen = 0.15;
  double topArmServoClose = 0.6;
  double topArmServoOpen = 0.45;
  double wristPanServoFolded = 0.6;
  double wristPanServoFloor = 0;
  OpenCvWebcam webcam;
  OpenCV_ContoursEx.Contours_Extraction pipeline;


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


    // raise arm to new zero point off ground (flat hand level)
    // TODO: Change code to not allow armmotor to drive below 200 count after this point
    armmotor.setTargetPosition(200);
    armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armmotor.setPower(1);

   /* while (opModeIsActive()) {
      telemetry.addData("ArmPos", "" + armmotor.getCurrentPosition());
      telemetry.update();
    }
   */

    // Set the Wrist to 'floor' mode for first pixel drop
    wristPanServo.setPosition(wristPanServoFloor);


   runAutomation();


  }

  public void runAutomation() {
    // automatic movement
    forwardSecs(0.5, forwardTime);
    if (zone == 1) {
      leftTurnSecs(0.5, lTurnTime);
    }
    else if (zone == 3) {
      rightTurnSecs(0.5, rTurnTime);
    }

    bottomArmServo.setPosition(bottomArmServoOpen);
    double temp = bottomArmServo.getPosition();
    telemetry.addData("Bottom Servo", "Command: %.3f, Value: %.3f", bottomArmServoOpen, temp);
    telemetry.update();
    sleep(20000);

  }
  public void forwardSecs(double power, double seconds) {
    motor1.setPower(power);
    motor2.setPower(-power);
    motor3.setPower(power);
    motor4.setPower(-power);
    sleep(Math.round(seconds*1000));
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
    sleep(Math.round(seconds*1000));
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
    sleep(Math.round(seconds*1000));
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
    wristPanServo.setPosition(wristPanServoFolded); // 0.5

    // Close fingers
    bottomArmServo.setPosition(bottomArmServoClose);
    topArmServo.setPosition(topArmServoClose);

    // initialize camera
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

    // Open async and start streaming inside opened callback
    webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
      @Override
      public void onOpened() {
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        pipeline = new OpenCV_ContoursEx.Contours_Extraction();
        webcam.setPipeline(pipeline);
      }

      @Override
      public void onError(int errorCode) {

      }
    });


  }
  public int detectZone() {

    int[] result = pipeline.getResult();

    if (result[1] < 80 && result[0] > 120 && result[0] < 200) {
      telemetry.addData("zone", "2");
      zone = 2;
    }
    else if (result[0] < 120) {
      telemetry.addData("zone", "1");
      zone = 1;
    }
    else if (result[0] > 200) {
      telemetry.addData("zone", "3");
      zone = 3;
    }
    else {
      zone = 2;
    }

    telemetry.addLine(String.format("%d,%d",result[0],result[1]));
    telemetry.update();
    return zone;
  }
}

class Contours_Extraction extends OpenCvPipeline {
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

  static final Scalar rLower1 = new Scalar (0,70,50);
  static final Scalar rUpper1 = new Scalar (10,255,255);
  static final Scalar rLower2 = new Scalar (170,70,50);
  static final Scalar rUpper2 = new Scalar (180,255,255);
  static final Scalar bLower = new Scalar (98,38,10);
  static final Scalar bUpper = new Scalar (140,255,255);
  MatOfPoint big_contour = null;
  static int cX, cY = 0;
  static int[] xyArray = new int[2];

  @Override
  public Mat processFrame(Mat input) {

    // Maybe EasyOpenCV uses "RGB" order instead of BGR (like OpenCV does).
    // Todo: This needs Testing to verify!
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
    for (int contourIdx = 0; contourIdx < contoursList.size(); contourIdx++)
    {
      double contourArea = Imgproc.contourArea(contoursList.get(contourIdx));
      if (maxVal < contourArea)
      {
        maxVal = contourArea;
        maxValIdx = contourIdx;
        big_contour = contoursList.get(contourIdx);
      }
    }

    // Draw that contour (Filled) A clean buffer
    // TODO: replace 'morph' which has drawings with an empty buffer of same size
    Imgproc.drawContours(morph, contoursList, maxValIdx, new Scalar(255,255,0), Imgproc.FILLED);

    Imgproc.rectangle(morph, new Rect(0, 0, 320, 80), new Scalar(255,0,0));
    Imgproc.rectangle(morph, new Rect(0, 80, 160, 240), new Scalar(255,0,0));
    Imgproc.rectangle(morph, new Rect(160, 80, 320, 240), new Scalar(255,0,0));

    // Find the Center point of largest contour if there is one.. using moments and store it
    if (big_contour != null) {
      Moments M = Imgproc.moments(big_contour);
      cX = (int) (M.get_m10() / M.get_m00());
      cY = (int) (M.get_m01() / M.get_m00());
    }

    // return the morphed and masked image for display.
    return morph;

  }
  ArrayList<MatOfPoint> findContours(Mat input) {
    // A list to store the contours we find
    ArrayList<MatOfPoint> contoursList = new ArrayList<>();

    // look for the contours
    Imgproc.findContours(input, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

    return contoursList;
  }


  // Returns the Largest Contour Prop location (Zone 1, 2, 3)
  // TODO: calculate the 'ZONE' from the point (cX,cY)
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
