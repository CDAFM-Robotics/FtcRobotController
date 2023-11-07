
package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
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


@TeleOp(name="Robot: Vision Portal Test", group="Test")

public class OpenCV_VisionPortal extends LinearOpMode {
  // OpenCvWebcam webcam;
  private VisionPortal visionPortal;
  Contours_Extraction pipeline = new Contours_Extraction();
  private int zone;
  private static final boolean USE_WEBCAM = true;

  @Override
  public void runOpMode() {

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

    // Create camera instance
    /*
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

    // Open async and start streaming inside opened callback
    webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
    {
      @Override
      public void onOpened() {
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        pipeline = new Contours_Extraction();
        webcam.setPipeline(pipeline);
      }

      @Override
      public void onError(int errorCode) {

      }

     */


    // Tell telemetry to update faster than the default 250ms period :)
    telemetry.setMsTransmissionInterval(20);

    waitForStart();

    while (opModeIsActive()) {
      sleep(20);

      // Figure out which zone was detected
      int[] result = pipeline.getResult();



      if (result[1] < 80) {
        telemetry.addData("zone", "2");
        zone = 2;
      }
      else if (result[0] < 320) {
        telemetry.addData("zone", "1");
        zone = 1;
      }
      else {
        telemetry.addData("zone", "3");
        zone = 3;
      }

      telemetry.addLine(String.format("%d,%d",result[0],result[1]));
      telemetry.update();
    }
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

    Scalar rLower1 = new Scalar (0,70,50);
    Scalar rUpper1 = new Scalar (10,255,255);
    Scalar rLower2 = new Scalar (170,70,50);
    Scalar rUpper2 = new Scalar (180,255,255);
    Scalar bLower = new Scalar (98,38,10);
    Scalar bUpper = new Scalar (140,255,255);
    MatOfPoint big_contour = null;
    int cX, cY = 0;
    int[] xyArray = new int[2];

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
      // Code executed on the first frame dispatched into this VisionProcessor
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {

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

      // Draw that contour (Filled) A mask buffer
      // TODO: replace 'morph' with empty buffer of same size!
      Imgproc.drawContours(input, contoursList, maxValIdx, new Scalar(255,255,0), Imgproc.FILLED);


      // Find the Center point of largest contour if there is one.. using moments and store it
      if (big_contour != null) {
        Moments M = Imgproc.moments(big_contour);
        cX = (int) (M.get_m10() / M.get_m00());
        cY = (int) (M.get_m01() / M.get_m00());
      }

      // mask.copyTo(input);
      // return the morphed and masked image for display.
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
}

