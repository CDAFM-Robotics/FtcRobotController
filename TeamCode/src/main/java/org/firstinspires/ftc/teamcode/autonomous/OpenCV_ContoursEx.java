
package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
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


@TeleOp(name="Robot: OCV_ContoursEx", group="Test")

public class OpenCV_ContoursEx extends LinearOpMode {
  OpenCvWebcam webcam;
  Contours_Extraction pipeline;
  private int zone;

  @Override
  public void runOpMode() {
    // Create camera instance
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
    });

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

  static class Contours_Extraction extends OpenCvPipeline {
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

      // Draw that contour (Filled) A mask buffer
      // TODO: replace 'morph' with empty buffer of same size!
      Imgproc.drawContours(mask, contoursList, maxValIdx, new Scalar(255,255,0), Imgproc.FILLED);


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
}

