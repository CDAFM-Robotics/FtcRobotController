package org.firstinspires.ftc.teamcode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name="Robot: OpenCV example", group="AutoTest")
@Disabled

public class ExampleOpenCV extends LinearOpMode {
    OpenCvWebcam webcam1;
    WebcamName webcamName = null;

    @Override
    public void runOpMode() {
        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new SamplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                webcam1.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addData("Cam status", "Camera Failed!");
            }
        });

    }

}

class SamplePipeline extends OpenCvPipeline {
    Mat YCbCr = new Mat();
    Mat topRect;
    Mat bottomLeftRect;
    Mat bottomRightRect;
    double avgTopFin;
    double avgBottomLeftFin;
    double avgBottomRightFin;
    Mat output = new Mat();
    Scalar rectColor = new Scalar(255,0,0);

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
        //telemetry.addLine("pipeline running");

        Rect leftRect = new Rect(1, 1, 319, 359);
        Rect rightRect = new Rect(320, 1, 319, 359);

        //input.copyTo();
        return output;
    }


}