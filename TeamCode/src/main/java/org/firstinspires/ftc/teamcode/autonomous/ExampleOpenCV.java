package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled

@Autonomous(group="Testing", name="Robot: OpenCV example")


public class ExampleOpenCV extends LinearOpMode {
    OpenCvWebcam webcam1;
    WebcamName webcamName = null;
    SamplePipeline pipeline;


    @Override
    public void runOpMode() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SamplePipeline();
        webcam1.setPipeline(pipeline);

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
        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("Test", "%2d", pipeline.getAreaCode());
            telemetry.update();

        }

    }

}

class SamplePipeline extends OpenCvPipeline {
    Mat YCbCr = new Mat();
    Mat leftCrop;
    Mat rightCrop;
    Mat topCrop;
    double avgTopFin;
    double avgLeftFin;
    double avgRightFin;
    Mat output = new Mat();
    Scalar rectColor = new Scalar(255,0,0);
    Scalar textColor = new Scalar(0,255,255);
    int areaCode;
    Scalar leftAvg;
    Scalar topAvg;
    Scalar rightAvg;


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
        input.copyTo(output);

        Rect topRect = new Rect(1, 1, 639, 89);
        Rect leftRect = new Rect(1, 90, 319, 239);
        Rect rightRect = new Rect(320, 90, 319, 239);

        //input.copyTo(output);
        Imgproc.rectangle(output, topRect, rectColor, 2);
        Imgproc.rectangle(output, leftRect, rectColor, 2);
        Imgproc.rectangle(output, rightRect, rectColor, 2);

        topCrop = YCbCr.submat(topRect);
        leftCrop = YCbCr.submat(leftRect);
        rightCrop = YCbCr.submat(rightRect);

        Core.extractChannel(topCrop, topCrop, 2);
        Core.extractChannel(leftCrop, leftCrop, 2);
        Core.extractChannel(rightCrop, rightCrop, 2);

        topAvg = Core.mean(topCrop);
        leftAvg = Core.mean(leftCrop);
        rightAvg = Core.mean(rightCrop);

        avgTopFin = topAvg.val[0];
        avgLeftFin = leftAvg.val[0];
        avgRightFin = rightAvg.val[0];


        if ( avgLeftFin > avgRightFin && avgLeftFin > avgTopFin ) {
            areaCode = 1;
            //telemetry.addData("rectangle", "1");
        }
        else if ( avgTopFin > avgLeftFin && avgTopFin > avgRightFin){
            areaCode = 2;
            //telemetry.addData("rectangle", "2");
        }
        else if ( avgRightFin > avgLeftFin && avgRightFin > avgTopFin ){
            areaCode = 3;
        }
        else {
            areaCode = -1;
        }

        Imgproc.putText(output, "" + areaCode + " " + Math.round(avgTopFin) + " " + Math.round(avgLeftFin) + " " + Math.round(avgRightFin), new Point(50, 50), Imgproc.FONT_HERSHEY_COMPLEX, 1, textColor, 1);


        //input.copyTo();
        return output;
    }

    public int getAreaCode()
    {
       return areaCode;
    }
    public double[] getAvgs() {
        return new double[] {avgLeftFin, avgRightFin, avgTopFin};
    }

}