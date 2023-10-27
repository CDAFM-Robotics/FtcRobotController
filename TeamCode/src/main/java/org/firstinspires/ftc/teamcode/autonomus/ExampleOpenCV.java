package org.firstinspires.ftc.teamcode.autonomus;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.provider.ContactsContract;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp(name="Robot: OpenCV example", group="AutoTest")

public class ExampleOpenCV extends LinearOpMode {
    OpenCvWebcam webcam1;
    WebcamName webcamName = null;
    public static int JW_GLOBAL=-1;
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
        while (true) {
            telemetry.addData("Test", "%2d", pipeline.getAreaCode());
        }
        //waitForStart();


    }

}

class SamplePipeline extends OpenCvPipeline {
    Mat YCbCr = new Mat();
    Mat leftCrop;
    Mat rightCrop;
    Mat topRect;
    Mat bottomLeftRect;
    Mat bottomRightRect;
    double avgTopFin;
    double avgBottomLeftFin;
    double avgBottomRightFin;
    Mat output = new Mat();
    Scalar rectColor = new Scalar(255,0,0);
    int areaCode;


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);

        Rect leftRect = new Rect(1, 1, 319, 359);
        Rect rightRect = new Rect(320, 1, 319, 359);

        input.copyTo(output);
        Imgproc.rectangle(output, leftRect, rectColor, 2);
        Imgproc.rectangle(output, rightRect, rectColor, 2);

        leftCrop = YCbCr.submat(leftRect);
        rightCrop = YCbCr.submat(rightRect);

        Core.extractChannel(leftCrop, leftCrop, 2);
        Core.extractChannel(rightCrop, rightCrop, 2);

        Scalar leftAvg = Core.mean(leftCrop);
        Scalar rightAvg = Core.mean(rightCrop);

        avgBottomLeftFin = leftAvg.val[0];
        avgBottomRightFin = rightAvg.val[0];


        if (avgBottomLeftFin > avgBottomRightFin) {
            ExampleOpenCV.JW_GLOBAL = 1;
            areaCode = 1;
            //telemetry.addData("rectangle", "1");
        }
        else {
            ExampleOpenCV.JW_GLOBAL = 2;
            areaCode = 2;
            //telemetry.addData("rectangle", "2");
        }



        //input.copyTo();
        return output;
    }

    public int getAreaCode()
    {
       return areaCode;
    }

}