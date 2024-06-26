package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//@Disabled
@TeleOp(group = "testing", name = "camera focus test")
public class CameraFocusControlTest extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    public ElapsedTime elapsedTime = new ElapsedTime();
    @Override
    public void runOpMode() {
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Select the Camera
        if (true) {
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
        builder.addProcessors(aprilTag);


        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){
            sleep(50);
        }
        waitForStart();
        if (isStopRequested()) {
            return;
        }

        // Next Lines get a FOCUSControl and set a FIXED Focus Mode at the distance roughly equal
        // to on Square from April Tag setFocusLength(40.0),  0 is infinity and 250 is closest
        // Focus-Time is now ~ 4ms
        elapsedTime.reset();
        double start_time = elapsedTime.milliseconds();
        double end_time;

        FocusControl focusControl = visionPortal.getCameraControl(FocusControl.class);
        focusControl.setMode(FocusControl.Mode.Fixed);
        focusControl.setFocusLength(40.0);
        end_time = elapsedTime.milliseconds();

        while (opModeIsActive()) {
            telemetry.addData("Focus Length", "%.2f", focusControl.getFocusLength());
            telemetry.addData("start/end", "%.2f/%.2f", start_time, end_time);
            telemetry.update();
        }
    }
}
