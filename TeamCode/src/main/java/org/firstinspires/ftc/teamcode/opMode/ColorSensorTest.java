package org.firstinspires.ftc.teamcode.opMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp (group = "testing", name = "colorSensorTest")

public class ColorSensorTest extends LinearOpMode {
  private Blinker control_Hub;
  private ColorSensor colorSensor;
  public ElapsedTime elapsedTime = new ElapsedTime();
  public int previousARGB;
  public double previousRefresh = -1;
  public double average = -1;
  @Override
  public void runOpMode() {
    colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    telemetry.addData("telemetry Refresh rate", "%d", telemetry.getMsTransmissionInterval());
    telemetry.update();

    waitForStart();
    if (isStopRequested()) {
      return;
    }
    resetRuntime();
    while (opModeIsActive()) {
      if (previousARGB != colorSensor.argb()) {
        if (previousRefresh != -1) {
          average += previousRefresh;
          average /= 2;
        }
        previousRefresh = getRuntime();
        resetRuntime();
        previousARGB = colorSensor.argb();
      }
      telemetry.addData("average refresh", "%.2f", average);
      telemetry.addData("time sicne refresh", "%.2f", getRuntime());
      /*
       * TODO: Green pixel:
       * TODO: Yellow Pixel:
       * purple pixel: 3210-3235, 2080-2160, 3140-3150, 4620-4660
       * White Pixel: 6750-6830, 4610-4700, 8280-8310, 7370-7376
       */
      telemetry.addData("ARGB Values", "%d, %d, %d, %d", colorSensor.alpha(),colorSensor.red(),colorSensor.green(),colorSensor.blue());
      telemetry.addData("raw ARGB value", "%d", colorSensor.argb());
      telemetry.update();
    }
  }
}
