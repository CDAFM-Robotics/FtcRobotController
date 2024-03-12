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
  private ColorSensor colorSensor1;
  private ColorSensor colorSensor2;
  public ElapsedTime elapsedTime = new ElapsedTime();
  public int previousARGB1;
  public double previousRefresh1 = -1;
  public double average1 = -1;
  public int previousARGB2;
  public double previousRefresh2 = -1;
  public double average2 = -1;
  @Override
  public void runOpMode() {
    colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
    colorSensor2 = hardwareMap.get(ColorSensor.class, "colorSensor2");
    telemetry.addData("telemetry Refresh rate", "%d", telemetry.getMsTransmissionInterval());
    telemetry.update();

    waitForStart();
    if (isStopRequested()) {
      return;
    }
    resetRuntime();
    while (opModeIsActive()) {
      if (previousARGB1 != colorSensor1.argb()) {
        if (previousRefresh1 != -1) {
          average1 += previousRefresh1;
          average1 /= 2;
        }
        previousRefresh1 = getRuntime();
        resetRuntime();
        previousARGB1 = colorSensor1.argb();
      }
      if (previousARGB2 != colorSensor2.argb()) {
        if (previousRefresh2 != -1) {
          average2 += previousRefresh2;
          average2 /= 2;
        }
        previousRefresh2 = getRuntime();
        resetRuntime();
        previousARGB2 = colorSensor2.argb();
      }
      telemetry.addData("average refresh", "%.2f", average1);
      telemetry.addData("average refresh 2", "%.2f", average2);
      telemetry.addData("time sicne refresh", "%.2f", getRuntime());
      /*
       * TODO: Green pixel:
       * TODO: Yellow Pixel:
       * purple pixel: 3210-3235, 2080-2160, 3140-3150, 4620-4660
       * White Pixel: 6750-6830, 4610-4700, 8280-8310, 7370-7376
       */
      telemetry.addData("ARGB Values", "%d, %d, %d, %d", colorSensor1.alpha(),colorSensor1.red(),colorSensor1.green(),colorSensor1.blue());
      telemetry.addData("raw ARGB value", "%d", colorSensor1.argb());
      telemetry.addData("ARGB Values 2", "%d, %d, %d, %d", colorSensor2.alpha(),colorSensor2.red(),colorSensor2.green(),colorSensor2.blue());
      telemetry.addData("raw ARGB value 2", "%d", colorSensor2.argb());
      if (colorSensor1.alpha() > 790 || colorSensor1.red() > 750 || colorSensor1.green() > 790 || colorSensor1.blue() > 790) {
        telemetry.addLine("pixel detected in 1");
      }
      if (colorSensor2.alpha() > 790 || colorSensor2.red() > 750 || colorSensor2.green() > 790 || colorSensor2.blue() > 790) {
        telemetry.addLine("pixel detected in 2");
      }

      telemetry.update();
    }
  }
}
