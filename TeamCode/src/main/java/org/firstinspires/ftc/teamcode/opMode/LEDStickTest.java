package org.firstinspires.ftc.teamcode.opMode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.testing.QwiicLEDStick;


@TeleOp (group = "testing", name = "LEDStickTest")

public class LEDStickTest extends LinearOpMode {
  private Blinker control_Hub;
  //private ColorSensor colorSensor1;
  private RevColorSensorV3 colorSensor1a;
  //private ColorSensor colorSensor2;
  public ElapsedTime elapsedTime = new ElapsedTime();
  public int previousARGB1;
  public double previousRefresh1 = -1;
  public double average1 = -1;
  public int previousARGB2;
  public double previousRefresh2 = -1;
  public double average2 = -1;
  //Sets color strip to white (Supposedly)
  public QwiicLEDStick ledstrip;


  @Override
  public void runOpMode() {
    //colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
   //colorSensor2 = hardwareMap.get(ColorSensor.class, "colorSensor2");
    telemetry.addData("telemetry Refresh rate", "%d", telemetry.getMsTransmissionInterval());
    telemetry.update();

    ledstrip = hardwareMap.get(QwiicLEDStick.class, "ledstrip");
    ledstrip.turnAllOff();
    ledstrip.setBrightness(1);
    ledstrip.setColor(Color.parseColor("blue"));
    sleep(500);
    ledstrip.setColor(5, Color.parseColor("yellow"));
    elapsedTime.reset();


    waitForStart();
    if (isStopRequested()) {
      return;
    }
    resetRuntime();
    while (opModeIsActive()) {

      if (elapsedTime.milliseconds() >= 500)
      {
        ledstrip.setColor(Color.parseColor("white"));
        ledstrip.setColor(5, Color.parseColor("blue"));
        ledstrip.setColor(0, Color.parseColor("red"));
        elapsedTime.reset();
      }
      telemetry.update();
    }
    sleep(500);
    ledstrip.turnAllOff();
  }
}
