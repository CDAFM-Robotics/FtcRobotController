package org.firstinspires.ftc.teamcode.opMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp (group = "testing", name = "HolderServoTest")

public class HolderServoTest extends LinearOpMode {
  private Blinker control_Hub;
  private Servo pixelHolderServo;
  public ElapsedTime elapsedTime = new ElapsedTime();
  private DcMotor intakeMotor = null;
  private ColorSensor colorSensor1;
  private ColorSensor colorSensor2;
  public enum HolderServoState {
    HOLD_0,
    HOLD_1,
    HOLD_2,
    HOLD_1_IN_2
  }
  HolderServoState holderServoState = HolderServoState.HOLD_0;

  private boolean pixelIn1 = false;
  private boolean pixelIn2 = false;
  private boolean pixelPickup = false;
  private boolean pixelSpit = false;
  private boolean drop1 = false;
  private boolean drop2 = false;

  @Override
  public void runOpMode() {
    colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
    colorSensor2 = hardwareMap.get(ColorSensor.class, "colorSensor2");
    intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
    pixelHolderServo = hardwareMap.get(Servo.class, "HoldPixel");
    telemetry.addData("telemetry Refresh rate", "%d", telemetry.getMsTransmissionInterval());
    telemetry.update();

    intakeMotor.setPower(0);
    intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    waitForStart();
    if (isStopRequested()) {
      return;
    }
    resetRuntime();
    while (opModeIsActive()) {
      /*
       * TODO: Green pixel:
       * TODO: Yellow Pixel:
       * purple pixel: 3210-3235, 2080-2160, 3140-3150, 4620-4660
       * White Pixel: 6750-6830, 4610-4700, 8280-8310, 7370-7376
       */
      previousGamepad1.copy(currentGamepad1);
      previousGamepad2.copy(currentGamepad2);

      currentGamepad1.copy(gamepad1);
      currentGamepad2.copy(gamepad2);
      telemetry.addData("ARGB Values", "%d, %d, %d, %d", colorSensor1.alpha(),colorSensor1.red(),colorSensor1.green(),colorSensor1.blue());
      telemetry.addData("raw ARGB value", "%d", colorSensor1.argb());
      telemetry.addData("ARGB Values 2", "%d, %d, %d, %d", colorSensor2.alpha(),colorSensor2.red(),colorSensor2.green(),colorSensor2.blue());
      telemetry.addData("raw ARGB value 2", "%d", colorSensor2.argb());
      pixelIn1 = (colorSensor1.alpha() > 790 || colorSensor1.red() > 750 || colorSensor1.green() > 790 || colorSensor1.blue() > 790);
      pixelIn2 = (colorSensor2.alpha() > 790 || colorSensor2.red() > 750 || colorSensor2.green() > 790 || colorSensor2.blue() > 790);
      drop1 = currentGamepad2.left_bumper;
      drop2 = currentGamepad2.right_bumper;

      switch (holderServoState) {
        case HOLD_0:
          if (currentGamepad2.x && !previousGamepad2.x) {
            pixelPickup = !pixelPickup;
            pixelSpit = false;
          }
          if (currentGamepad2.y && !previousGamepad2.y) {
            pixelSpit = !pixelSpit;
            pixelPickup = false;
          }
          if (pixelPickup) {
            intakeMotor.setPower(-1);
          }
          else if (pixelSpit) {
            intakeMotor.setPower(1);
          }
          else {
            intakeMotor.setPower(0);
          }
          if (pixelIn1) {
            if (pixelIn2) {
              holderServoState = HolderServoState.HOLD_2;
            }
            else {
              holderServoState = HolderServoState.HOLD_1;
            }
          }
          else if (pixelIn2 && !pixelPickup) {
            holderServoState = HolderServoState.HOLD_1_IN_2;
          }
          pixelHolderServo.setPosition(0.12);
          break;

        case HOLD_1:
          if (currentGamepad2.x && !previousGamepad2.x) {
            pixelPickup = !pixelPickup;
            pixelSpit = false;
          }
          if (currentGamepad2.y && !previousGamepad2.y) {
            pixelSpit = !pixelSpit;
            pixelPickup = false;
          }
          if (pixelPickup) {
            intakeMotor.setPower(-1);
          }
          else if (pixelSpit) {
            intakeMotor.setPower(1);
          }
          else {
            intakeMotor.setPower(0);
          }
          if (pixelIn2) {
            holderServoState = HolderServoState.HOLD_2;
          }
          else if (drop1 || drop2) {
            holderServoState = HolderServoState.HOLD_0;
          }
          pixelHolderServo.setPosition(0.20);
          break;

        case HOLD_2:
          pixelPickup = false;
          if (currentGamepad2.x && !previousGamepad2.x) {
            pixelSpit = false;
          }
          if (currentGamepad2.y && !previousGamepad2.y) {
            pixelPickup = false;
          }
          if (pixelSpit) {
            intakeMotor.setPower(1);
          }
          else {
            intakeMotor.setPower(0);
          }
          if (drop1) {
            holderServoState = HolderServoState.HOLD_1;
          }
          else if (drop2) {
            holderServoState = HolderServoState.HOLD_0;
          }
          pixelHolderServo.setPosition(0);
          break;

        case HOLD_1_IN_2:
          if (currentGamepad2.x && !previousGamepad2.x) {
            pixelPickup = !pixelPickup;
            pixelSpit = false;
          }
          if (currentGamepad2.y && !previousGamepad2.y) {
            pixelSpit = !pixelSpit;
            pixelPickup = false;
          }
          if (pixelPickup) {
            intakeMotor.setPower(-1);
            holderServoState = HolderServoState.HOLD_0;
          }
          else if (pixelSpit) {
            intakeMotor.setPower(1);
          }
          else {
            intakeMotor.setPower(0);
          }
          if (drop1 || drop2) {
            holderServoState = HolderServoState.HOLD_0;
          }
          pixelHolderServo.setPosition(0);
          break;

      }
      telemetry.addData("current holder state", holderServoState);
      telemetry.update();
    }
  }
}
