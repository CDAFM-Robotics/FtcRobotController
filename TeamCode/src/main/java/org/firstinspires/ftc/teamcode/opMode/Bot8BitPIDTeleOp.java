package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.BotConstants;
import org.firstinspires.ftc.teamcode.opMode.SimplePIDControl;

/***********************************************************************
* This program is the driver control software for the new robot, 8Bit.
************************************************************************/

// Next line will prevent code from building and showing up on Control Hub
//@Disabled
@TeleOp (group = "8Bit Robot", name = "8Bit Driver PID Control")

public class Bot8BitPIDTeleOp extends LinearOpMode {

  // Define constants that can be adjusted
  private static final double DRIVE_SPEED_FAST = 1.0;    // full speed driving
  private static final double DRIVE_SPEED_SLOW = 0.5;    // half speed driving
  private static final double STRAFE_SPEED_FAST = 0.5;    // Speed when strafe to the correct location to drop pixel
  // 0.12 not holding and pixel, 0.00 hold both, 0.2 hold one
  private static final double HOLD_SERVO_NO_HOLD = 0.1;    // Not holding any pixel position
  private static final double HOLD_SERVO_HOLD_0NE = 0.2;
  private static final double HOLD_SERVO_HOLD_TWO = 0.0;
  private static final double WRIST_CLOSED = 0.792;
  private static final double WRIST_MAX_OPEN = 0;
  private static final double WRIST_SERVO_SPEED = 0.008;

  //define robot
  private Blinker control_Hub;
  private IMU imu;
  //define motors
  private DcMotor slideMotor1 = null; //slide motor 1
  private DcMotor slideMotor2 = null; //slide motor 2
  private DcMotor intakeMotor = null; //
  private DcMotor armRotationMotor = null; //
  private DcMotor frontLeftMotor = null; //front left
  private DcMotor frontRightMotor = null; //front right
  private DcMotor backLeftMotor = null; //back left
  private DcMotor backRightMotor = null; //back right
  //define servos
  private Servo pixelHolderServo;
  private Servo wristServo;
  //define sensors

  // User for any motion requiring a hold time or timeout.
  private ElapsedTime runtime = new ElapsedTime();

  //define initial values for variables
  private double lStickX = 0;
  private double lStickY = 0;
  private double rStickX = 0;
  private double lStickY2 = 0; //Gamepad 2 left stick servo1 control
  private double rStickY2 = 0; //Gamepad 2 right stick servo2 control
  private double holdServoPosition;
  private double wristServoPosition;
  private boolean fieldCentric = false;
  private double driveSpeed = DRIVE_SPEED_FAST;
  private double botHeading = 0;
  private ColorSensor colorSensor1;
  private ColorSensor colorSensor2;
  public enum HolderServoState {
    HOLD_0,
    HOLD_1,
    HOLD_2,
    HOLD_1_IN_2
  }
  HolderServoTest.HolderServoState holderServoState = HolderServoTest.HolderServoState.HOLD_0;

  private boolean pixelIn1 = false;
  private boolean pixelIn2 = false;
  private boolean pixelPickup = false;
  private boolean pixelSpit = false;
  private boolean drop1 = false;
  private boolean drop2 = false;
  private boolean pixelBackout = false;
  private double initTheta = 7;
  private double bInit = (Math.tan(Math.toRadians(initTheta)) * BotConstants.DIST_R * 2)/(Math.sqrt(3)-Math.tan(Math.toRadians(initTheta)));
  private double backdroppos = bInit;
  private int slidePosition = 0;
  private boolean readyHanging = false;
  private boolean isHanging = false;
  @Override
  public void runOpMode() {
    //local variables
    boolean lStickY2Pressed = false;
    double holdTime = 0;
    double kp = 0;
    double kd = 0;
    double ki = 0;
    SimplePIDControl pid = new SimplePIDControl(kp, kd, ki);

    //read hardware configurations
    control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
    imu = hardwareMap.get(IMU.class, "imu");
    // these servos and motors are connected to the control hub
    pixelHolderServo = hardwareMap.get(Servo.class, "HoldPixel");
    wristServo = hardwareMap.get(Servo.class, "RotateWrist");
    slideMotor1 = hardwareMap.get(DcMotor.class, "SlideMotor1");
    slideMotor2 = hardwareMap.get(DcMotor.class, "SlideMotor2");
    armRotationMotor = hardwareMap.get(DcMotor.class, "SlideRotation");
    intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
    // these motors are connected to the extension hub
    frontLeftMotor = hardwareMap.get(DcMotor.class, "FLmotor");
    frontRightMotor = hardwareMap.get(DcMotor.class, "FRmotor");
    backLeftMotor = hardwareMap.get(DcMotor.class, "BLmotor");
    backRightMotor = hardwareMap.get(DcMotor.class, "BRmotor");
    colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
    colorSensor2 = hardwareMap.get(ColorSensor.class, "colorSensor2");

    //Initialize motor
    slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideMotor1.setTargetPosition(0);
    slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    slideMotor1.setPower(0);

//    slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    slideMotor2.setTargetPosition(0);
//    slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    slideMotor2.setPower(0);
    slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    slideMotor2.setPower(0);

    armRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armRotationMotor.setTargetPosition(0);
    armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armRotationMotor.setTargetPosition(-35);
    armRotationMotor.setPower(1);
    sleep(500);
    armRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armRotationMotor.setTargetPosition(0);
    armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armRotationMotor.setPower(0);

    intakeMotor.setPower(0);
    intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    frontLeftMotor.setPower(0);
    frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    frontRightMotor.setPower(0);
    frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    backLeftMotor.setPower(0);
    backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    backRightMotor.setPower(0);
    backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    telemetry.addData("Motors", "initialized");

    //init imu
    //commented out to use the Yaw from Automation
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
    ));
    imu.initialize(parameters);
    imu.resetYaw();

    // initialize servos
    // 0.12 not holding and pixel, 0.00 hold both, 0.2 hold one

    holdServoPosition = pixelHolderServo.getPosition();
    telemetry.addData("pixelHolderServo Position initialized:", "%.3f", holdServoPosition);

    wristServo.setPosition(WRIST_CLOSED);
    wristServoPosition = wristServo.getPosition();
    telemetry.addData("wristServo Position initialized:", "%.3f", wristServoPosition);
    telemetry.addData("Servos", "initialized");

    telemetry.addData("Push play to start", " ");

    // other initialization code goes here
    // By setting these values to new Gamepad(), they will default to all
    // boolean values as false and all float values as 0
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    telemetry.update();

    waitForStart();

    if (isStopRequested()) {
      return;
    }

    while (opModeIsActive()) {

      // this ensure the button presses just recognized once
      previousGamepad1.copy(currentGamepad1);
      previousGamepad2.copy(currentGamepad2);

      currentGamepad1.copy(gamepad1);
      currentGamepad2.copy(gamepad2);

      // push in on the left stick Gamepad1 will toggle between fast and slow driving
      if (currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button) {
        if (driveSpeed == DRIVE_SPEED_FAST) {
          driveSpeed = DRIVE_SPEED_SLOW;
        } else {
          driveSpeed = DRIVE_SPEED_FAST;
        }
      }

      // right bumber will give a temporary speed change
      if (currentGamepad1.right_bumper != previousGamepad1.right_bumper) {
        if (driveSpeed == DRIVE_SPEED_FAST) {
          driveSpeed = DRIVE_SPEED_SLOW;
        } else {
          driveSpeed = DRIVE_SPEED_FAST;
        }
      }

      //If the field centric drive lost direction, push Back button to reset heading to Bot Front
      if (currentGamepad1.back && !previousGamepad1.back) {
        imu.resetYaw();
      }

      // A button toggles between field centric vs. bot centric
      if (currentGamepad1.a && !previousGamepad1.a) {
        fieldCentric = !fieldCentric;

        if (fieldCentric) {
          botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
        else {
          botHeading = 0;
        }
      }

      lStickX = driveSpeed * (gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x));
      lStickY = driveSpeed * (-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y));
      rStickX = driveSpeed * (gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x));

      telemetry.addData("Stick Powers", ":lStickX: %.2f, lStickY: %.2f, RStickX:%.2f", lStickX, lStickY, rStickX);
      telemetry.addData("drive speed", ": %.2f", driveSpeed);
      telemetry.addData("imu", "yaw: %.2f", botHeading);

      // ignore the first 15% of the left stick push.
      // So driving straight or strafing will be easier
      if (rStickX < 0.05) {
        if (lStickX > 0) {
          if (lStickY < (0.15 * lStickX) && lStickY > -0.15 * lStickX)
            lStickY = 0;
          if (lStickY > 15 * lStickX || lStickY < -15 * lStickX)
            lStickX = 0;
        } else if (lStickX < 0) {
          if (lStickY > (0.15 * lStickX) && lStickY < -0.15 * lStickX)
            lStickY = 0;
          if (lStickY < 15 * lStickX || lStickY > -15 * lStickX)
            lStickX = 0;
        }
      }

      setMotorPowers(lStickX, lStickY, rStickX, botHeading);

      /* *****************************************
       * Arm and pixel intake/out take control
       ******************************************/

      // Left stick on gamepade2 controls the drop off
      if (Math.abs(gamepad2.left_stick_y) > 0.05
          && Math.abs(gamepad2.right_stick_y) < 0.05) {
          if ( !lStickY2Pressed ) {
            lStickY2Pressed = true;
          }
          holdTime += runtime.milliseconds();
          lStickY2 = -gamepad2.left_stick_y;

          if (lStickY2 >= 0 ) {
            backdroppos = 940;
           }  else if (lStickY2 < 0 ){
            backdroppos = 500;
          }
//        lStickY2 = -gamepad2.left_stick_y * BotConstants.ARM_SPEED;
//        telemetry.addData("GP2 Left StickY2", "%.5f", lStickY2);
//        backdroppos += lStickY2;
//        telemetry.addData("backdrop position 1", "%.2f", backdroppos);
//
        if (backdroppos < 0) {
          backdroppos = 0;
        } else if (backdroppos > 940) {
          backdroppos = 940;
        }
        telemetry.addData("backdrop position 1", "%.2f", backdroppos);

        if (backdroppos > 250) {
          slidePosition = (int) Math.round(calculateExtensionLength(backdroppos) * BotConstants.SLIDE_COUNTS_PER_MILLIMETER);
        }
        else if (backdroppos > bInit){
          slidePosition = BotConstants.SLIDE_READY_POS;
        }
        else {
          slidePosition = 0;
        }

        if (slidePosition == 0 && Math.abs(slideMotor1.getCurrentPosition()) < 10) {
          slideMotor1.setPower(0);
          slideMotor2.setPower(0);
        }
        else {
          slideMotor1.setTargetPosition(-slidePosition);
          slideMotor2.setTargetPosition(slidePosition);
          slideMotor1.setPower(0.5);
          slideMotor2.setPower(0.5);
        }
        armRotationMotor.setTargetPosition(-(int) Math.round((calculateRotationAngle(backdroppos) - 7) * (BotConstants.ROTATION_COUNTS_PER_DEGREE + 4.5)));
        armRotationMotor.setPower(1);

        if (backdroppos > 380) {
          wristServoPosition = WRIST_CLOSED - (BotConstants.BACKDROP_ANGLE - calculateRotationAngle(backdroppos)) / 270 + 18 / 270;
          wristServo.setPosition(wristServoPosition);
          telemetry.addData("wristServo", "Position %.3f", wristServo.getPosition());
        } else {
          wristServoPosition = WRIST_CLOSED;
          wristServo.setPosition(wristServoPosition);
          telemetry.addData("wristServo", "Position %.3f", wristServo.getPosition());
        }
      } else if (Math.abs(gamepad2.left_stick_y) < 0.05
              && Math.abs(gamepad2.right_stick_y) < 0.05) {
        runtime.reset();
        lStickY2Pressed = false;
      }

      // right stick on gamepad 2 controls pick up
      // rotate the arm to pick up pixels with left stick on gamepad 2
      if (Math.abs(gamepad2.left_stick_y) < 0.05
              && Math.abs(gamepad2.right_stick_y) > 0.05) {
        rStickY2 = -gamepad2.right_stick_y;
        telemetry.addData("GP2 Right StickY2", "%.5f", rStickY2);

        //Make sure the slide is in pick up position
        if ( Math.abs(slideMotor1.getCurrentPosition()) > 5 ) {
            slidePosition = 0;
            slideMotor1.setTargetPosition(-slidePosition);
            slideMotor2.setTargetPosition(slidePosition);
            slideMotor1.setPower(1);
            slideMotor2.setPower(1);
        }
        else {
            slideMotor1.setPower(0);
            slideMotor2.setPower(0);
        }

        if (rStickY2 > 0) {
          armRotationMotor.setTargetPosition(BotConstants.ARM_PIXEL_ROTATE_FLOOR);
          armRotationMotor.setPower(1);
        } else {
          armRotationMotor.setTargetPosition(0);
          armRotationMotor.setPower(1);
        }
      }

      telemetry.addData("backdrop position", "%.2f", backdroppos);
      telemetry.addData("slideMMs", "%.2f", calculateExtensionLength(backdroppos));
      telemetry.addData("slidePosition", "%d", slidePosition);
      telemetry.addData("slideMotor1 encoder", "%d", slideMotor1.getCurrentPosition());
      telemetry.addData("slideMotor2 encoder", "%d", slideMotor2.getCurrentPosition());
      telemetry.addData("slidemotor 1 power", "%.3f", slideMotor1.getPower());
      telemetry.addData("slidemotor 2 power", "%.3f", slideMotor2.getPower());
      telemetry.addData("rotation degrees", "%.2f", calculateRotationAngle(backdroppos));
      telemetry.addData("rotation position", "%d", armRotationMotor.getTargetPosition());
      telemetry.addData("rotation current position", "%d", armRotationMotor.getCurrentPosition());

      // intake and pixel holder control
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
          if (currentGamepad2.right_trigger > 0 && !(previousGamepad2.right_trigger > 0)) {
            pixelPickup = !pixelPickup;
            pixelSpit = false;
          }
          if (currentGamepad2.left_trigger > 0 && !(previousGamepad2.left_trigger > 0)) {
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
              holderServoState = HolderServoTest.HolderServoState.HOLD_2;
            }
            else {
              holderServoState = HolderServoTest.HolderServoState.HOLD_1;
            }
          }
          else if (pixelIn2 && !pixelPickup) {
            holderServoState = HolderServoTest.HolderServoState.HOLD_1_IN_2;
          }
          pixelHolderServo.setPosition(0.12);
          break;

        case HOLD_1:
          if (currentGamepad2.right_trigger > 0 && !(previousGamepad2.right_trigger > 0)) {
            pixelPickup = !pixelPickup;
            pixelSpit = false;
          }
          if (currentGamepad2.left_trigger > 0 && !(previousGamepad2.left_trigger > 0)) {
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
            holderServoState = HolderServoTest.HolderServoState.HOLD_2;
          }
          else if (drop1 || drop2) {
            holderServoState = HolderServoTest.HolderServoState.HOLD_0;
          }
          pixelHolderServo.setPosition(0.20);
          break;

        case HOLD_2:
          pixelPickup = false;
          if (currentGamepad2.left_trigger > 0 && !(previousGamepad2.left_trigger > 0)) {
            pixelSpit = !pixelSpit;
          }
          if (pixelSpit) {
            intakeMotor.setPower(1);
          }
          else {
            intakeMotor.setPower(0);
          }
          if (drop1) {
            holderServoState = HolderServoTest.HolderServoState.HOLD_1;
          }
          else if (drop2) {
            holderServoState = HolderServoTest.HolderServoState.HOLD_0;
          }
          pixelHolderServo.setPosition(0);
          break;

        case HOLD_1_IN_2:
          if (currentGamepad2.right_trigger > 0 && !(previousGamepad2.right_trigger > 0)) {
            pixelPickup = !pixelPickup;
            pixelSpit = false;
          }
          if (currentGamepad2.left_trigger > 0 && !(previousGamepad2.left_trigger > 0)) {
            pixelSpit = !pixelSpit;
            pixelPickup = false;
          }
          if (pixelPickup) {
            intakeMotor.setPower(-1);
            holderServoState = HolderServoTest.HolderServoState.HOLD_0;
          }
          else if (pixelSpit) {
            intakeMotor.setPower(1);
          }
          else {
            intakeMotor.setPower(0);
          }
          if (drop1 || drop2) {
            holderServoState = HolderServoTest.HolderServoState.HOLD_0;
          }
          pixelHolderServo.setPosition(0);
          break;

      }
      telemetry.addData("current holder state", holderServoState);

      if (currentGamepad2.b && !previousGamepad2.b && wristServoPosition < WRIST_CLOSED) {
        wristServoPosition = wristServo.getPosition() + WRIST_SERVO_SPEED;
      } else if (currentGamepad2.a && !previousGamepad2.a && wristServoPosition > WRIST_MAX_OPEN) {
        wristServoPosition = wristServo.getPosition() - WRIST_SERVO_SPEED;
      }

      wristServo.setPosition(wristServoPosition);
      telemetry.addData("wristServo", "Position %.3f", wristServo.getPosition());

      telemetry.addData("pixelHolderServo", "Position %.3f", pixelHolderServo.getPosition());

      // Prepare for hanging
      if (currentGamepad1.x && !previousGamepad1.x) {
        readyHanging = !readyHanging;
        if (readyHanging) {
          //close the drawer if it is open
          slidePosition = BotConstants.HANG_POS_8BIT;
          slideMotor1.setTargetPosition(-slidePosition);
          slideMotor2.setTargetPosition(slidePosition);
          slideMotor1.setPower(1);
          slideMotor2.setPower(1);
          armRotationMotor.setTargetPosition(-(int) (83 * (BotConstants.ROTATION_COUNTS_PER_DEGREE + 4.5)));
        }
        else {
          slidePosition = BotConstants.SLIDE_READY_POS;
          slideMotor1.setTargetPosition(-slidePosition);
          slideMotor2.setTargetPosition(slidePosition);
          slideMotor1.setPower(1);
          slideMotor2.setPower(1);
          armRotationMotor.setTargetPosition(-(int) (83 * (BotConstants.ROTATION_COUNTS_PER_DEGREE + 4.5)));
        }
      }

      // Hang the robot
      if (currentGamepad1.start && !previousGamepad1.start) {
        isHanging = !isHanging;
        if (isHanging) {
          slidePosition = 0;
          slideMotor1.setTargetPosition(-slidePosition);
          slideMotor2.setTargetPosition(slidePosition);
          slideMotor1.setPower(1);
          slideMotor2.setPower(1);
        }
        else {
          slidePosition = BotConstants.HANG_POS_8BIT;
          slideMotor1.setTargetPosition(-slidePosition);
          slideMotor2.setTargetPosition(slidePosition);
          slideMotor1.setPower(1);
          slideMotor2.setPower(1);
        }
      }

      telemetry.update();


    }
  }
  public void setMotorPowers(double x, double y, double rx, double heading) {
    double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
    double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

    // put strafing factors here
    rotX = rotX * 1;
    rotY = rotY * 1;

    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

    double frontLeftPower = (rotY + rotX - rx)/denominator;
    double backLeftPower = (rotY - rotX - rx)/denominator;
    double frontRightPower = (rotY - rotX + rx)/denominator;
    double backRightPower = (rotY + rotX + rx)/denominator;

    frontLeftMotor.setPower(frontLeftPower);
    backLeftMotor.setPower(backLeftPower);
    frontRightMotor.setPower(frontRightPower);
    backRightMotor.setPower(backRightPower);
  }

  public static double calculateRotationAngle(double b) {
    return Math.toDegrees(Math.atan((Math.sqrt(3) * b) / (b + 2 * BotConstants.DIST_R)));
  }
  public static double calculateExtensionLength(double b) {
    double r = Math.sqrt(Math.pow(BotConstants.DIST_R, 2) + (BotConstants.DIST_R * b) + Math.pow(b, 2));
    double x = BotConstants.IN_OUT_THICKNESS / Math.tan(Math.toRadians(60 - calculateRotationAngle(b)));
    r -= x;
    r -= BotConstants.FIRST_SLIDE_LENGTH;
    return r;
  }
}
