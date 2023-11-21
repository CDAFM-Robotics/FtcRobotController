package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.BotConstants;

@Disabled
@TeleOp (group = "Competition", name = "Field Centric 1P")

// Next line will prevent code from building and showing up on Control Hub
// @Disabled
// comment

public class FieldCentric1P extends LinearOpMode {
  private Blinker control_Hub;
  private Servo bottomArmServo;
  private IMU imu;
  private Servo topArmServo;
  private Servo wristPanServo;
  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor frontLeftMotor = null; //front left
  private DcMotor frontRightMotor = null; //front right
  private DcMotor backLeftMotor = null; //back left
  private DcMotor backRightMotor = null; //back right
  private DcMotor armmotor = null;
  private Servo droneServo = null;

  @Override
  public void runOpMode() {
    //read hardware configurations
    control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
    bottomArmServo = hardwareMap.get(Servo.class, "bottomArmServo");
    topArmServo = hardwareMap.get(Servo.class, "topArmServo");
    wristPanServo = hardwareMap.get(Servo.class, "wristPanServo");
    frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
    frontRightMotor = hardwareMap.get(DcMotor.class, "motor2");
    backLeftMotor = hardwareMap.get(DcMotor.class, "motor3");
    backRightMotor = hardwareMap.get(DcMotor.class, "motor4");
    armmotor = hardwareMap.get(DcMotor.class, "armcontrol");
    droneServo = hardwareMap.get(Servo.class, "droneServo");

    //define initial values for variables
    double lTrigger;
    double rTrigger;
    double lStickX;
    double lStickY;
    double rStickX;
    boolean lBumper;
    boolean rBumper;
    boolean bottomArmServoStatus = false;
    boolean topArmServoStatus = false;
    boolean lBumperDown = false;
    boolean rBumperDown = false;
    double wristPanPos = 0;
    double botHeading = 0;

    double slow_mode = BotConstants.DRIVE_SLOW_MODE;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();


    telemetry.addData("Status", "Initializing...");
    telemetry.update();

    //Initialize the motors
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

    //Initialize arm motor
    armmotor.setPower(0);
    armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    //initialize wristPanServo and drone servo
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FOLDED);
    wristPanPos = BotConstants.WRIST_PAN_SERVO_FOLDED;
    droneServo.setPosition(BotConstants.DRONE_POSITION_ARMED);

    //initialize both hand servos
    // Reverse Top Servo
    topArmServo.setDirection(Servo.Direction.REVERSE);
    
    //init imu

    imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
      RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
      RevHubOrientationOnRobot.UsbFacingDirection.UP
    ));
    imu.initialize(parameters);

    imu.resetYaw();

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    if (isStopRequested()) {
      return;
    }

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      previousGamepad1.copy(currentGamepad1);
      previousGamepad2.copy(currentGamepad2);

      currentGamepad1.copy(gamepad1);
      currentGamepad2.copy(gamepad2);


      rTrigger = gamepad1.right_trigger;
      lTrigger = gamepad1.left_trigger;
      lBumper = gamepad1.left_bumper;
      rBumper = gamepad1.right_bumper;

      //LR bumper
      if (lBumper) {
        if (!lBumperDown) {
          bottomArmServoStatus = !bottomArmServoStatus;
          lBumperDown = true;
          if (bottomArmServoStatus)
            bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
          else
            bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
        }
      }
      else { lBumperDown = false;}
      if (rBumper) {
        if (!rBumperDown) {
          topArmServoStatus = !topArmServoStatus;
          rBumperDown = true;
          if (topArmServoStatus)
            topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);
          else
            topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
        }
      }
      else {rBumperDown = false;}

      //arm control
      armmotor.setPower(lTrigger*lTrigger-rTrigger*rTrigger);

      //mecanum drive train
      // TODO: Assign Slow_mode a toggle control (1) or (botconstants.slow_mode)
      lStickX = slow_mode*(gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x));
      lStickY = slow_mode*(-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y));
      rStickX = slow_mode*(gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x));

      telemetry.addData("Stick Powers", ":lStickX: %.2f, lStickY: %.2f, RStickX:%.2f", lStickX, lStickY, rStickX);


      if (currentGamepad1.back && !previousGamepad1.back) {
        imu.resetYaw();
      }
      botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
      telemetry.addData("imu", "yaw: %.2f", botHeading);

      setMotorPowers(lStickX, lStickY, rStickX, botHeading);

      if (currentGamepad1.y && !previousGamepad1.y) {
        if (droneServo.getPosition() == BotConstants.DRONE_POSITION_ARMED)
          droneServo.setPosition(BotConstants.DRONE_POSITION_LAUNCH);
        else
          droneServo.setPosition(BotConstants.DRONE_POSITION_ARMED);
      }
      telemetry.addData("Drone Servos", "%.3f", droneServo.getPosition());

      //wrist control
      if (gamepad1.a) {
        wristPanPos += BotConstants.WRIST_PAN_SERVO_SPEED;
        if (wristPanPos > BotConstants.WRIST_PAN_SERVO_FOLDED) {
          wristPanPos = BotConstants.WRIST_PAN_SERVO_FOLDED;
        }
      }
      if (gamepad1.b) {
        wristPanPos -= BotConstants.WRIST_PAN_SERVO_SPEED;
        if (wristPanPos < BotConstants.WRIST_PAN_SERVO_FLOOR) {
          wristPanPos = BotConstants.WRIST_PAN_SERVO_FLOOR;
        }
      }

      telemetry.addData("Wrist Servos", "wristPanPos %.3f", wristPanPos);
      telemetry.update();

      wristPanServo.setPosition(wristPanPos);

    }
  }
  public void setMotorPowers(double x, double y, double rx, double heading) {
    double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
    double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

    // put strafing factors here
    rotX = rotX * 1;
    rotY = rotY * 1;

    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

    double frontLeftPower = (rotY + rotX + rx)/denominator;
    double backLeftPower = (rotY - rotX + rx)/denominator;
    double frontRightPower = (rotY - rotX - rx)/denominator;
    double backRightPower = (rotY + rotX - rx)/denominator;

    frontLeftMotor.setPower(frontLeftPower);
    backLeftMotor.setPower(backLeftPower);
    frontRightMotor.setPower(frontRightPower);
    backRightMotor.setPower(backRightPower);
  }
}
