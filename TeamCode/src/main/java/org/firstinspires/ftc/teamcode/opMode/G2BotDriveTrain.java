package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.BotConstants;

import org.firstinspires.ftc.teamcode.common.BotConstants;

/*
* This program provide the hardware design team a simple way to test four motors and two
* servos.
*/
@Disabled
@TeleOp (group = "8Bit Robot", name = "8Bit drive train")

// Next line will prevent code from building and showing up on Control Hub

public class G2BotDriveTrain extends LinearOpMode {
  private Blinker control_Hub;
  private IMU imu;

  private DcMotor motor1 = null; //slide motor 1
  private DcMotor motor2 = null; //slide motor 2
  private DcMotor motor3 = null; //
  private DcMotor motor4 = null; //
  private DcMotor frontLeftMotor = null; //front left
  private DcMotor frontRightMotor = null; //front right
  private DcMotor backLeftMotor = null; //back left
  private DcMotor backRightMotor = null; //back right

  private Servo servo1;
  private Servo servo2;

  private ElapsedTime runtime = new ElapsedTime();
  double botHeading = 0;
  @Override
  public void runOpMode() {
    //read hardware configurations
    control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
//    servo1 = hardwareMap.get(Servo.class, "Servo1");
//    servo2 = hardwareMap.get(Servo.class, "Servo2");
//    motor1 = hardwareMap.get(DcMotor.class, "SlideMotor1");
//    motor2 = hardwareMap.get(DcMotor.class, "SlideMotor2");
//    motor3 = hardwareMap.get(DcMotor.class, "SlideRotation");
//    motor4 = hardwareMap.get(DcMotor.class, "IntakeMotor");
    frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
    frontRightMotor = hardwareMap.get(DcMotor.class, "motor2");
    backLeftMotor = hardwareMap.get(DcMotor.class, "motor3");
    backRightMotor = hardwareMap.get(DcMotor.class, "motor4");
    imu = hardwareMap.get(IMU.class, "imu");

    //define initial values for variables
    double lStickX;
    double lStickY;
    double rStickX;
    double lStickY2; //Gamepad 2 left stick servo1 control
    double rStickY2; //Gamepad 2 right stick servo2 control
    double servo1Position;
    double servo2Position;
    boolean fieldCentric = true;
    double driveSpeed = BotConstants.DRIVE_NORMAL_MODE;

    //Initialize motor
/*    motor1.setPower(0);
    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    motor2.setPower(0);
    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    motor3.setPower(0);
    motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    motor4.setPower(0);
    motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

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

    telemetry.addData("Motors","initialized");

    // By setting these values to new Gamepad(), they will default to all
    // boolean values as false and all float values as 0
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    //init imu
    //commented out to use the Yaw from Automation
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
      RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
      RevHubOrientationOnRobot.UsbFacingDirection.UP
    ));
    imu.initialize(parameters);

    imu.resetYaw();

/*    servo1.setPosition(0);
    servo1Position = servo1.getPosition();
    telemetry.addData("Servo 1 Position initialized:", "%.3f", servo1Position);

    servo2.setPosition(0);
    servo2Position = servo2.getPosition();
    telemetry.addData("Servo 2 Position initialized:", "%.3f", servo2Position);
    telemetry.addData("Push play to start"," ");
    telemetry.update();*/

    // other initialization code goes here

    waitForStart();

    if (isStopRequested()) {
      return;
    }

    while (opModeIsActive()) {

      previousGamepad1.copy(currentGamepad1);
      previousGamepad2.copy(currentGamepad2);

      currentGamepad1.copy(gamepad1);
      currentGamepad2.copy(gamepad2);

      if (currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button) {
        if (driveSpeed == BotConstants.DRIVE_NORMAL_MODE) {
          driveSpeed = BotConstants.DRIVE_SLOW_MODE;
        }
        else {
          driveSpeed = BotConstants.DRIVE_NORMAL_MODE;
        }
      }
      if (currentGamepad1.right_bumper != previousGamepad1.right_bumper) {
        if (driveSpeed == BotConstants.DRIVE_NORMAL_MODE) {
          driveSpeed = BotConstants.DRIVE_SLOW_MODE;
        }
        else {
          driveSpeed = BotConstants.DRIVE_NORMAL_MODE;
        }
      }

      lStickX = driveSpeed * (gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x));
      lStickY = driveSpeed * (-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y));
      rStickX = driveSpeed * (gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x));

      //If the field centric drive lost direction, push Back button to reset heading to Bot Front
      if (currentGamepad1.back && !previousGamepad1.back) {
        imu.resetYaw();
      }

      if (currentGamepad1.a && !previousGamepad1.a) {
        fieldCentric = !fieldCentric;
      }

      if (fieldCentric) {
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addLine("field centric");
      }
      else {
        botHeading = 0;
        telemetry.addLine("bot centric");
      }

      telemetry.addData("Stick Powers", ":lStickX: %.2f, lStickY: %.2f, RStickX:%.2f", lStickX, lStickY, rStickX);
      telemetry.addData("drive speed", ": %.2f", driveSpeed);
      telemetry.addData("imu", "yaw: %.2f", botHeading);

      setMotorPowers(lStickX, lStickY, rStickX, botHeading);

/*      lStickY2 = -gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y);
      if (lStickY2 > 0) {
        servo1Position += BotConstants.WRIST_PAN_SERVO_SPEED;
        if ( servo1Position > 1 )
          servo1Position = 1;
      }
      else if (lStickY2 < 0 ) {
        servo1Position -= BotConstants.WRIST_PAN_SERVO_SPEED;
        if ( servo1Position < 0 )
          servo1Position = 0;
      }
      else {
        //do nothing when the stick is at 0 position
      }
 //     servo1.setPosition(servo1Position);
      telemetry.addData("Servo1", "Position %.3f", servo1.getPosition());

      rStickY2 = -gamepad2.right_stick_y * Math.abs(gamepad2.right_stick_y);
      if (rStickY2 > 0) {
        servo2Position += BotConstants.WRIST_PAN_SERVO_SPEED;
        if ( servo2Position > 1 )
          servo2Position = 1;
      }
      else if (rStickY2 < 0 ) {
        servo2Position -= BotConstants.WRIST_PAN_SERVO_SPEED;
        if ( servo2Position < 0 )
          servo2Position = 0;

      }
      else {
        //do nothing when the stick is at 0 position
      }
 //     servo2.setPosition(servo2Position);
      telemetry.addData("Servo2", "Position %.3f", servo2.getPosition());*/
      telemetry.update();
    }
  }

  public void setMotorPowers(double x, double y, double rx, double heading) {

    if (x > 0) {
      if ( y < 0.15 * x && y > -0.15 * x )
        y = 0;
      if ( y > 15 * x || y < -15 * x )
        x = 0;
    }

    if ( x < 0 ) {
      if ( y > 0.15 * x && y < -0.15 * x )
        y = 0;
      if ( y < 15 * x || y > -15 * x )
        x = 0;
    }

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
}
