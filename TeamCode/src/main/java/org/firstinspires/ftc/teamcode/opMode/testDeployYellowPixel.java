package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (group = "Testing", name = "test Deploy Yellow Pixel")

// Next line will prevent code from building and showing up on Control Hub
// @Disabled
// comment

public class testDeployYellowPixel extends LinearOpMode {
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

  private int ARM_DRIVE_POSITION = 400;
  private int ARM_DEPLOY_POSITION = 7718;
  private double WRIST_DEPLOY_POSITION = 0.323;
  private double wristPanServoFolded = 0.6;
  private int SLEEP_DEPLOY_ARM = 6000;
  private int SLEEP_DEPLOY_WRIST = 1500;
  private double DEPLOY_ARM_SPEED = 0.5;

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
    double wristPanServoFloor = 0;
    double wristPanPos = wristPanServoFolded;
    double wristPanSpeed = 0.001;
    double botHeading = 0;
    double dronePositionArmed=0;
    double dronePositionLaunch=0.25; //Early Guess

    double servoSetPosition = 0.15; // Initial SETUP position 0.15 (on 0-1 scale) install first notch where jaws don't touch
    double bottomServoClose = 0.10; // (close to touch)
    double bottomServoOpen = 0.30;  // (old open distance)
    double topServoClose = 0.10;
    double topServoOpen = 0.30;

    double slow_mode = 1;

    telemetry.addData("Status", "Initializing...");
    telemetry.update();

    //Initialized the motors
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
    armmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    armmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    telemetry.addData("Arm Motor Position", "Arm Motor decoder: %d", armmotor.getCurrentPosition());


    //initialize wristPanServo and drone servo
    wristPanServo.setPosition(wristPanServoFolded);

    droneServo.setPosition(dronePositionArmed);

    //initialize both hand servos
    // Reverse Top Servo
    topArmServo.setDirection(Servo.Direction.REVERSE);
    bottomArmServo.setPosition(bottomServoClose);
    topArmServo.setPosition(topServoClose);


    //init imu

    imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
      RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
      RevHubOrientationOnRobot.UsbFacingDirection.UP
    ));
    imu.initialize(parameters);

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    if (isStopRequested()) {
      return;
    }

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      /*rTrigger = gamepad1.right_trigger;
      lTrigger = gamepad1.left_trigger;
      lBumper = gamepad1.left_bumper;
      rBumper = gamepad1.right_bumper;

      //LR bumper
      if (lBumper) {
        if (!lBumperDown) {
          bottomArmServoStatus = !bottomArmServoStatus;
          lBumperDown = true;
          if (bottomArmServoStatus)
            bottomArmServo.setPosition(bottomServoOpen);
          else
            bottomArmServo.setPosition(bottomServoClose);
        }
      }
      else { lBumperDown = false;}
      if (rBumper) {
        if (!rBumperDown) {
          topArmServoStatus = !topArmServoStatus;
          rBumperDown = true;
          if (topArmServoStatus)
            topArmServo.setPosition(topServoOpen);
          else
            topArmServo.setPosition(topServoClose);
        }
      }
      else {rBumperDown = false;}
/*
      //arm control
      armmotor.setPower(lTrigger*lTrigger-rTrigger*rTrigger);
      telemetry.addData("Arm Motor Position", "Arm Motor decoder %d", armmotor.getCurrentPosition());

      //bottomArmServo.setPosition((1-lTrigger)*0.15);
      //topArmServo.setPosition(rTrigger*0.15+0.45);

      //close:
      //bottomArmServo.setPosition(0);
      //topArmServo.setPosition(0.6);

      //open:
      //bottomArmServo.setPosition(0.15);
      //topArmServo.setPosition(0.45);

      //mecanum drive train
      lStickX = slow_mode*(gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x));
      lStickY = slow_mode*(-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y));
      rStickX = slow_mode*(gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x));

      if (gamepad1.back) {
        imu.resetYaw();
      }

      botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

      telemetry.addData("Stick Powers", ":lStickX: %.2f, lStickY: %.2f, RStickX:%.2f", lStickX, lStickY, rStickX);

      setMotorPowers(lStickX, lStickY, rStickX, botHeading);

      //wrist control
      if (gamepad1.a) {
        wristPanPos += wristPanSpeed;
        if (wristPanPos > 1) {
          wristPanPos = 1;
        }
        wristPanServo.setPosition(wristPanPos);
      }
      if (gamepad1.b) {
        wristPanPos -= wristPanSpeed;
        if (wristPanPos < 0) {
          wristPanPos = 0;
        }
        wristPanServo.setPosition(wristPanPos);
      }

      telemetry.addData("imu", "yaw: %.2f", botHeading);

      telemetry.addData("Wrist Servos", "wristPanPos %.3f", wristPanPos);
      telemetry.update();


      //telemetry.addData("Servos", "Bottom: %.3f, Top: %.3f", lTrigger, rTrigger);
*/
      telemetry.addData("Arm Motor Position", "Arm Motor decoder: %d", armmotor.getCurrentPosition());
      telemetry.addData("Wrist Servos", "wristPanPos %.3f", wristPanPos);

      if(gamepad1.x){
        setArmDeployPosition();
        sleep(SLEEP_DEPLOY_ARM);
        setWristDeployPosition();
        sleep(SLEEP_DEPLOY_WRIST);
        bottomArmServo.setPosition(bottomServoOpen);
        topArmServo.setPosition(topServoOpen);
      }

      if(gamepad1.y){
        bottomArmServo.setPosition(bottomServoClose);
        topArmServo.setPosition(topServoClose);
        setArmDrivePosition();
        setWristFoldPosition();
      }

      telemetry.addData("Arm Motor Position", "Arm Motor decoder: %d", armmotor.getCurrentPosition());
      telemetry.addData("Wrist Servos", "wristPanPos %.3f", wristPanPos);
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

    double frontLeftPower = (rotY + rotX + rx)/denominator;
    double backLeftPower = (rotY - rotX + rx)/denominator;
    double frontRightPower = (rotY - rotX - rx)/denominator;
    double backRightPower = (rotY + rotX - rx)/denominator;

    frontLeftMotor.setPower(frontLeftPower);
    backLeftMotor.setPower(backLeftPower);
    frontRightMotor.setPower(frontRightPower);
    backRightMotor.setPower(backRightPower);
  }

  public void setArmDrivePosition() {
    armmotor.setTargetPosition(ARM_DRIVE_POSITION);

    if (armmotor.isBusy()){
      armmotor.setPower(DEPLOY_ARM_SPEED);
    }
    else {
      armmotor.setPower(0);
    }
  }

  public void setArmDeployPosition() {
    armmotor.setTargetPosition(ARM_DEPLOY_POSITION);

    if (armmotor.isBusy()){
      armmotor.setPower(DEPLOY_ARM_SPEED);
    }
    else {
      armmotor.setPower(0);
    }
  }

  public void setWristFoldPosition() {
    wristPanServo.setPosition(wristPanServoFolded);
  }
  public void setWristDeployPosition() {
    wristPanServo.setPosition(WRIST_DEPLOY_POSITION);
 }

}
