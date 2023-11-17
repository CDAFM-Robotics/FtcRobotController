package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
* This OpMode is for Installing and Aligning Fingers.  press 'DPAD - UP' and 'DPAD - DOWN' to
* set each finger servo to 0.15, then align fingers to first notch before fully closed
*/

// @Disabled
@TeleOp (group = "Testing", name = "ServoCalibrate")

// Next line will prevent code from building and showing up on Control Hub

public class ServoCalibration extends LinearOpMode {
  private Blinker control_Hub;
  private Servo bottomArmServo;
  private Gyroscope imu;
  private Servo topArmServo;
  private Servo wristPanServo;
  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor motor1 = null; //front left
  private DcMotor motor2 = null; //front right
  private DcMotor motor3 = null; //back left
  private DcMotor motor4 = null; //back right
  private DcMotor armmotor = null;

  @Override
  public void runOpMode() {
    //read hardware configurations
    control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
    bottomArmServo = hardwareMap.get(Servo.class, "bottomArmServo");
    topArmServo = hardwareMap.get(Servo.class, "topArmServo");
    wristPanServo = hardwareMap.get(Servo.class, "wristPanServo");
    motor1 = hardwareMap.get(DcMotor.class, "motor1");
    motor2 = hardwareMap.get(DcMotor.class, "motor2");
    motor3 = hardwareMap.get(DcMotor.class, "motor3");
    motor4 = hardwareMap.get(DcMotor.class, "motor4");
    armmotor = hardwareMap.get(DcMotor.class, "armcontrol");


    //define initial values for variables
    double lTrigger;
    double rTrigger;
    double lStickX;
    double lStickY;
    double rStickX;
    boolean lBumper;
    boolean rBumper;

    boolean dUp, dDn, dLt, dRt;
    boolean dUpDown = false;
    boolean dDnDown = false;
    boolean dLtDown = false;
    boolean dRtDown = false;

    boolean bottomArmServoStatus = false;
    boolean topArmServoStatus = false;
    boolean lBumperDown = false;
    boolean rBumperDown = false;
    double wristPanPos = 0.5;
    double wristFolded = 0.6;
    double wristOpen = 1.0;
    double wristPanSpeed = 0.001;

    double servoSetPosition = 0.15; // Initial SETUP position 0.15 (on 0-1 scale) use first pos where Jaws don't touch at 0.15
    double bottomServoClose =0.10; // 0.01 was good set position
    double bottomServoOpen =0.30;
    double topServoClose = 0.10;
    double topServoOpen = 0.30;



    double slow_mode = 0.75;

    // telemetry.addData("Status", "Initializing...");
    // telemetry.update();

    //Initialized the motors
    motor1.setPower(0);
    motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor2.setPower(0);
    motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor3.setPower(0);
    motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor4.setPower(0);
    motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    //Initialize arm motor
    armmotor.setPower(0);
    armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    //initialize wristPanServo
    // wristPanServo.setPosition(wristFolded);


    //initialize both hand servos

    topArmServo.setDirection(Servo.Direction.REVERSE);
    topArmServo.setPosition(servoSetPosition);
    bottomArmServo.setPosition(servoSetPosition);


    // telemetry.addData("Status", "Initialized psh");
    // telemetry.update();

    // Cam Servo Positions.
    /*    telemetry.addLine(String.format("%.3f", camServo.getPosition()));
    telemetry.update();
    sleep(3000);
    int val = 1;
    while (val == 1)
    {
      camServo.setPosition(BotConstants.CAM_SERVO_SPIKE);
      telemetry.addLine(String.format("strike: %.3f", camServo.getPosition()));
      telemetry.update();
      sleep(3000);
      camServo.setPosition(BotConstants.CAM_SERVO_RIGHT);
      telemetry.addLine(String.format("right: %.3f", camServo.getPosition()));
      telemetry.update();
      sleep(3000);
      camServo.setPosition(BotConstants.CAM_SERVO_REAR);
      telemetry.addLine(String.format("rear: %.3f", camServo.getPosition()));
      telemetry.update();
      sleep(3000);


      val = 1;
    }
*/

    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      rTrigger = gamepad2.right_trigger;
      lTrigger = gamepad2.left_trigger;
      lBumper = gamepad2.left_bumper;
      rBumper = gamepad2.right_bumper;

      dUp = gamepad2.dpad_up;
      dDn = gamepad2.dpad_down;

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


      //dpad  Servo Incremental move
      if (dUp) {
        if (!dUpDown) {
          topArmServo.setPosition(servoSetPosition);
          dUpDown = true;
        }
        else { dUpDown = false; };


      }
      if (dDn) {
        if (!dDnDown) {
          bottomArmServo.setPosition(servoSetPosition);
          dDnDown = true;
        }
        else { dDnDown = false; };

      }



/*      if (bottomArmServoStatus) {
        bottomArmServo.setPosition(bottomServoClose);
      }
      else {
        bottomArmServo.setPosition(bottomServoOpen);
      }
      if (topArmServoStatus) {
        topArmServo.setPosition(topServoClose);
      }
      else {
        topArmServo.setPosition(topServoOpen);
      }
*/

      //arm control
      armmotor.setPower(lTrigger*lTrigger-rTrigger*rTrigger);

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

      double distanceFrom0 = Math.sqrt(Math.pow(lStickX, 2) + Math.pow(lStickY, 2));

      double direction = findDir(lStickX * (1.0/distanceFrom0), lStickY * (1.0/distanceFrom0));

      telemetry.addData("Stick Powers", ":lStickX: %.2f, lStickY: %.2f, RStickX:%.2f", lStickX, lStickY, rStickX);

      setMotorPowers(direction, distanceFrom0, rStickX);

      //wrist control
      if (gamepad2.a) {
        wristPanPos += wristPanSpeed;
        if (wristPanPos > 1) {
          wristPanPos = 1;
        }
      }
      if (gamepad2.b) {
        wristPanPos -= wristPanSpeed;
        if (wristPanPos < 0) {
          wristPanPos = 0;
        }
      }

      telemetry.addData("Wrist Servos", "wristPanPos %.3f", wristPanPos);

      wristPanServo.setPosition(wristPanPos);

      telemetry.addData("Servos", "Bottom: %.2f, Top: %.2f", bottomArmServo.getPosition(), topArmServo.getPosition());
      telemetry.update();

    }
  }
  public double findDir(double x, double y) {
    double temp = Math.toDegrees(Math.asin(y));
    if (y >= 0) {
      if (x >= 0) {
        return temp;
      }
      else {
        return 180 - temp;
      }
    }
    else {
      if (x >= 0) {
        return temp;
      }
      else {
        return -180 - temp;
      }
    }
  }
  public void setMotorPowers(double dir, double dist, double turn) {

    double motor1Power = 0;
    double motor2Power = 0;
    double motor3Power = 0;
    double motor4Power = 0;
    double temp;
    if (dir <= 90 && dir > 0) {
      motor1Power = dist;
      motor4Power = dist;
      temp = dir/45.0 - 1;
      motor2Power = temp * dist;
      motor3Power = temp * dist;
    }
    else if (dir <= 0 && dir > -90) {
      temp = (dir + 90)/45.0 - 1;
      motor1Power = temp * dist;
      motor4Power = temp * dist;
      motor2Power = -dist;
      motor3Power = -dist;
    }
    else if (dir <= -90 && dir > -180) {
      motor1Power = -dist;
      motor4Power = -dist;
      temp = (dir + 180)/45.0 - 1;
      motor2Power = temp * -dist;
      motor3Power = temp * -dist;
    }
    else if (dir <= 180 && dir > 90) {
      motor2Power = dist;
      motor3Power = dist;
      temp = (dir - 90)/45.0 - 1;
      motor1Power = - temp * dist;
      motor4Power = - temp * dist;
    }

    motor1Power += turn;
    motor3Power += turn;

    motor2Power -= turn;
    motor4Power -= turn;

    motor1.setPower(motor1Power);
    motor2.setPower(-motor2Power);
    motor3.setPower(motor3Power);
    motor4.setPower(-motor4Power);
    // telemetry.addData("Wheel Powers", "Wheel1: %.2f, Wheel2: %.2f, Wheel3:%.2f, Wheel 4: %.2f", motor1Power, motor2Power, motor3Power, motor4Power);
  }
}
