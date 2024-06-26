package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp (group = "2023 Screamage", name = "ArmServoDrive 2P")



public class ArmServoDrive2P extends LinearOpMode {
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
    boolean bottomArmServoStatus = false;
    boolean topArmServoStatus = false;
    boolean lBumperDown = false;
    boolean rBumperDown = false;
    double wristPanPos = 0.5;
    double wristPanSpeed = 0.001;


    double servoSetPosition = 0.15; // Initial SETUP position 0.15 (on 0-1 scale) install first notch where jaws don't touch
    double bottomServoClose =0.10; // (close to touch)
    double bottomServoOpen =0.30;  // (old open distance)
    double topServoClose = 0.10;
    double topServoOpen = 0.30;

    double slow_mode = 0.75;

    telemetry.addData("Status", "Initializing...");
    telemetry.update();

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
    wristPanServo.setPosition(0.5);


    //initialize both hand servos
    // Reverse Top Servo
    topArmServo.setDirection(Servo.Direction.REVERSE);


    telemetry.addData("Status", "Initialized psh");
    telemetry.update();

    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      rTrigger = gamepad2.right_trigger;
      lTrigger = gamepad2.left_trigger;
      lBumper = gamepad2.left_bumper;
      rBumper = gamepad2.right_bumper;

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
      telemetry.update();

      wristPanServo.setPosition(wristPanPos);

      //telemetry.addData("Servos", "Bottom: %.3f, Top: %.3f", lTrigger, rTrigger);

      telemetry.addData("Status", "Running");
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
    telemetry.addData("Wheel Powers", "Wheel1: %.2f, Wheel2: %.2f, Wheel3:%.2f, Wheel 4: %.2f", motor1Power, motor2Power, motor3Power, motor4Power);
  }
}
