package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.BotConstants;


/*
* This program provide the hardware design team a simple way to test four motors and two
* servos.
*/
//@Disabled
@TeleOp (group = "G2 Robot", name = "G2 Motor Servo Test")

// Next line will prevent code from building and showing up on Control Hub

public class G2MotorServoTest extends LinearOpMode {
  private Blinker control_Hub;
  private Gyroscope imu;

  private DcMotor motor1 = null; //slide motor 1
  private DcMotor motor2 = null; //slide motor 2
  private DcMotor motor3 = null; //
  private DcMotor motor4 = null; //

  private Servo pixelHolderServo;
  private Servo wristServo;

  private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {
    //read hardware configurations
    control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
    pixelHolderServo = hardwareMap.get(Servo.class, "Servo1");
    wristServo = hardwareMap.get(Servo.class, "Servo2");
    motor1 = hardwareMap.get(DcMotor.class, "SlideMotor1");
    motor2 = hardwareMap.get(DcMotor.class, "SlideMotor2");
//    motor3 = hardwareMap.get(DcMotor.class, "SlideRotation");
    motor4 = hardwareMap.get(DcMotor.class, "IntakeMotor");

    //define initial values for variables
    double lStickY2; //Gamepad 2 left stick servo1 control
    double rStickY2; //Gamepad 2 right stick servo2 control
    double servo1Position;
    double servo2Position;

    //Initialize motor
    motor1.setPower(0);
    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    motor2.setPower(0);
    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//    motor3.setPower(0);
//    motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    motor4.setPower(0);
    motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    telemetry.addData("Motors","initialized");

    // By setting these values to new Gamepad(), they will default to all
    // boolean values as false and all float values as 0
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    // 0.12 not holding and pixel, 0.00 hold both, 0.2 hold one
    pixelHolderServo.setPosition(0.12);
    servo1Position = pixelHolderServo.getPosition();
    telemetry.addData("Servo 1 Position initialized:", "%.3f", servo1Position);

    wristServo.setPosition(0.76);
    servo2Position = wristServo.getPosition();
    telemetry.addData("Servo 2 Position initialized:", "%.3f", servo2Position);
    telemetry.addData("Push play to start"," ");
    telemetry.update();

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

      telemetry.addData("Left stick gamepad1","controls motor 1 & motor 2");
      telemetry.addData("  -Motor 1 and motor 2 should run the slides"," ");
      telemetry.addData("Right stick gamepad1","controls motor 3");
      telemetry.addData("  - Motor 3 is configured as a GoBilda 5202 motor"," ");
      telemetry.addData("Right&Left trigger gamepad1","controls motor 4");
      telemetry.addData("  - Motor 4 is configured as a REV core hex motor"," ");
      telemetry.addData("Left stick gamepad2","controls servo 1");
      telemetry.addData("Right stick gamepad2","controls servo 2");

      motor1.setPower(gamepad2.left_stick_y);
      motor2.setPower(-gamepad2.left_stick_y);
//      motor3.setPower(gamepad1.right_stick_y);
      motor4.setPower(gamepad2.right_stick_y);
      telemetry.addData(" ", " ");
      telemetry.addData("Motor1&2 power", "%.3f", motor1.getPower());
//      telemetry.addData("Motor3 power", "%.3f", motor3.getPower());
      telemetry.addData("Motor4 power", "%.3f", motor4.getPower());

      lStickY2 = -gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y);
      /*if (lStickY2 > 0) {
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
      }*/
      if (currentGamepad2.b && !previousGamepad2.b && servo2Position < 0.76) {
        servo2Position = wristServo.getPosition() + BotConstants.WRIST_PAN_SERVO_SPEED;
      }
      else if(currentGamepad2.a && !previousGamepad2.a && servo2Position > 0) {
        servo2Position = wristServo.getPosition() - BotConstants.WRIST_PAN_SERVO_SPEED;
      }

      wristServo.setPosition(servo2Position);
      telemetry.addData("Servo1", "Position %.3f", pixelHolderServo.getPosition());

/*      rStickY2 = -gamepad2.right_stick_y * Math.abs(gamepad2.right_stick_y);
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
      }*/

      if (currentGamepad2.y && !previousGamepad2.y && servo1Position < 0.2) {
        servo1Position = pixelHolderServo.getPosition() + BotConstants.WRIST_PAN_SERVO_SPEED;
      }
      else if(currentGamepad2.x && !previousGamepad2.x && servo1Position > 0) {
        servo1Position = pixelHolderServo.getPosition() - BotConstants.WRIST_PAN_SERVO_SPEED;
      }

      if (gamepad2.right_bumper) {
        // release one
        servo1Position = 0.2;
      }
      else if (gamepad2.left_bumper) {
        // release both
        servo1Position = 0.10;
      }
      else if (currentGamepad2.right_trigger != 0 && previousGamepad2.right_trigger == 0) {
        // hold both
        servo1Position = 0;
      }
      pixelHolderServo.setPosition(servo1Position);

      telemetry.addData("Servo2", "Position %.3f", wristServo.getPosition());
      telemetry.update();


    }

  }


}
