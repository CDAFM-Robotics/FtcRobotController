package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
* This OpMode is for Installing and Aligning Fingers.  press 'DPAD - UP' and 'DPAD - DOWN' to
* set each finger servo to 0.15, then align fingers to first notch before fully closed
*/

// @Disabled
@TeleOp (group = "Testing", name = "ServoSetupTest_Hook")

// Next line will prevent code from building and showing up on Control Hub

public class ServoSetupTest_Hook extends LinearOpMode {
  private Blinker control_Hub;
  private Servo bottomArmServo;
  private Gyroscope imu;
  private Servo topArmServo;
  private Servo wristPanServo;
  private Servo hookServo;
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
    hookServo = hardwareMap.get(Servo.class, "hookServo");
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


    //Initialize arm motor
    armmotor.setPower(0);
    armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // By setting these values to new Gamepad(), they will default to all
    // boolean values as false and all float values as 0
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    hookServo.setPosition(0);
    double temp = hookServo.getPosition();
    telemetry.addData("hook Servo Position:", "%.3f", temp);
    telemetry.update();

    // other initialization code goes here

    while (opModeIsActive()) {
      previousGamepad1.copy(currentGamepad1);
      previousGamepad2.copy(currentGamepad2);

      currentGamepad1.copy(gamepad1);
      currentGamepad2.copy(gamepad2);

      if (currentGamepad1.a && !previousGamepad1.a) {
        hookServo.setPosition(hookServo.getPosition() + 0.1);
        temp = hookServo.getPosition();
        telemetry.addData("hook Servo Position:", "%.3f", temp);
        telemetry.update();
      }

      if (currentGamepad1.b && !previousGamepad1.b) {
        hookServo.setPosition(hookServo.getPosition() - 0.1);
        temp = hookServo.getPosition();
        telemetry.addData("hook Servo Position:", "%.3f", temp);
        telemetry.update();
      }

      //arm control
      // armmotor.setPower(lTrigger*lTrigger-rTrigger*rTrigger);


    }
  }

}
