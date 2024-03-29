package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.BotConstants;

@TeleOp (group = "Competition", name = "Toggle Centric w/ PID")

// Comment next line to prevent code from building and showing up on Control Hub
// @Disabled


public class ToggleCentricPID extends LinearOpMode {
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
  private DcMotor armMotor = null;
  private Servo droneServo = null;
  private Servo hookServo = null;
  double driveSpeed = BotConstants.DRIVE_NORMAL_MODE;
  int armMotorTargetPosition = 0;

  //ALL Common CONSTANTS MOVED TO BotConstants Class

  double botHeading = 0;

  //PID Constants

  double KP = 0.01;
  double KI = 0.005;
  double KD = 0.01;

  // PID Variables

  double error = 0;
  double previousError = 0;
  double integral = 0;
  double derivative = 0;


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
    armMotor = hardwareMap.get(DcMotor.class, "armcontrol");
    droneServo = hardwareMap.get(Servo.class, "droneServo");
    hookServo = hardwareMap.get(Servo.class, "hookServo");
    imu = hardwareMap.get(IMU.class, "imu");

    //define initial values for variables
    double lTrigger;
    double rTrigger;
    double lStickX;
    double lStickY;
    double rStickX;
    double lStickY2; //Gamepad 2 left stick
    double rStickY2; //Gamepad 2 right stick
    boolean left_Stick_Button = false;
    boolean bottomFingerServoOpen = false;
    boolean topFingerServoOpen = false;
    boolean robotHanging = false;
    //Wrist initial position is folded
    double wristPanPos = BotConstants.WRIST_PAN_SERVO_FOLDED;
    // By setting these values to new Gamepad(), they will default to all
    // boolean values as false and all float values as 0
    int previousArmPos;
    boolean dPadPressed = false;
    boolean fieldCentric = false;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();



    PwmControl hookServoPWM = (PwmControl) hookServo;

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

    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // jw test

    //Initialize arm motor
    armMotorTargetPosition = BotConstants.ARM_POS_FLOOR_TELEOP;
    armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    armMotor.setPower(1);

    telemetry.addData("Arm Motor init", "Arm Motor decoder: %d", armMotor.getCurrentPosition());
    telemetry.addData("Arm Motor init", "run mode: %s", armMotor.getMode().toString());


    //initialize wristPanServo and drone servo
    wristPanServo.setPosition(BotConstants.WRIST_PAN_SERVO_FOLDED);
    droneServo.setPosition(BotConstants.DRONE_POSITION_ARMED);
    hookServo.setPosition(BotConstants.HOOK_POS_RETRACT);
    telemetry.addData("Wrist servo Position:", "%f", wristPanServo.getPosition());
    telemetry.addData("Dronw servo Position", "%f", droneServo.getPosition());
    telemetry.addData("Hook servo Position", "%f", hookServo.getPosition());

    //initialize both hand servos
    // Reverse Top Servo
    topArmServo.setDirection(Servo.Direction.REVERSE);
    topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
    bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
    telemetry.addData("top finger servo Position", "%f", topArmServo.getPosition());
    telemetry.addData("bottom finger servo Position", "%f", bottomArmServo.getPosition());

    //init imu
    // commented out to use the Yaw from Automation
    /*IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
      RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
      RevHubOrientationOnRobot.UsbFacingDirection.UP
    ));
    imu.initialize(parameters);

    imu.resetYaw();*/

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

      if (!robotHanging) {
        //Driving control from Gamepad 1
        //mecanum drive train
        // TODO: ADD DRIVE_SLOW_MODE Toggle between 1.0 and BotConstants.DRIVE_SLOW_MODE

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


        lStickX = driveSpeed * gamepad1.left_stick_x;
        lStickY = driveSpeed * -gamepad1.left_stick_y;
        rStickX = driveSpeed * gamepad1.right_stick_x;

        //If the field centric drive lost direction, push Back button to reset heading to Bot Front
        if (currentGamepad1.back && !previousGamepad1.back) {
          IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                  RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                  RevHubOrientationOnRobot.UsbFacingDirection.UP
          ));
          imu.initialize(parameters);

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

        telemetry.addData("Stick Powers", ":lStickX: %.2f, lStickY: %.2f, RStickX:%.2f", lStickX, lStickY, rStickX);
        telemetry.addData("imu", "yaw: %.2f", botHeading);

        setMotorPowers(lStickX, lStickY, rStickX, botHeading);

        lStickY2 = -gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y);

        if (lStickY2 != 0) {
          if (lStickY2 < 0) {
            armMotorTargetPosition = BotConstants.ARM_POS_FLOOR_TELEOP;
          }
          else {
            armMotorTargetPosition = BotConstants.ARM_POS_MAX;
          }
        }
        else if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
          wristPanPos = BotConstants.WRIST_PAN_SERVO_L2_DEPLOY;
          armMotorTargetPosition = BotConstants.ARM_POS_L2_DROP;
        }
        else if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
          wristPanPos = BotConstants.WRIST_PICK_UP;
          armMotorTargetPosition = BotConstants.ARM_POS_FLOOR_TELEOP;
        }
        else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
          wristPanPos = BotConstants.WRIST_PICK_UP;
          armMotorTargetPosition = BotConstants.ARM_POS_2_outof_5;
        }
        else if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
          wristPanPos = BotConstants.WRIST_PAN_SERVO_FOLDED;
          armMotorTargetPosition = BotConstants.ARM_POS_DRIVE;
        }
        else if (currentGamepad2.b && !previousGamepad2.b) {
          wristPanPos = BotConstants.WRIST_PICK_UP;
          armMotorTargetPosition = BotConstants.ARM_POS_2_outof_3;
        }
        else if (currentGamepad2.x && !previousGamepad2.x) {
          wristPanPos = BotConstants.WRIST_PAN_SERVO_MOSAIC;
          armMotorTargetPosition = BotConstants.ARM_POS_L2_MOSAIC;
        }

        // PID Control Code

        previousError = error;
        error = armMotorTargetPosition - armMotor.getCurrentPosition();
        integral += error;
        derivative = error - previousError;

        armMotor.setPower((error * KP) + (integral * KI) + (derivative * KD));

        wristPanServo.setPosition(wristPanPos);


        telemetry.addData("Arm Motor Position", "Arm Motor encoder: %d", armMotor.getCurrentPosition());
        telemetry.addData("Arm Motor Position", "run mode: %s", armMotor.getMode().toString());

        //servo slot 0 & 1 are for the finger controls
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
          if (!topFingerServoOpen) {
            topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);
            topFingerServoOpen = true;
          }
          else {
            topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
            topFingerServoOpen = false;
          }
        }

        if (currentGamepad2.right_trigger > 0 && !(previousGamepad2.right_trigger > 0)) {
          if (!bottomFingerServoOpen) {
            bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
            bottomFingerServoOpen = true;
          }
          else {
            bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
            bottomFingerServoOpen = false;
          }
        }

        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
          if (!topFingerServoOpen) {
            topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_OPEN);
            topFingerServoOpen = true;
            bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_OPEN);
            bottomFingerServoOpen = true;
          } else {
            topArmServo.setPosition(BotConstants.TOP_ARM_SERVO_CLOSE);
            topFingerServoOpen = false;
            bottomArmServo.setPosition(BotConstants.BOTTOM_ARM_SERVO_CLOSE);
            bottomFingerServoOpen = false;
          }
        }
        telemetry.addData("top finger servo Position", "%f", topArmServo.getPosition());
        telemetry.addData("bottom finger servo Position", "%f", bottomArmServo.getPosition());

        //wrist servo is in slot 2
        rStickY2 = -gamepad2.right_stick_y * Math.abs(gamepad2.right_stick_y);
        if (rStickY2 > 0) {
          wristPanPos += BotConstants.WRIST_PAN_SERVO_SPEED;
          if (wristPanPos > BotConstants.WRIST_PAN_SERVO_FOLDED) {
            wristPanPos = BotConstants.WRIST_PAN_SERVO_FOLDED;
          }
        }
        else if (rStickY2 < 0 ) {
          wristPanPos -= BotConstants.WRIST_PAN_SERVO_SPEED;
          if (wristPanPos < BotConstants.WRIST_PAN_SERVO_FLOOR) {
            wristPanPos = BotConstants.WRIST_PAN_SERVO_FLOOR;
          }
        }
        else {
          //do nothing when the stick is at 0 position
        }

        wristPanServo.setPosition(wristPanPos);
        telemetry.addData("Wrist Servos", "wristPanPos %.3f", wristPanServo.getPosition());

        //camera is servo in slot 3. Not implemented in TeleOp

        //airplane launcher servo is in slot 4
        if (currentGamepad2.y && !previousGamepad2.y) {
          if (droneServo.getPosition() == BotConstants.DRONE_POSITION_ARMED)
            droneServo.setPosition(BotConstants.DRONE_POSITION_LAUNCH);
          else
            droneServo.setPosition(BotConstants.DRONE_POSITION_ARMED);
        }
        telemetry.addData("Drone Servos", "%.3f", droneServo.getPosition());

        //hook servo is in slot 5
        if (currentGamepad1.x && !previousGamepad1.x) {
          if (hookServo.getPosition() == BotConstants.HOOK_POS_RETRACT) {
            wristPanPos = BotConstants.WRIST_PAN_SERVO_FOLDED;
            wristPanServo.setPosition(wristPanPos);
            setArmPosition(BotConstants.ARM_POS_HANG, BotConstants.ARM_POWER);
            hookServo.setPosition(BotConstants.HOOK_POS_DEPLOY);
          }
          else
            hookServo.setPosition(BotConstants.HOOK_POS_RETRACT);
        }
        telemetry.addData("Hook servo Position", ":%f", hookServo.getPosition());

        if (currentGamepad1.start && !previousGamepad1.start) {
          // make sure the wrist is in fold position
          wristPanPos = BotConstants.WRIST_PAN_SERVO_FOLDED;
          wristPanServo.setPosition(wristPanPos);
          // Disable the hook servo to disengage the servo connected to the hook.
          // hookServo.getController().pwmDisable();
          hookServoPWM.setPwmDisable();
          setArmPosition(BotConstants.ARM_POS_FLOOR,BotConstants.ARM_POWER);
          robotHanging = true;
        }
        telemetry.addData("Servo Power", ":%s", hookServo.getController().getPwmStatus().toString());
        telemetry.update();

        previousArmPos = armMotor.getCurrentPosition();

      }
      else {
        //robot is hanging
        if (currentGamepad1.start && !previousGamepad1.start) {
          robotHanging = false;
          hookServoPWM.setPwmEnable();
        }
        telemetry.addData("Servo Power", ":%s", hookServo.getController().getPwmStatus().toString());
        telemetry.update();

      }
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

    setPowerSlew(frontLeftMotor, powerRamping(frontLeftPower, driveSpeed), BotConstants.DRIVE_SLEW_RATE);
    setPowerSlew(frontRightMotor, powerRamping(frontRightPower, driveSpeed), BotConstants.DRIVE_SLEW_RATE);
    setPowerSlew(backLeftMotor, powerRamping(backLeftPower, driveSpeed), BotConstants.DRIVE_SLEW_RATE);
    setPowerSlew(backRightMotor, powerRamping(backRightPower, driveSpeed), BotConstants.DRIVE_SLEW_RATE);
  }
  public void setArmPosition(int position, double speed) {
    armMotor.setTargetPosition(position);
    //armmotor.setPower(speed);
    setPowerSlew(armMotor, speed, BotConstants.ARM_SLEW_RATE);

    for (int i=0; i<5; i++) {
      while (armMotor.isBusy()) {
        sleep(10);
      }
    }
  }
  public static double powerRamping(double power, double speed) {
    if (power < (4.0/5.0)*speed && power > (-4.0/5.0)*speed) {
      return power * (1.0/4.0);
    }
    else if (power > 0) {
      return (power * (4)) - 3;
    }
    else {
      return (power * (4)) + 3;
    }
  }
  public static void setPowerSlew(DcMotor motor, double power, double slewRate) {
    double mpower = motor.getPower();
    if (mpower > power) {
      if (mpower - slewRate > power) {
        motor.setPower(mpower-slewRate);
      }
      else {
        motor.setPower(power);
      }
    }
    else if (mpower <= power) {
      if (mpower + slewRate < power) {
        motor.setPower(mpower + slewRate);
      }
      else {
        motor.setPower(power);
      }
    }

  }
}
