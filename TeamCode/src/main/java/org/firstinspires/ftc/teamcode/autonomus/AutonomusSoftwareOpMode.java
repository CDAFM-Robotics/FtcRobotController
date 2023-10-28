package org.firstinspires.ftc.teamcode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group = "Testing", name = "autonomus")

public class AutonomusSoftwareOpMode extends LinearOpMode {
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
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        initHardware();

        setupVariables();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


    }

    public void setupVariables() {
        //setup variables
        int zone = (int) Math.floor((Math.random() * 3 + 1)); // replace with recognition code

    }

    public void initHardware() {
        //map hardware

        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        bottomArmServo = hardwareMap.get(Servo.class, "bottomArmServo");
        topArmServo = hardwareMap.get(Servo.class, "topArmServo");
        wristPanServo = hardwareMap.get(Servo.class, "wristPanServo");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        armmotor = hardwareMap.get(DcMotor.class, "armcontrol");

        //set motor behavior

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set initial positions

        bottomArmServo.setPosition(0);
        topArmServo.setPosition(0.6);

    }
}
