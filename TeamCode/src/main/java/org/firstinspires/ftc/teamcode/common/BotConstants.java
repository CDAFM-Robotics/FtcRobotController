package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Rect;

@Config
public class BotConstants {


    // Finger Constants
    public static double BOTTOM_ARM_SERVO_CLOSE = 0.10;
    public static double BOTTOM_ARM_SERVO_OPEN = 0.30;
    public static double TOP_ARM_SERVO_CLOSE = 0.08; // JW quickfix?
    public static double TOP_ARM_SERVO_OPEN = 0.30;
    public static double FINGER_SERVO_SETUP_POSITION = 0.15; // Initial SETUP position 0.15 (on 0-1 scale) install first notch where jaws don't touch

    // Wrist Constants
    public static double WRIST_PAN_SERVO_FOLDED = 0.6;
    public static double WRIST_PAN_SERVO_FLOOR = 0.024; // was 0 -> FieldCentric2P
    public static double WRIST_PAN_SERVO_AUTO_DEPLOY = 0.32;
    public static double WRIST_PAN_SERVO_SPEED = 0.008;
    public static int WRIST_DEPLOY_SLEEP = 1500;

    // Arm Constants
    public static int ARM_POS_FLOOR = 175; // was 150 (used FieldCentric2p val
    public static int ARM_POS_DRIVE = 600;
    public static int ARM_POS_AUTO_DEPLOY = 7718;
    public static int ARM_POS_MAX = ARM_POS_AUTO_DEPLOY;
    public static int ARM_POS_L2_DROP = 5720;
    public static int ARM_POS_HANG = 3716;
    public static double ARM_POWER = 1.0;
    public static int ARM_DEPLOY_SLEEP = 6000;



    // Camera Servo Constants
    public static double CAM_SERVO_FRONT = 0.3; // old 0;
    public static double CAM_SERVO_STRIKE = 0.27; // 0.25 (edge hit),
    public static double CAM_SERVO_REAR =0.97; // old 0.67
    public static double CAM_SERVO_RIGHT=0.635; // old 0.335

    // DRONE, HOOK and MISC
    public static double DRONE_POSITION_ARMED=0;
    public static double DRONE_POSITION_LAUNCH=0.25;
    public static double SLOW_MODE = 0.8;
    public static int    RED_TEAM_ID_OFFSET = 3;
    public static int    BLUE_TEAM_ID_OFFSET= 3; // TODO: FIX Temp test
    public static int    BLUE_TEAM = 0;
    public static int    RED_TEAM = 1;
    public static int    START_SIDE_PIXEL = 0;
    public static int    START_SIDE_BACKDROP = 1;
    public static double APRIL_POSE_XOFFSET = 6;
    public static double APRIL_POSE_YOFFSET = -9.5;
    public static double DRONE_POS_ARMED = 0;
    public static double DRONE_POS_LAUNCH = 0.25; //Early Guess
    public static double HOOK_POS_RETRACT = 0.0;
    public static double HOOK_POS_DEPLOY = 0.35; //Early Guess



    // Blue inrange constants for prop detect
    public static int BLUE_HUE_LOW=98; // 112
    public static int BLUE_HUE_HIGH=130; // 125
    public static int BLUE_SAT_LOW=50;  // 89
    public static int BLUE_SAT_HIGH=225; // 219
    public static int BLUE_VAL_LOW=30;  // 51
    public static int BLUE_VAL_HIGH=255;  // 255

    // Region of Interest (Team Prop)
    public static int ROI_RECT_TOP_LEFT_X = 20;
    public static int ROI_RECT_TOP_LEFT_Y = 120;
    public static int ROI_RECT_BOTTOM_RIGHT_X = 620;
    public static int ROI_RECT_BOTTOM_RIGHT_Y = 350;

    // Zone 1 Rect
    public static int Z1_RECT_TLX = ROI_RECT_TOP_LEFT_X;
    public static int Z1_RECT_TLY = ROI_RECT_TOP_LEFT_Y;
    public static int Z1_RECT_BRX = 190;
    public static int Z1_RECT_BRY = ROI_RECT_BOTTOM_RIGHT_Y;

    // Zone 2 Rect
    public static int Z2_RECT_TLX = 250;
    public static int Z2_RECT_TLY = ROI_RECT_TOP_LEFT_Y;
    public static int Z2_RECT_BRX = 420;
    public static int Z2_RECT_BRY = ROI_RECT_BOTTOM_RIGHT_Y;

    // Zone 3 Rect
    public static int Z3_RECT_TLX = 480;
    public static int Z3_RECT_TLY = ROI_RECT_TOP_LEFT_Y;
    public static int Z3_RECT_BRX = ROI_RECT_BOTTOM_RIGHT_X;
    public static int Z3_RECT_BRY = ROI_RECT_BOTTOM_RIGHT_Y;
}
