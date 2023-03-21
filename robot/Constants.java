// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //drivetrain
  //real
  
    public static final int MOTOR_LEFT_FRONT = 1;
    public static final int MOTOR_LEFT_MID = 2;
    public static final int MOTOR_LEFT_BACK = 3;
    public static final int MOTOR_RIGHT_FRONT = 11;
    public static final int MOTOR_RIGHT_MID = 10;
    public static final int MOTOR_RIGHT_BACK = 9;
   
  
   //fake
   /*
  public static final int MOTOR_LEFT_BACK = 13;
  public static final int MOTOR_LEFT_FRONT = 12;
  public static final int MOTOR_RIGHT_BACK = 15;
  public static final int MOTOR_RIGHT_FRONT =14; 
  */
  public static final double DRIVE_GEAR_RATIO = 5.0/38;
  public static final double DRIVE_WHEEL_RADIUS = 3.0;

  public static final double FORWARD_kP = .04;
  public static final double FORWARD_kI = 0;
  public static final double FORWARD_kD = 0;

  public static final double BALANCE_kP = .058;
  public static final double BALANCE_kI = 0.003;
  public static final double BALANCE_kD =0.0075;

  public static final double TURN_kP = .052; //0.0354;
  public static final double TURN_kI = .000 ; //0.0002;
  public static final double TURN_kD = .0000; //0.0001;

  //arm
  public static final int ARM_LEFT = 4;
  public static final int ARM_RIGHT = 5;
  public static final int ARM_LEFT_ENC = 0;
  public static final int ARM_RIGHT_ENC = 0;
  //public static final int ARM_TICKS_PER_ROTATION =0;
  public static final double ARM_GEAR_RATIO = 15.0/32 * 10.0/72 * 18.0/72;

  public static final boolean ARM_LEFT_REVERSE = false;
  public static final boolean ARM_RIGHT_REVERSE = false;

  public static final double ARM_kP = 0.016;
  public static final double ARM_kI = 0;
  public static final double ARM_kD = 0;

  public static final double ARM_DOWN_ANGLE =0, ARM_MID_ANGLE = -59.665, ARM_HIGH_ANGLE =0;

  //public static final double
  //public static final double
  //public static final double


  //wrist
  public static final int WRIST = 6;
  public static final double WRIST_GEAR_RATIO = 1.0/36/*1/48*/; 
  public static final int WRIST_TICKS_PER_ROTATION = 42;
  public static final boolean WRIST_REVERSE = false;
  public static final double WIRST_START_ANGLE = 0, WRIST_DOWN_CONE = 0, WRIST_MID_CONE =105, WRIST_HIGH_CONE = 0; 
  public static final double WRIST_DOWN_CUBE = 0, WRIST_MID_CUBE = 60, WRIST_HIGH_CUBE =0;
  public static final double WRIST_CONE_ANGLE = 78.5, WRIST_CUBE_ANGLE = 65;

  //lift
  public static final int LIFT_L = 0;
  public static final int LIFT_R =0;
  public static final double LIFT_PULLEY_DIAMETER = 1.25;
  public static final boolean LIFT_L_REVERSE = false;
  public static final boolean LIFT_R_ROTATION = true;
  public static final int LIFT_TICKS_PER_ROTATION =42;
  public static final double LIFT_GEAR_RATIO = 3.0/20;
  public static final double LIFT_EXTENSION_LENGTH = 26.0; //in
  public static final double LIFT_RETRACTION_LENGTH =0.0;
  public static final int LIFT_LIMIT_SWITCH = 0;

  //Claw
  public static final int CLAW =7;
  public static final double CLAW_GEAR_RATIO = 12.0/18;
  public static final double CLAW_TICKS_PER_ROTATION = 42.0;

  public static final double DAVIS_CONSTANT = .5; //INTAKE_CONE_SPEED
  public static final double INTAKE_CUBE_SPEED = .282;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;//?
  }
}
