// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    /* CAN IDs of Drive Motors */
    public static final int LEFT_FRONT_DRIVE_ID = 2;
    public static final int RIGHT_FRONT_DRIVE_ID = 5;
    public static final int LEFT_BACK_DRIVE_ID = 8;
    public static final int RIGHT_BACK_DRIVE_ID = 11;

    /* Steer Encoder CAN IDs */
    public static final int LEFT_FRONT_STEER_CANCODER_ID = 3;
    public static final int RIGHT_FRONT_STEER_CANCODER_ID = 6;
    public static final int LEFT_BACK_STEER_CANCODER_ID = 9;
    public static final int RIGHT_BACK_STEER_CANCODER_ID = 12;

    /* CAN IDs of steer Motors turning */
    public static final int LEFT_FRONT_STEER_ID = 4;
    public static final int RIGHT_FRONT_STEER_ID = 7;
    public static final int LEFT_BACK_STEER_ID = 10;
    public static final int RIGHT_BACK_STEER_ID = 13;

    public static final double WHEELBASE = 0.5;
    public static final double TRACKWIDTH = 0.5;
    public static final double WHEEL_DIAMETER = 0.10033;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static final double DRIVE_GEAR_RATIO = 0.5;

    public static final double MAX_SPEED_METERS_PER_SECOND = 4.5;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;

    public static final double DRIVE_P = 0.1;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;

    public static final double STEER_P = 1.0;
    public static final double STEER_I = 0.0;
    public static final double STEER_D = 0.1;

    /* Current limits */
    public static final double MAX_VOLTAGE = 12.0;
    public static final double DRIVE_CURRENT_LIMIT = 40;
    public static final int TURN_CURRENT_LIMIT = 20;

    /* Feedforward */
    public static final double kS = 0.1; // static friction compensation
    public static final double kV = 0.8; // velocity proportional feedforward
    public static final double kA = 0.1; // acceleration feedforward

    /* Odometry */
    public static final double DRIVE_ENCODER_DISTANCE_PER_PULSE = (DriveConstants.WHEEL_DIAMETER * Math.PI)
        / (DriveConstants.DRIVE_GEAR_RATIO * 2048.0);

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
