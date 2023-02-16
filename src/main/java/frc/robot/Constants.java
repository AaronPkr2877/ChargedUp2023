// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(19.5625);

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(24.625);

    public static final int DRIVETRAIN_NAVX_ID = 0;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(87.5);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.5);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(77.3);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(110.2);

    //changed these all to 2.0 (the correct speed)
    public static final double X_PID_CONTROLLER_P = 2.0;
    public static final double Y_PID_CONTROLLER_P = 2.0;
    public static final double THETA_PID_CONTROLLER_P = 2.0;

    // scoring position numbers (m_positionNumber explained)
    // p1-9 is for the scoring grid
    // L Center R
    // 1 2 3 Top
    // 4 5 6 Middle
    // 7 8 9 Bottom
    // p10 and p11 are for the left and right pickup stations
    // note: enums cannot be integers, can only be strings
    public enum Position {
        LEFT_TOP, CENTER_TOP, RIGHT_TOP, LEFT_MIDDLE, CENTER_MIDDLE, RIGHT_MIDDLE, LEFT_BOTTOM, CENTER_BOTTOM,
        RIGHT_BOTTOM,
        // for the two substations/pick up stations
        LEFT_SUBSTATION, RIGHT_SUBSTATION
    }

    public static final double TRAJ_MAX_VEL = 2.0;
    public static final double TRAJ_MAX_ACC = 1.0;

    public static final double CUSTOM_FIELD_LENGTH = 8.780;    // meters
    public static final double CUSTOM_FIELD_WIDTH = 6.0;       // meters

    // max voltages
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the
     * robot.
     */

	public static final double MAX_VOLTAGE = 12.0; //default

    // Xbox button mapping
    public static final int XBOX_A = 1;
    public static final int XBOX_B = 2;
    public static final int XBOX_X = 3;
    public static final int XBOX_Y = 4;

    // bumpers
    public static final int XBOX_LB = 5;
    public static final int XBOX_RB = 6;

    public static final int XBOX_BACK = 7;
    public static final int XBOX_START = 8;

    // joy stick button
    public static final int XBOX_JL = 9;
    public static final int XBOX_JR = 10;

    // Constants for reaching the top level, middle level, bottom level, and feeder station. Currently just have
    // placeholder values

    public static final double ARM_TOP_ROW = Math.toRadians(90);
    public static final double ARM_MIDDLE_ROW = Math.toRadians(0.0);
    public static final double ARM_BOTTOM_ROW = Math.toRadians(0.0);
    public static final double FEEDER_STATION = Math.toRadians(0.0);

}
