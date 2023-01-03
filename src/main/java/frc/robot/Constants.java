// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.drive.SwervePod;

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
    public static final SwervePod.Config FRONT_LEFT_CONFIG = new SwervePod.Config(1, 2);
    public static final SwervePod.Config FRONT_RIGHT_CONFIG = new SwervePod.Config(3, 4);
    public static final SwervePod.Config BACK_LEFT_CONFIG = new SwervePod.Config(5, 6);
    public static final SwervePod.Config BACK_RIGHT_CONFIG = new SwervePod.Config(7, 8);

    public static final int JOYSTICK_ID = 0;

    public static final double SPIN_ENCODER_VOLTS_TO_DEGREES = 360 / 3.3;

    public static final double TELEOP_ROLL_SPEED = 0.50;
    public static final double TELEOP_AXIS_THRESHOLD = 0.2;
    public static final double TELEOP_SPIN_SPEED = 10;
    public static final double ROLL_SLEW_RATE = 1;
    public static final double SPIN_KP = 0.0025;

    public static final IdleMode ROLL_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode SPIN_IDLE_MODE = IdleMode.kCoast;

    public static final double FEET_PER_ROLL_WHEEL_TURN = 1.1;
    public static final double MOTOR_TURNS_PER_ROLL_WHEEL_TURN = 7;
    public static final double ROLL_ROTATIONS_PER_FOOT = FEET_PER_ROLL_WHEEL_TURN / MOTOR_TURNS_PER_ROLL_WHEEL_TURN;

    public static final double CENTER_TO_POD = 0.707;
    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(CENTER_TO_POD, CENTER_TO_POD);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(CENTER_TO_POD, -CENTER_TO_POD);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-CENTER_TO_POD, CENTER_TO_POD);
    public static final Translation2d BACk_RIGHT_LOCATION = new Translation2d(-CENTER_TO_POD, -CENTER_TO_POD);
    public static final Translation2d[] WHEEL_LOCATIONS = new Translation2d[] {
            FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACk_RIGHT_LOCATION };
}
