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
    public static final int MOTOR_SPIN_ID = 4;
    public static final int MOTOR_ROLL_ID = 3;
    public static final int ENCODER_ID = 17;

    public static final int JOYSTICK_ID = 0;

    public static final int ENCODER_COUNT_PER_REV = 1024;

    public static final double TELEOP_ROLL_SPEED = 1.00;
    public static final double TELEOP_AXIS_THRESHOLD = 0.1;

    public static final double ROLL_SLEW_RATE = 1;
    public static final double SPIN_SLEW_RATE = 1;
    public static final double SPIN_KP = 0.0025;
}
