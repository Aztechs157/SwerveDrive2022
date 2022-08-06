// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class DegreeDrive extends CommandBase {
    private final Drive drive;
    private final Joystick joystick;

    /** Creates a new TeleopDrive. */
    public DegreeDrive(final Drive drive, final Joystick joystick) {
        this.drive = drive;
        addRequirements(drive);
        this.joystick = joystick;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final var x = joystick.getRawAxis(0);
        final var y = joystick.getRawAxis(1);

        final var aboveThreshold = Math.abs(x) > Constants.TELEOP_AXIS_THRESHOLD
                || Math.abs(y) > Constants.TELEOP_AXIS_THRESHOLD;

        if (!aboveThreshold) {
            drive.spin(0);
            drive.roll(0);
            return;
        }

        final var angle = Math.toDegrees(Math.atan2(y, x)) + 180;
        drive.runSuperDegreeRoutine(angle);

        final var length = Math.sqrt((x * x) + (y * y));
        final var speed = Math.min(1, length) * Constants.TELEOP_ROLL_SPEED;
        drive.roll(speed);
    }
}
