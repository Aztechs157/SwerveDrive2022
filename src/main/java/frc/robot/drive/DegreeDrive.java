// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
        final double x = joystick.getRawAxis(0);
        final double y = joystick.getRawAxis(1);

        final double angle = Math.toDegrees(Math.atan2(y, x)) + 180;

        final double speed = joystick.getRawAxis(5) * 0.1;

        drive.runSuperDegreeRoutine(angle);
        drive.roll(speed);
    }
}
