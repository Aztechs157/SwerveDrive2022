// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {
    private final Drive drive;
    private final Joystick joystick;

    /** Creates a new TeleopDrive. */
    public TeleopDrive(final Drive drive, final Joystick joystick) {
        this.drive = drive;
        addRequirements(drive);
        this.joystick = joystick;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final double speed = 0.3;
        final double rollSpeed = joystick.getRawAxis(1) * speed;
        final double spinSpeed = joystick.getRawAxis(0) * speed;

        drive.roll(rollSpeed);
        drive.spin(spinSpeed);
    }
}
