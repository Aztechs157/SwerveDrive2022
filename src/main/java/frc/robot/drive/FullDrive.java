// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class FullDrive extends CommandBase {
    private final Drive drive;
    private final Joystick joystick;

    /** Creates a new FullDrive. */
    public FullDrive(final Drive drive, final Joystick joystick) {
        this.drive = drive;
        addRequirements(drive);
        this.joystick = joystick;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final var x = joystick.getRawAxis(1) * Constants.TELEOP_ROLL_SPEED;
        final var y = joystick.getRawAxis(0) * Constants.TELEOP_ROLL_SPEED;
        final var r = joystick.getRawAxis(2) * Constants.TELEOP_SPIN_SPEED;

        final var speeds = new ChassisSpeeds(x, y, Math.toRadians(r));
        drive.set(speeds);
    }
}
