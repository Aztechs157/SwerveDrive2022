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
        final var speed = (-joystick.getRawAxis(3) + 1) / 2;
        final var x = deadZone(joystick.getRawAxis(1)) * speed;
        final var y = deadZone(joystick.getRawAxis(0)) * speed;
        final var r = deadZone(joystick.getRawAxis(2)) * Constants.TELEOP_SPIN_SPEED;

        final var speeds = new ChassisSpeeds(x, y, Math.toRadians(r));
        drive.set(speeds);
    }

    public double deadZone(final double input) {
        if (Math.abs(input) < Constants.TELEOP_AXIS_THRESHOLD) {
            return 0;
        }
        return input;
    }
}
