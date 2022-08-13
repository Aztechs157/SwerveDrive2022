// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
    public SwervePod swervePod = new SwervePod(Constants.MOTOR_ROLL_ID, Constants.MOTOR_SPIN_ID);

    public void spin(final double target) {
        swervePod.spin(target);
    }

    public void roll(final double speed) {
        swervePod.roll(speed);
    }
}
