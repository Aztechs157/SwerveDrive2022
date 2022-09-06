// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Shuffle;

public class Drive extends SubsystemBase {
    public SwervePod swervePod = new SwervePod(Constants.MOTOR_ROLL_ID, Constants.MOTOR_SPIN_ID);

    public void set(final ChassisSpeeds speeds) {
        final var states = kinematics.toSwerveModuleStates(speeds);
        final var state = states[chooser.getSelected()];
        set(state);
    }

    public void set(final SwerveModuleState state) {
        swervePod.set(state);
    }

    public void stop() {
        swervePod.stop();
    }

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.WHEEL_LOCATIONS);

    private final SendableChooser<Integer> chooser = new SendableChooser<>();
    {
        chooser.addOption("Front Left", 0);
        chooser.addOption("Front Right", 1);
        chooser.addOption("Back Left", 2);
        chooser.addOption("Back Right", 3);
        Shuffle.driveTab.add("Position Selector", chooser);
    }
}
