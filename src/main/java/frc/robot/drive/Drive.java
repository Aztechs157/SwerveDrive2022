// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Shuffle;

public class Drive extends SubsystemBase {
    public SwervePod[] swervePods = new SwervePod[] {
            new SwervePod(Constants.FRONT_LEFT_CONFIG),
            new SwervePod(Constants.FRONT_RIGHT_CONFIG),
            new SwervePod(Constants.BACK_LEFT_CONFIG),
            new SwervePod(Constants.BACK_RIGHT_CONFIG),

    };

    public void set(final ChassisSpeeds speeds) {
        final var states = kinematics.toSwerveModuleStates(speeds);

        for (var i = 0; i < states.length; i++) {
            this.speeds[i].setNumber(states[i].speedMetersPerSecond);
            this.angles[i].setNumber(states[i].angle.getDegrees());

            swervePods[i].set(states[i]);
        }
    }

    private final NetworkTableEntry[] speeds = new NetworkTableEntry[] { null, null, null, null };
    private final NetworkTableEntry[] angles = new NetworkTableEntry[] { null, null, null, null };;
    {
        final var labels = new String[] { "Front Left", "Front Right", "Back Left", "Back Right" };

        for (var i = 0; i < labels.length; i++) {
            final var layout = Shuffle.states.getLayout(labels[i], BuiltInLayouts.kList).withPosition(i, 0);

            speeds[i] = layout.add("Speed", 0).getEntry();
            angles[i] = layout.add("Angle", 0).getEntry();
        }
    }

    public void set(final SwerveModuleState state) {
        this.speeds[chooser.getSelected()].setNumber(state.speedMetersPerSecond);
        this.angles[chooser.getSelected()].setNumber(state.angle.getDegrees());
        swervePods[chooser.getSelected()].set(state);
    }

    public void stop() {
        swervePods[chooser.getSelected()].stop();
    }

    public void directSet(final double rollSpeed, final double spinSpeed) {
        swervePods[chooser.getSelected()].directSet(rollSpeed, spinSpeed);
    }

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.WHEEL_LOCATIONS);

    private final SendableChooser<Integer> chooser = new SendableChooser<>();
    {
        chooser.setDefaultOption("Front Left", 0);
        chooser.addOption("Front Right", 1);
        chooser.addOption("Back Left", 2);
        chooser.addOption("Back Right", 3);
        Shuffle.driveTab.add("Position Selector", chooser);
    }
}
