package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Shuffle {
    public static final ShuffleboardTab driveTab = Shuffleboard.getTab("aaaaaaaaa");

    public static final NetworkTableEntry encoderSpin = driveTab.add("Encoder Spin Degrees", 0).getEntry();
    public static final NetworkTableEntry initialDelta = driveTab.add("Initial Delta", 0).getEntry();
    public static final NetworkTableEntry shortestDelta = driveTab.add("Shortest Delta", 0).getEntry();
    public static final NetworkTableEntry direction = driveTab.add("Direction", false).getEntry();
    public static final NetworkTableEntry pidOutput = driveTab.add("PID Output", 0).getEntry();
    public static final NetworkTableEntry spinVolts = driveTab.add("Spin Encoder Volts", 0).getEntry();

    public static final ShuffleboardLayout states = driveTab.getLayout("States", BuiltInLayouts.kGrid);

    public static final NetworkTable podsTable = NetworkTableInstance.getDefault()
            .getTable("/FRC157/Pods");
}
