package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Shuffle {
    public static final ShuffleboardTab driveTab = Shuffleboard.getTab("aaaaaaaaa");

    public static final NetworkTableEntry direction = driveTab.add("Direction", 0).getEntry();
    public static final NetworkTableEntry initialDelta = driveTab.add("Initial Delta", 0).getEntry();
    public static final NetworkTableEntry shortestDelta = driveTab.add("Shortest Delta", 0).getEntry();
    public static final NetworkTableEntry pidOutput = driveTab.add("PID Output", 0).getEntry();
}
