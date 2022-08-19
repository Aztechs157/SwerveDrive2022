package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Shuffle {
    public static final ShuffleboardTab driveTab = Shuffleboard.getTab("aaaaaaaaa");

    public static final NetworkTableEntry encoderSpin = driveTab.add("Encoder Spin Degrees", 0).getEntry();
    public static final NetworkTableEntry initialDelta = driveTab.add("Initial Delta", 0).getEntry();
    public static final NetworkTableEntry shortestDelta = driveTab.add("Shortest Delta", 0).getEntry();
    public static final NetworkTableEntry direction = driveTab.add("Direction", false).getEntry();
    public static final NetworkTableEntry pidOutput = driveTab.add("PID Output", 0).getEntry();

    public final Tab drivingTab = new Tab("Driving");

    public final BooleanEntry rainbows = drivingTab.add("Rainbows", false);
    public final DoubleEntry lights = drivingTab.add("Lights", 0);

    {
        rainbows.get();
        rainbows.set(true);
    }

    class Tab {
        private final ShuffleboardTab tab;

        public Tab(final String title) {
            this.tab = Shuffleboard.getTab(title);
        }

        public BooleanEntry add(final String title, final boolean defaultValue) {
            final var entry = tab.add(title, defaultValue).getEntry();
            return new BooleanEntry(entry, defaultValue);
        }

        public DoubleEntry add(final String title, final double defaultValue) {
            final var entry = tab.add(title, defaultValue).getEntry();
            return new DoubleEntry(entry, defaultValue);
        }
    }

    class BooleanEntry {
        private NetworkTableEntry entry;
        private boolean defaultValue;

        public BooleanEntry(final NetworkTableEntry entry, final boolean defaultValue) {
            this.entry = entry;
            this.defaultValue = defaultValue;
        }

        public boolean get() {
            return entry.getBoolean(defaultValue);
        }

        public void set(final boolean value) {
            entry.setBoolean(value);
        }
    }

    class DoubleEntry {
        private NetworkTableEntry entry;
        private double defaultValue;

        public DoubleEntry(final NetworkTableEntry entry, final double defaultValue) {
            this.entry = entry;
            this.defaultValue = defaultValue;
        }

        public double get() {
            return entry.getDouble(defaultValue);
        }

        public void set(final double value) {
            entry.setDouble(value);
        }
    }
}
