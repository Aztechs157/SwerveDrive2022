package frc.robot.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Constants;

import static frc.robot.lib.ExpectDouble.expect;

public class SwervePod {
    public static class Config {
        public final int motorSpinId;
        public final int motorRollId;
        public boolean inverted = false;

        public Config(final int motorSpinId, final int motorRollId) {
            this.motorSpinId = motorSpinId;
            this.motorRollId = motorRollId;
        }

        public Config inverted() {
            inverted = true;
            return this;
        }
    }

    private final CANSparkMax motorRoll;
    private final CANSparkMax motorSpin;
    // Lamprey1 Absolute Encoder
    private final SparkMaxAnalogSensor encoderSpin;
    private final NetworkTable table;

    public SwervePod(final Config config, final NetworkTable table) {
        this.table = table;
        motorRoll = new CANSparkMax(config.motorRollId, MotorType.kBrushless);
        motorSpin = new CANSparkMax(config.motorSpinId, MotorType.kBrushless);
        encoderSpin = motorSpin.getAnalog(Mode.kAbsolute);

        table.getEntry("Inverted").setBoolean(config.inverted);

        motorRoll.getEncoder().setPositionConversionFactor(Constants.ROLL_ROTATIONS_PER_FOOT);
        motorRoll.setIdleMode(Constants.ROLL_IDLE_MODE);
        motorRoll.setInverted(false);
        motorSpin.setIdleMode(Constants.SPIN_IDLE_MODE);
        motorSpin.setInverted(false);
    }

    public void set(final SwerveModuleState state) {
        spin(wrapDegrees(state.angle.getDegrees()));
        roll(state.speedMetersPerSecond);
    }

    public void directSet(final double rollSpeed, final double spinSpeed) {
        getCurrentSpin();

        directSpin(spinSpeed);
        roll(rollSpeed);
    }

    public void stop() {
        directSpin(0);
        roll(0);
    }

    private final SlewRateLimiter rollSlewRate = new SlewRateLimiter(Constants.ROLL_SLEW_RATE);
    private boolean reversed = false;

    private void roll(double speed) {
        table.getEntry("Input Speed").setDouble(speed);
        table.getEntry("Roll Reversed?").setBoolean(reversed);

        if (reversed) {
            speed = -speed;
        }

        final var inverted = table.getEntry("Inverted");
        if (inverted.getBoolean(false)) {
            speed = -speed;
        }

        motorRoll.set(rollSlewRate.calculate(speed));
    }

    public double getRawRoll() {
        return motorRoll.getEncoder().getPosition();
    }

    private double wrapDegrees(final double degrees) {
        var wrapped = degrees % 360;

        if (wrapped < 0) {
            wrapped += 360;
        }

        expect(wrapped).greaterOrEqual(0).lessOrEqual(360);
        return wrapped;
    }

    private double getCurrentSpin() {
        final var volts = encoderSpin.getPosition();
        table.getEntry("Encoder Volts").setDouble(volts);
        final var degrees = wrapDegrees(volts * Constants.SPIN_ENCODER_VOLTS_TO_DEGREES);

        expect(degrees).greaterOrEqual(0).lessOrEqual(360);
        table.getEntry("Encoder Degrees").setDouble(degrees);
        return degrees;
    }

    private void directSpin(final double speed) {
        motorSpin.set(speed);
    }

    private void spin(final double target) {
        table.getEntry("Input Angle").setDouble(target);
        final var initialDelta = computeInitialDelta(target);
        final var shortestDelta = computeShortestDelta(initialDelta);
        final var pidOutput = computeSpinPidOutput(shortestDelta);
        directSpin(pidOutput);
    }

    private double computeInitialDelta(final double target) {
        final double initial = getCurrentSpin();

        expect(target).greaterOrEqual(0).lessOrEqual(360);
        expect(initial).greaterOrEqual(0).lessOrEqual(360);

        var initialDelta = target - initial;
        expect(initialDelta).greaterOrEqual(-360).lessOrEqual(360);

        if (initialDelta < 0) {
            initialDelta += 360;
        }
        expect(initialDelta).greaterOrEqual(0).lessOrEqual(360);

        return initialDelta;
    }

    private double computeShortestDelta(final double initialDelta) {
        expect(initialDelta).greaterOrEqual(0).lessOrEqual(360);

        if (initialDelta < 90) {
            reversed = false;
            return initialDelta;

        } else if (initialDelta < 270) {
            reversed = true;
            return initialDelta - 180;

        } else if (initialDelta <= 360) {
            reversed = false;
            return initialDelta - 360;
        }

        throw new RuntimeException("Above expect() should have covered this.");
    }

    private final PIDController spinPid = new PIDController(Constants.SPIN_KP, 0, 0);

    private double computeSpinPidOutput(final double shortestDelta) {
        final var pidOutput = spinPid.calculate(getCurrentSpin() + shortestDelta, getCurrentSpin());
        return pidOutput;
    }
}
