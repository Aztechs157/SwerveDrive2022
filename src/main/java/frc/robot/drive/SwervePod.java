package frc.robot.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Shuffle;

import static frc.robot.lib.ExpectDouble.expect;

public class SwervePod {
    public static class Config {
        public final int motorSpinId;
        public final int motorRollId;

        public Config(final int motorSpinId, final int motorRollId) {
            this.motorSpinId = motorSpinId;
            this.motorRollId = motorRollId;
        }
    }

    private final CANSparkMax motorRoll;
    private final CANSparkMax motorSpin;

    // Lamprey1 Absolute Encoder
    private final SparkMaxAnalogSensor encoderSpin;

    public SwervePod(final Config config) {
        motorRoll = new CANSparkMax(config.motorRollId, MotorType.kBrushless);
        motorSpin = new CANSparkMax(config.motorSpinId, MotorType.kBrushless);
        encoderSpin = motorSpin.getAnalog(Mode.kAbsolute);
        motorRoll.getEncoder().setPositionConversionFactor(Constants.SPIN_ROTATIONS_PER_FOOT);
        motorRoll.setIdleMode(Constants.ROLL_IDLE_MODE);
        motorSpin.setIdleMode(Constants.SPIN_IDLE_MODE);
    }

    public void set(final SwerveModuleState state) {
        roll(state.speedMetersPerSecond);
        spin(wrapDegrees(state.angle.getDegrees()));
    }

    public void directSet(final double rollSpeed, final double spinSpeed) {
        getCurrentSpin();

        roll(rollSpeed);
        directSpin(spinSpeed);
    }

    public void stop() {
        directSpin(0);
        roll(0);
    }

    private final SlewRateLimiter rollSlewRate = new SlewRateLimiter(Constants.ROLL_SLEW_RATE);
    private boolean reversed = false;

    private void roll(double speed) {
        if (reversed) {
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
        final var degrees = wrapDegrees(volts * Constants.VOLTS_TO_DEGREES);

        expect(degrees).greaterOrEqual(0).lessOrEqual(360);
        Shuffle.encoderSpin.setNumber(degrees);
        return degrees;
    }

    private void directSpin(final double speed) {
        motorSpin.set(speed);
    }

    private void spin(final double target) {
        final var initialDelta = computeInitialDelta(target);
        Shuffle.initialDelta.setNumber(initialDelta);

        final var shortestDelta = computeShortestDelta(initialDelta);
        Shuffle.shortestDelta.setNumber(shortestDelta);
        Shuffle.direction.setBoolean(!reversed);

        final var pidOutput = computeSpinPidOutput(shortestDelta);
        Shuffle.pidOutput.setNumber(pidOutput);
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
    private final SlewRateLimiter spinSlewRate = new SlewRateLimiter(Constants.SPIN_SLEW_RATE);

    private double computeSpinPidOutput(final double shortestDelta) {
        final var pidOutput = spinPid.calculate(getCurrentSpin() + shortestDelta, getCurrentSpin());
        final var slewOutput = spinSlewRate.calculate(pidOutput);
        return slewOutput;
    }
}
