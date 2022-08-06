// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Shuffle;

import static frc.robot.lib.ExpectDouble.expect;

public class Drive extends SubsystemBase {
    private final CANSparkMax motorSpin = new CANSparkMax(Constants.MOTOR_SPIN_ID, MotorType.kBrushless);
    private final CANSparkMax motorRoll = new CANSparkMax(Constants.MOTOR_ROLL_ID, MotorType.kBrushless);

    // Lamprey1 Absolute Encoder
    private final SensorCollection encoderSpin = new TalonSRX(Constants.ENCODER_ID).getSensorCollection();

    /** Creates a new Drive. */
    public Drive() {
        final var tab = Shuffleboard.getTab("aaaaaaaaa");
        tab.addNumber("Encoder Spin", encoderSpin::getAnalogIn);
        tab.addNumber("Encoder Spin Degrees", this::getSpin);
    }

    public double getSpin() {
        final double raw = encoderSpin.getAnalogIn();
        final double revolutions = raw / Constants.ENCODER_COUNT_PER_REV;
        double degrees = (revolutions * 360.0) % 360;

        if (degrees < 0) {
            degrees += 360;
        }

        expect(degrees).greaterOrEqual(0).lessOrEqual(360);

        return degrees;
    }

    public void spin(final double speed) {
        motorSpin.set(speed);
    }

    private final SlewRateLimiter rollSlewRate = new SlewRateLimiter(Constants.ROLL_SLEW_RATE);
    private boolean reversed = false;

    public void roll(final double speed) {
        if (reversed) {
            motorRoll.set(rollSlewRate.calculate(-speed));
        } else {
            motorRoll.set(rollSlewRate.calculate(speed));
        }
    }

    public void runSuperDegreeRoutine(final double target) {
        final var initialDelta = computeInitialDelta(target);
        Shuffle.initialDelta.setNumber(initialDelta);

        final var shortestDelta = computeShortestDelta(initialDelta);
        Shuffle.shortestDelta.setNumber(shortestDelta);
        Shuffle.direction.setBoolean(!reversed);

        final var pidOutput = computeSpinPidOutput(shortestDelta);
        Shuffle.pidOutput.setNumber(pidOutput);
        spin(pidOutput);
    }

    private double computeInitialDelta(final double target) {
        final double initial = getSpin();

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
        return spinSlewRate.calculate(spinPid.calculate(getSpin() + shortestDelta, getSpin()));
    }
}
