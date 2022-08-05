// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
        // tab.addBoolean("Direction", () -> !this.reversed);
        // tab.addNumber("Initial Delta", () -> this.initialDelta);
        // tab.addNumber("Shortest Delta", () -> this.shortestDelta);
        // tab.addNumber("PID Output", () -> this.pidValue);
    }

    public double getSpin() {
        final double raw = encoderSpin.getAnalogIn();
        final double revolutions = raw / Constants.ENCODER_COUNT_PER_REV;
        double degrees = (revolutions * 360.0) % 360;

        if (degrees < 0) {
            degrees += 360;
        }

        expect(degrees).greaterOrEqual(0).lessThan(360);

        return degrees;
    }

    public void spin(final double speed) {
        motorSpin.set(speed);
    }

    private boolean reversed = false;

    public void roll(final double speed) {
        if (reversed) {
            motorRoll.set(-speed);
        } else {
            motorRoll.set(speed);
        }
    }

    public void runSuperDegreeRoutine(final double target) {
        computeInitialDelta(target);
        computeShortestDelta();
        spinPid();
    }

    private double initialDelta = 0;

    private void computeInitialDelta(final double target) {
        final double initial = getSpin();

        expect(target).greaterOrEqual(0).lessOrEqual(360);
        expect(initial).greaterOrEqual(0).lessOrEqual(360);

        double angle = target - initial;
        expect(angle).greaterThan(-360).lessOrEqual(360);

        if (angle < 0) {
            angle += 360;
        }
        expect(angle).greaterThan(0).lessOrEqual(360);

        initialDelta = angle;
    }

    private double shortestDelta = 0;

    private void computeShortestDelta() {
        expect(initialDelta).greaterOrEqual(0).lessOrEqual(360);

        if (initialDelta < 90) {
            shortestDelta = initialDelta;
            reversed = false;
        } else if (initialDelta < 270) {
            shortestDelta = initialDelta - 180;
            reversed = true;
        } else if (initialDelta <= 360) {
            shortestDelta = initialDelta - 360;
            reversed = false;
        }
    }

    private PIDController pid = new PIDController(0.001, 0, 0);

    private double pidValue = 0;

    private void spinPid() {
        pidValue = pid.calculate(getSpin() + shortestDelta, getSpin());
        spin(pidValue);
    }
}
