// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.carriage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface CarriageIO {
    @AutoLog
    public class CarriageData {
        public AngularVelocity motorVelocity = RotationsPerSecond.of(0);
        public Angle motorPosition = Rotations.of(0);
        public Current motorCurrent = Amps.of(0);
        public Temperature motorTemperature = Celsius.of(0);
        public Voltage motorVoltage = Volts.of(0);
    }
    public abstract void setVolts(double volts);
    public abstract void updateData(CarriageDataAutoLogged data);
}
