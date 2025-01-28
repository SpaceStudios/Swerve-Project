// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface ElevatorIO {
    @AutoLog
    public class ElevatorData {
        public Distance distanceSetpoint = Meters.of(0);
        public Distance currentPosition = Meters.of(0);
        public LinearVelocity currentVelocity = MetersPerSecond.of(0);
        public Voltage appliedVolts = Volts.of(0);
        public Current motorCurrent = Amps.of(0);
        public Temperature motorTemp = Celsius.of(0);
    }
    
    public abstract void setHeight(Distance height);
    public abstract void updateData(ElevatorDataAutoLogged data);
}
