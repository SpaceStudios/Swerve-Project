// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.modules;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;

/** Add your docs here. */
public interface ModuleIO {
    @AutoLog
    public class ModuleData {
        public Distance driveDistance = Meters.of(0);
        public LinearVelocity driveVelocity = MetersPerSecond.of(0);
        public AngularVelocity driveAngularVelocity = RadiansPerSecond.of(0);
        public Current driveCurrent = Amps.of(0);
        public Voltage driveVoltage = Volts.of(0);
        public Temperature driveTemperature = Celsius.of(0);

        public Angle steerDistance = Radians.of(0);
        public AngularVelocity steerVelocity = RadiansPerSecond.of(0);
        public Current steerCurrent = Amps.of(0);
        public Voltage steerVoltage = Volts.of(0);
        public Temperature steerTemperature = Celsius.of(0);
    }

    public abstract void drive(SwerveModuleState moduleState);
    public abstract SwerveModuleState getState();
    public abstract SwerveModulePosition getPosition();
    public abstract void updateInputs(ModuleDataAutoLogged data);
}
