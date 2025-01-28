// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.carriage.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import frc.robot.subsystems.carriage.CarriageDataAutoLogged;
import frc.robot.subsystems.carriage.CarriageIO;

/** Add your docs here. */
public class CarriageIO_REAL implements CarriageIO {
    TalonSRX carriageMasterSRX;
    TalonSRX carriageFollowerSRX;

    public CarriageIO_REAL(int masterID, int followerID) {
        carriageMasterSRX = new TalonSRX(masterID);
        carriageFollowerSRX = new TalonSRX(followerID);
        
        carriageFollowerSRX.follow(carriageMasterSRX);

        carriageFollowerSRX.setNeutralMode(NeutralMode.Coast);
        carriageMasterSRX.setNeutralMode(NeutralMode.Coast);

        TalonSRXConfiguration motorConfig = new TalonSRXConfiguration();
        motorConfig.continuousCurrentLimit = 20;
        motorConfig.peakCurrentLimit = 40;

        carriageFollowerSRX.configAllSettings(motorConfig);
        carriageMasterSRX.configAllSettings(motorConfig);
    }

    @Override
    public void setVolts(double volts) {
        carriageMasterSRX.set(ControlMode.PercentOutput, volts/12);
    }

    @Override
    public void updateData(CarriageDataAutoLogged data) {
        data.motorVoltage = Volts.of(carriageMasterSRX.getMotorOutputVoltage());
        data.motorTemperature = Celsius.of(carriageMasterSRX.getTemperature());
        data.motorCurrent = Amps.of(carriageMasterSRX.getSupplyCurrent());
    }

}
