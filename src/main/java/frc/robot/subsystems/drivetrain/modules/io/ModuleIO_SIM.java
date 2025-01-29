// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.modules.io;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drivetrain.modules.ModuleDataAutoLogged;
import frc.robot.subsystems.drivetrain.modules.ModuleIO;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DrivetrainConstants.*;

/** Add your docs here. */
public class ModuleIO_SIM implements ModuleIO {
    double driveGearRatio;
    double steerGearRatio;
    PIDController drivePID;
    PIDController steerPID;
    DCMotorSim driveMotor;
    DCMotorSim steerMotor;

    public ModuleIO_SIM(int DriveID, int SteerID) {
        drivePID = new PIDController(kPDrive, kIDrive, kDDrive);
        steerPID = new PIDController(kPSteer, kISteer, kDSteer);
        steerGearRatio = steerRatio;
        driveGearRatio = driveRatio;
        driveMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.04, driveGearRatio), DCMotor.getKrakenX60(1), new double[] {0,0});
        steerMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.04, steerGearRatio), DCMotor.getKrakenX60(1), new double[]{0,0});
        steerPID.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    @Override
    public void drive(SwerveModuleState moduleState) {
        drivePID.setSetpoint(moduleState.speedMetersPerSecond);
        steerPID.setSetpoint(moduleState.angle.getRadians());
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState((driveMotor.getAngularVelocity().in(RotationsPerSecond)*driveGearRatio)*wheelCircumfrence.in(Meters), 
        Rotation2d.fromRadians(steerMotor.getAngularPositionRad()*steerGearRatio));
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition((driveMotor.getAngularPositionRotations()*driveGearRatio)*wheelCircumfrence.in(Meters), Rotation2d.fromRadians(steerMotor.getAngularPositionRad()*steerGearRatio));
    }

    @Override
    public void updateInputs(ModuleDataAutoLogged data) {
        driveMotor.update(0.020);
        steerMotor.update(0.020);

        driveMotor.setInputVoltage(drivePID.calculate((driveMotor.getAngularVelocity().in(RotationsPerSecond)*driveGearRatio)*wheelCircumfrence.in(Meters)));
        steerMotor.setInputVoltage(steerPID.calculate(steerMotor.getAngularPositionRad()*steerGearRatio));
        
        data.driveDistance = Meters.of(driveMotor.getAngularPosition().in(Rotations)*wheelCircumfrence.in(Meters));
        data.driveVelocity = MetersPerSecond.of(driveMotor.getAngularVelocity().in(RotationsPerSecond)*wheelCircumfrence.in(Meters));
        data.driveAngularVelocity = driveMotor.getAngularVelocity();
        data.driveCurrent = Amps.of(driveMotor.getCurrentDrawAmps());
        data.driveVoltage = Volts.of(driveMotor.getInputVoltage());
        data.driveTemperature = Fahrenheit.of(451.0);

        data.steerDistance = steerMotor.getAngularPosition().times(steerGearRatio);
        data.steerVelocity = steerMotor.getAngularVelocity().times(steerGearRatio);
        data.steerCurrent = Amps.of(steerMotor.getCurrentDrawAmps());
        data.steerVoltage = Volts.of(steerMotor.getInputVoltage());
        data.steerTemperature = Fahrenheit.of(451.0);
    }
}
