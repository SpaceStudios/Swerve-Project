// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.modules.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DrivetrainConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.subsystems.drivetrain.modules.ModuleDataAutoLogged;
import frc.robot.subsystems.drivetrain.modules.ModuleIO;

/** Add your docs here. */
public class ModuleIO_REAL implements ModuleIO {
    SparkMax driveSpark;
    SparkMax steerSpark;
    PIDController drivePID;
    PIDController steerPID;
    RelativeEncoder driveEncoder;
    RelativeEncoder steerEncoder;
    

    public ModuleIO_REAL(int driveID, int steerID, int encoderID) {
        // SparkMaxConfig driveConfig = new SparkMaxConfig();
        // SparkMaxConfig steerConfig = new SparkMaxConfig();
        // driveConfig.inverted(false).smartCurrentLimit(40).idleMode(IdleMode.kBrake);
        // steerConfig.inverted(false).smartCurrentLimit(40).idleMode(IdleMode.kCoast);
        driveSpark = new SparkMax(driveID, MotorType.kBrushless);
        steerSpark = new SparkMax(steerID, MotorType.kBrushless);
        drivePID = new PIDController(kPDrive, kIDrive, kDDrive, 0.020);
        steerPID = new PIDController(kPSteer, kISteer, kDSteer, 0.020);
        steerPID.enableContinuousInput(-Math.PI, Math.PI);

        // driveSpark.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // steerSpark.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        driveEncoder = driveSpark.getEncoder();
        steerEncoder = steerSpark.getEncoder();
    }

    @Override
    public void drive(SwerveModuleState moduleState) {
        drivePID.setSetpoint(moduleState.speedMetersPerSecond);
        steerPID.setSetpoint(moduleState.angle.getRotations());
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(((driveEncoder.getVelocity()/60.0)*driveRatio)*wheelCircumfrence.in(Meters),Rotation2d.fromRotations(steerEncoder.getPosition()*steerRatio));
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition((driveEncoder.getPosition()*driveRatio)*wheelCircumfrence.in(Meters),(Rotation2d.fromRotations(steerEncoder.getPosition()*steerRatio)));
    }

    @Override
    public void updateInputs(ModuleDataAutoLogged data) {
        data.driveDistance = Meters.of(driveEncoder.getPosition() / driveRatio * wheelCircumfrence.in(Meters));
        data.driveAngularVelocity = RotationsPerSecond.of(driveEncoder.getVelocity() / 60.0);
        data.driveVelocity = MetersPerSecond.of(data.driveAngularVelocity.in(RotationsPerSecond) / driveRatio * wheelCircumfrence.in(Meters));
        data.driveCurrent = Amps.of(driveSpark.getOutputCurrent());
        data.driveVoltage = Volts.of(driveSpark.getBusVoltage() * driveSpark.getAppliedOutput());
        data.driveTemperature = Celsius.of(driveSpark.getMotorTemperature());

        data.steerDistance = Rotations.of(steerEncoder.getPosition() / steerRatio);
        data.steerVelocity = RotationsPerSecond.of(steerEncoder.getVelocity() / 60.0 / steerRatio);
        data.steerCurrent = Amps.of(steerSpark.getOutputCurrent());
        data.steerVoltage = Volts.of(steerSpark.getBusVoltage() * steerSpark.getAppliedOutput());
        data.steerTemperature = Celsius.of(steerSpark.getMotorTemperature());

        driveSpark.setVoltage(drivePID.calculate(data.driveVelocity.in(MetersPerSecond)));
        steerSpark.setVoltage(steerPID.calculate(data.steerDistance.in(Rotations)));
    }
}
