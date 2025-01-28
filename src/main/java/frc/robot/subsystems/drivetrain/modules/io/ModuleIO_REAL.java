// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.modules.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DrivetrainConstants.*;

import org.opencv.calib3d.StereoBM;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

    public ModuleIO_REAL(int driveID, int steerID) {
        driveSpark = new SparkMax(driveID, MotorType.kBrushless);
        steerSpark = new SparkMax(steerID, MotorType.kBrushless);
        drivePID = new PIDController(kPDrive, kIDrive, kDDrive);
        steerPID = new PIDController(kPSteer, kISteer, kDSteer);
        steerPID.enableContinuousInput(-Math.PI, Math.PI);
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
        driveSpark.setVoltage(drivePID.calculate(((driveEncoder.getVelocity()/60.0)*driveRatio)*wheelCircumfrence.in(Meters)));
        steerSpark.setVoltage(steerPID.calculate(steerEncoder.getPosition()*steerRatio));

        data.driveDistance = Meters.of((driveEncoder.getPosition()*driveRatio)*wheelCircumfrence.in(Meters));
        data.driveVelocity = MetersPerSecond.of(((driveEncoder.getVelocity()/60.0)*driveRatio)*wheelCircumfrence.in(Meters));
        data.driveAngularVelocity = RotationsPerSecond.of(driveEncoder.getVelocity()/60);
        data.driveCurrent = Amps.of(driveSpark.getOutputCurrent());
        data.driveVoltage = Volts.of(driveSpark.getBusVoltage()*driveSpark.getAppliedOutput());
        data.driveTemperature = Celsius.of(driveSpark.getMotorTemperature());

        data.steerDistance = Rotations.of(steerEncoder.getPosition()*steerRatio);
        data.steerVelocity = RotationsPerSecond.of((steerEncoder.getVelocity()/60)*steerRatio);
        data.steerCurrent = Amps.of(steerSpark.getOutputCurrent());
        data.steerVoltage = Volts.of(steerSpark.getBusVoltage()*steerSpark.getAppliedOutput());
        data.steerTemperature = Celsius.of(steerSpark.getMotorTemperature());
    }
}
