// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.modules.Module;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.RobotConstants.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  Module[] modules;
  SwerveDriveKinematics kinematics;
  SwerveDrivePoseEstimator poseEstimator;
  boolean useGyro;
  SwerveModulePosition[] lastPositions;
  SwerveModulePosition[] moduleDeltas;
  SelfControlledSwerveDriveSimulation sim;

  public Drivetrain(boolean usesGyro) {
    modules = new Module[4];
    for (int i=0; i<4; i++) {
      modules[i] = new Module(ModuleIDs[i][0], ModuleIDs[i][1], i);
    }

    kinematics = new SwerveDriveKinematics(moduleTranslations);
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.fromRadians(0), new SwerveModulePosition[] {new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()}, new Pose2d());
    lastPositions = new SwerveModulePosition[] {new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()};
    moduleDeltas = new SwerveModulePosition[] {new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()};
    useGyro = usesGyro;
    switch (robotState) {
      case REAL:
        
        break;
      case SIM:
        SwerveDriveSimulation driveSim = new SwerveDriveSimulation(
          new DriveTrainSimulationConfig(
            Kilogram.of(31), 
            Inches.of(31), 
            Inches.of(31), 
            Inches.of(20.5), 
            Inches.of(20.5), 
            () -> (new SwerveModuleSimulation(new SwerveModuleSimulationConfig(
              DCMotor.getKrakenX60(1), 
              DCMotor.getKrakenX60(1), 
              driveRatio, 
              steerRatio, 
              Volts.of(0), 
              Volts.of(0), 
              wheelRadius, 
              KilogramSquareMeters.of(0), 
              driveRatio))), 
            () -> (new GyroSimulation(0.0, 0.0))
          ),
          new Pose2d()
        );
        sim = new SelfControlledSwerveDriveSimulation(driveSim, null, null);
        SimulatedArena.getInstance().addDriveTrainSimulation(sim.getDriveTrainSimulation());
        break;
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
    for (int i=0; i<modules.length; i++) {
      moduleStates[i] = modules[i].getState();
    }
    return moduleStates;
  }

  public Rotation2d getRotation() {
    Rotation2d robotRot = new Rotation2d(0);
    Logger.recordOutput("Use Gyro", useGyro);
    if (useGyro) {

    } else {
      robotRot.plus(new Rotation2d(kinematics.toTwist2d(moduleDeltas).dtheta));
      Logger.recordOutput("Robot Twist", robotRot.plus(new Rotation2d(kinematics.toTwist2d(moduleDeltas).dtheta)));
      robotRot = sim.getRawGyroAngle();
    }
    return robotRot;
  }

  public void Drive(double joystick1x, double joystick1y, double joystick2x, boolean robotRelative) {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(robotRelative ? ChassisSpeeds.fromRobotRelativeSpeeds(
      maxSpeed.in(MetersPerSecond)*joystick1y, 
      maxSpeed.in(MetersPerSecond)*joystick1x, 
      maxTurnSpeed.in(RadiansPerSecond)*joystick2x, 
      this.getRotation())
       : ChassisSpeeds.fromFieldRelativeSpeeds(
        maxSpeed.in(MetersPerSecond)*joystick1y, 
        maxSpeed.in(MetersPerSecond)*joystick1x, 
        maxTurnSpeed.in(RadiansPerSecond)*joystick2x, 
        this.getRotation())
    );
    Logger.recordOutput("Drivetrain/Intended States", moduleStates);
    for (int i=0; i<modules.length; i++) {
      moduleStates[i].optimize(modules[i].getState().angle);
      modules[i].setState(moduleStates[i]);
    }
    sim.runSwerveStates(moduleStates);
  }

  @Override
  public void periodic() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
    for (int i=0; i<modules.length; i++) {
      modules[i].periodic();
      moduleStates[i] = modules[i].getState();
      modulePositions[i] = modules[i].getPosition();
      moduleDeltas[i] = new SwerveModulePosition(modulePositions[i].distanceMeters-lastPositions[i].distanceMeters,modulePositions[i].angle);
    }
    Pose2d pose = poseEstimator.update(getRotation(), modulePositions);
    Logger.recordOutput("Drivetrain/Robot Rotation", getRotation());
    Logger.recordOutput("Drivetrain/Swerve Pose", pose);
    Logger.recordOutput("Maple Sim Pose", sim.getActualPoseInSimulationWorld());
    Logger.recordOutput("Drivetrain/Swerve States", moduleStates);
  }
}
