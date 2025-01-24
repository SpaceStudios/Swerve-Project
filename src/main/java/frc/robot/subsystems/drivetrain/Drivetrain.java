// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.modules.Module;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.RobotConstants.*;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  Module[] modules;
  SwerveDriveKinematics kinematics;
  SwerveDrivePoseEstimator poseEstimator;
  SwerveDriveOdometry odometry;
  boolean useGyro;
  SwerveModulePosition[] lastPositions;

  public Drivetrain(boolean usesGyro) {
    modules = new Module[4];
    for (int i=0; i<4; i++) {
      modules[i] = new Module(ModuleIDs[i][0], ModuleIDs[i][1]);
    }

    kinematics = new SwerveDriveKinematics(moduleTranslations);
    odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromRadians(0), new SwerveModulePosition[4]);
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.fromRadians(0), new SwerveModulePosition[4], new Pose2d());
    useGyro = usesGyro;
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
    for (int i=0; i<modules.length; i++) {
      moduleStates[i] = modules[i].getState();
    }
    return moduleStates;
  }

  public Rotation2d getRotation() {
    Rotation2d robotRot = new Rotation2d();
    if (useGyro) {

    } else {
      robotRot.plus(new Rotation2d(kinematics.toTwist2d(lastPositions).dtheta));
    }
    return new Rotation2d();
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
    for (int i=0; i<modules.length; i++) {
      modules[i].setState(moduleStates[i]);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
