// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.vision.io.VisionIO_REAL;
import frc.robot.subsystems.vision.io.VisionIO_SIM;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  VisionIO io;
  public Vision() {
    switch (RobotConstants.robotState) {
      case SIM:
        io = new VisionIO_SIM();
        break;
      case REAL:
        io = new VisionIO_REAL();
        break;
    }
  }

  public Pose2d[] getVisionMeasurements() {
    return io.getMeasurements();
  }

  public void update(Pose2d pose) {
    io.update(pose);
  }

  @Override
  public void periodic() {
    Pose2d[] measuredPoses = io.getMeasurements();
    for (int i=0; i<measuredPoses.length; i++) {
      if (measuredPoses[i] != null) {
        Logger.recordOutput("Cameras/Camera #"+(i+1)+" Estimated Pose", measuredPoses[i]);
      }
    }
  }
}
