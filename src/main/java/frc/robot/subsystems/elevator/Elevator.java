// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  ElevatorIO io;
  ElevatorDataAutoLogged data;
  public Elevator() {
    data = new ElevatorDataAutoLogged();
  }

  public void setHeight(Distance height) {
    io.setHeight(height);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateData(data);
    Logger.processInputs("Robot/Elevator", data);
  }
}
