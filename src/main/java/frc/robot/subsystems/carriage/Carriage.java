// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.carriage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.carriage.io.CarriageIO_REAL;
import static frc.robot.Constants.CarraigeConstants.*;

import org.littletonrobotics.junction.Logger;

public class Carriage extends SubsystemBase {
  /** Creates a new Carriage. */
  CarriageIO io;
  CarriageDataAutoLogged data;
  public Carriage() {
    io = new CarriageIO_REAL(carriageIDs[0], carriageIDs[1]);
    data = new CarriageDataAutoLogged();
  }

  public void setVolts(double volts) {
    io.setVolts(volts);
  }

  @Override
  public void periodic() {
    io.updateData(data);
    Logger.processInputs("Carriage", data);
  }
}
