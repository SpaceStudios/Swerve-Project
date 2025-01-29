// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.RobotConstants.robotState;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2. command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotConstants.state;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    robotState = isReal() ? state.REAL : state.SIM;
    switch (robotState) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        SimulatedArena.getInstance();
        SimulatedArena.overrideInstance(SimulatedArena.getInstance());
        SimulatedArena.getInstance().placeGamePiecesOnField();
        break;
    }
    Logger.start();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    switch (robotState) {
      case REAL:
        
        break;
      case SIM:
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput("Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        break;
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    if (isSimulation()) {
      SimulatedArena.getInstance().resetFieldForAuto();
    }
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
