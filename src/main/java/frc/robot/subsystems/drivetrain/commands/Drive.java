// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  /** Creates a new Drive. */
  DoubleSupplier joystick1x;
  DoubleSupplier joystick1y;
  DoubleSupplier joystick2x;
  boolean robotRelative;
  Drivetrain drivetrain;
  
  public Drive(Drivetrain drivetrain, DoubleSupplier joystick1X, DoubleSupplier joystick1y, DoubleSupplier joystick2x, boolean robotRelative) {
    this.joystick1x = joystick1X;
    this.joystick1y = joystick1y;
    this.joystick2x = joystick2x;
    this.robotRelative = robotRelative;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.Drive(joystick1x.getAsDouble(), joystick1y.getAsDouble(), joystick2x.getAsDouble(), robotRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
