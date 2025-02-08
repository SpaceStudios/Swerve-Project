// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.carriage.commands.setVolts;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.SetHeight;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(0);
  Vision vision = new Vision();
  // Drivetrain drivetrain = new Drivetrain(false, vision);
  // Carriage carriage = new Carriage();
  // Elevator elevator = new Elevator();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // drivetrain.setDefaultCommand(new Drive(drivetrain, 
    // () -> MathUtil.applyDeadband(-controller.getLeftX(), 0.1), 
    // () -> MathUtil.applyDeadband(-controller.getLeftY(), 0.1), 
    // () -> MathUtil.applyDeadband(-controller.getRightX(), 0.1), 
    // false));
    // controller.leftTrigger().whileTrue(new setVolts(carriage, 12));
    // controller.leftBumper().whileTrue(new setVolts(carriage, -12));
    // controller.a().whileTrue(new SetHeight(Inches.of(20), elevator));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
