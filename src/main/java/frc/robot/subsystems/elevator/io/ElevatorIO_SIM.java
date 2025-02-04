// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.RobotConstants.state;
import frc.robot.subsystems.elevator.ElevatorDataAutoLogged;
import frc.robot.subsystems.elevator.ElevatorIO;

/** Add your docs here. */
public class ElevatorIO_SIM implements ElevatorIO {
    ElevatorSim sim;
    PIDController elevatorController;

    public ElevatorIO_SIM() {
        sim = new ElevatorSim(
            DCMotor.getNEO(2), 
            gearing, //Replace with Elevator Constants Gearing
            carriageMass.in(Kilogram), //Replace with the actual mass of the carriage
            drumRadiusMeters.in(Meters), //Replace with Drum Radius
            minHeight.in(Meters), //Will always be 0 because the elevator is going to be relative unless otherwise noted
            maxHeight.in(Meters), //Replace with Elevator max height constant
            true, //Always will be true because the elevator will never be sideways
            startingHeight.in(Meters), //Will alwats be at 0 because elevator should always be at lowest position during a match unless otherwise noted
            new double[] {0,0}
        );
        elevatorController = new PIDController(0.9, 0, 0,0.020);
    }


    @Override
    public void setHeight(Distance height) {
        Logger.recordOutput("Elevator Set Height", height);
        elevatorController.setSetpoint(height.in(Meters));
    }

    @Override
    public void updateData(ElevatorDataAutoLogged data) {
        sim.update(0.020);
        sim.setInputVoltage(elevatorController.calculate(sim.getPositionMeters()));
        
        data.currentPosition = Meters.of(sim.getPositionMeters());
        data.currentVelocity = MetersPerSecond.of(sim.getVelocityMetersPerSecond());
        data.appliedVolts = Volts.of(0.0);
        data.distanceSetpoint = Meters.of(elevatorController.getSetpoint());
        data.motorCurrent = Amps.of(sim.getCurrentDrawAmps());
        data.motorTemp = Celsius.of(0.0);
    }
}
