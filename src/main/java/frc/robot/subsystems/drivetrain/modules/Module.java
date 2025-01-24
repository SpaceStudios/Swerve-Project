// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.modules;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class Module {
    ModuleIO io;
    ModuleDataAutoLogged data;
    int index;
    public Module(int DriveID, int SteerID, int Index) {
        index = Index+1;
    }

    public void setState(SwerveModuleState state) {
        io.drive(state);
    }

    public SwerveModuleState getState() {
        return io.getState();
    }

    public void periodic() {
        io.updateInputs(data);
        Logger.processInputs("Module"+index, data);
    }
}
