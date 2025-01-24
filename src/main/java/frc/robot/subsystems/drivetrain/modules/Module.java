// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.modules;

import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class Module {
    ModuleIO io;
    public Module(int DriveID, int SteerID) {

    }

    public void setState(SwerveModuleState state) {
        io.drive(state);
    }

    public SwerveModuleState getState() {
        return io.getState();
    }
}
