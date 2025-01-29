// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.modules;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.drivetrain.modules.io.ModuleIO_REAL;
import frc.robot.subsystems.drivetrain.modules.io.ModuleIO_SIM;

/** Add your docs here. */
public class Module {
    ModuleIO io;
    ModuleDataAutoLogged data;
    int index;
    public Module(int DriveID, int SteerID, int Index) {
        index = Index+1;
        data = new ModuleDataAutoLogged();
        switch (RobotConstants.robotState) {
            case SIM:
            io = new ModuleIO_SIM(DriveID, SteerID);
                break;
            case REAL:
                io = new ModuleIO_REAL(DriveID, SteerID);
                break;
        }
    }

    public void setState(SwerveModuleState state) {
        state.speedMetersPerSecond *= state.angle.minus(getState().angle).getCos();
        io.drive(state);
    }

    public SwerveModuleState getState() {
        return io.getState();
    }

    public SwerveModulePosition getPosition() {
        return io.getPosition();
    }

    public void periodic() {
        io.updateInputs(data);
        Logger.processInputs("Module"+index, data);
    }
}
