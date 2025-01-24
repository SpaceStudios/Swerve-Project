// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class Constants {
    public class DrivetrainConstants {

        //Module IDs
        public static final int[][] ModuleIDs = new int[][] {
            {1,2},
            {3,4},
            {5,6},
            {7,8}
        };
        
        //Module Translations
        public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(Inches.of(14).in(Meters),Inches.of(14).in(Meters)),
            new Translation2d(-Inches.of(14).in(Meters),Inches.of(14).in(Meters)),
            new Translation2d(-Inches.of(14).in(Meters),-Inches.of(14).in(Meters)),
            new Translation2d(Inches.of(14).in(Meters),-Inches.of(14).in(Meters))
        };

        public static final double kPDrive = 0.8;
        public static final double kIDrive = 0.0;
        public static final double kDDrive = 0.0;
        public static final double kPSteer = 0.8;
        public static final double kISteer = 0.0;
        public static final double kDSteer = 0.0;

        public static final Distance wheelCircumfrence = Inches.of(2).times(Math.PI*2);
    }

    public class RobotConstants {
        public static final LinearVelocity maxSpeed = MetersPerSecond.of(20);
        public static final AngularVelocity maxTurnSpeed = RotationsPerSecond.of(2);
    }
}
