package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        // --- Wheel & Mechanical Specs (Assuming standard SDS MK4i L2 modules) ---
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
        public static final double kDriveGearRatio = 1.0 / 6.75; // L2 ratio
        public static final double kSteerGearRatio = 1.0 / (150.0 / 7.0); // Mk4i Steer (150/7 : 1)

        // --- SPARK MAX Conversion Factors ---
        // Motor encoder outputs in rotations. Multiply by this to get Meters.
        public static final double kDriveEncoderRot2Meter = kDriveGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;

        // Motor encoder outputs in rotations. Multiply by this to get Radians.
        public static final double kSteerEncoderRot2Rad = kSteerGearRatio * 2 * Math.PI;
        public static final double kSteerEncoderRPM2RadPerSec = kSteerEncoderRot2Rad / 60.0;

        // --- PID Constants for Steering (Add these lines!) ---
        public static final double kSteerP = 1.0; 
        public static final double kSteerI = 0.0;
        public static final double kSteerD = 0.05;

        public static final double kMaxSpeedMetersPerSecond = 0.5;
    }
}
