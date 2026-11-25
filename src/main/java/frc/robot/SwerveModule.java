// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

// Core Spark Max Hardware Classes
import com.revrobotics.spark.SparkMax;

// Configuration Classes (REVLib 2026 unified configuration structure)
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final SparkMax driveMotor;
  private final SparkMax steerMotor;
  private final CANcoder absoluteEncoder;
  private final double angleOffsetRadians;

  public SwerveModule(int driveID, int steerID, int cancoderID, double angleOffsetRotations, boolean inverted) {
    this.angleOffsetRadians = angleOffsetRotations * Math.PI * 2.0;

    driveMotor = new SparkMax(driveID, SparkMax.MotorType.kBrushless);
    steerMotor = new SparkMax(steerID, SparkMax.MotorType.kBrushless);
    absoluteEncoder = new CANcoder(cancoderID);

    // Configure Drive Motor
    SparkMaxConfig driveConfig = new SparkMaxConfig();

    driveConfig.inverted(inverted);

    driveConfig.encoder
        .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
        .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure Steer Motor
    SparkMaxConfig steerConfig = new SparkMaxConfig();
    
    steerConfig.inverted(true);

    steerConfig.encoder
        .positionConversionFactor(ModuleConstants.kSteerEncoderRot2Rad)
        .velocityConversionFactor(ModuleConstants.kSteerEncoderRPM2RadPerSec);

    // Setup internal PID to target position using integrated encoder
    steerConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ModuleConstants.kSteerP)
        .i(ModuleConstants.kSteerI)
        .d(ModuleConstants.kSteerD)
        .positionWrappingEnabled(true) // Wraps steer domain automatically
        .positionWrappingInputRange(-Math.PI, Math.PI); // -180 to 180 deg

    steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Synchronize Encoders at boot
    resetToAbsolute();
  }

  /**
   * Seeds the absolute position from the CANcoder into the internal motor
   * encoder.
   */
  public void resetToAbsolute() {
    // Phoenix 6 returns position in Rotations (0.0 to 1.0). Convert to Radians.
    double absoluteRotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    double absoluteRadians = absoluteRotations * 2.0 * Math.PI;

    // Subtract the physical offset determined during your zeroing procedure
    double correctedPosition = absoluteRadians - angleOffsetRadians;

    System.out.println(absoluteRotations);

    // Seed the SparkMax internal encoder
    steerMotor.getEncoder().setPosition(correctedPosition);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveMotor.getEncoder().getVelocity(), new Rotation2d(steerMotor.getEncoder().getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotor.getEncoder().getPosition(), new Rotation2d(steerMotor.getEncoder().getPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // 1. Optimize the state to prevent the wheel from spinning 180 degrees
    // needlessly
    // Example: If a wheel is at 0° and wants to go backwards, it flips the motor
    // direction
    // instead of wasting time rotating the wheel to 180°.
    SwerveModuleState optimizedState = SwerveModuleState.optimize(
        desiredState,
        Rotation2d.fromRadians(steerMotor.getEncoder().getPosition()));

    // 2. Command the Drive Motor speed (Velocity Control Loop or direct
    // PercentOutput)
    // Here we use reference voltage/duty cycle directly, but you can also use a PID
    // controller
    driveMotor.set(optimizedState.speedMetersPerSecond / ModuleConstants.kMaxSpeedMetersPerSecond);

    // 3. Command the Steer Motor position (Using the SparkMax Closed-Loop
    // Controller)
    // The Spark closed-loop handles the onboard PID math to snap the wheel to the
    // correct angle
    steerMotor.getClosedLoopController().setReference(
        optimizedState.angle.getRadians(),
        ControlType.kPosition);
  }
}
