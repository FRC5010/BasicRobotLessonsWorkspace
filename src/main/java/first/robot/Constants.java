// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Translation2d;

import com.ctre.phoenix6.signals.InvertedValue;

public final class Constants {
  // Added by Try It #4: one CAN ID + one magnet offset per corner, all named,
  // instead of literals baked into the array in Drivetrain.
  public static final class DriveConstants {
    public static final int kFrontLeftDrivePort = 4;     // CAN IDs — change to yours
    public static final int kFrontLeftSteerPort = 3;
    public static final int kFrontLeftCancoderPort = 13;
    public static final int kFrontRightDrivePort = 2;
    public static final int kFrontRightSteerPort = 1;
    public static final int kFrontRightCancoderPort = 16;
    public static final int kBackLeftDrivePort = 6;
    public static final int kBackLeftSteerPort = 5;
    public static final int kBackLeftCancoderPort = 14;
    public static final int kBackRightDrivePort = 8;
    public static final int kBackRightSteerPort = 7;
    public static final int kBackRightCancoderPort = 0;
    public static final int kGyroPort = 50;               // CAN ID — change to yours

    // Magnet offsets (rotations) — measure with Phoenix Tuner X, change to yours.
    public static final double kFrontLeftMagnetOffset = 0.272949;
    public static final double kFrontRightMagnetOffset = 0.170898;
    public static final double kBackLeftMagnetOffset = 0.114502;
    public static final double kBackRightMagnetOffset = -0.418213;

    public static final double kDriveGearRatio = 6.75;                 // rotor : wheel
    public static final double kWheelDiameterMeters = 0.1016;          // 4 inch wheel
    public static final double kWheelCircumferenceMeters =
        Math.PI * kWheelDiameterMeters;                                // ≈ 0.319 m

    public static final double kHalfLength = 0.3;  // meters, wheelbase / 2
    public static final double kHalfWidth  = 0.3;  // meters, track width / 2

    // +X is forward, +Y is left, yaw is CCW-positive.
    public static final Translation2d kFrontLeft  = new Translation2d( kHalfLength,  kHalfWidth);
    public static final Translation2d kFrontRight = new Translation2d( kHalfLength, -kHalfWidth);
    public static final Translation2d kBackLeft   = new Translation2d(-kHalfLength,  kHalfWidth);
    public static final Translation2d kBackRight  = new Translation2d(-kHalfLength, -kHalfWidth);
  }

  public static final class SteerConstants {
    public static final double kP = 0.008;          // from Lesson 5 — retune once the real gearing lands
    public static final double kSteerGearRatio = RobotBase.isSimulation() ? 25 : 1; // rotor : steering
    public static final InvertedValue kSteerInverted = InvertedValue.CounterClockwise_Positive; // flip if your steering counts backward
  }

  public static final class HeadingConstants {
    public static final double kP = 0.02; // turn power per degree of heading error
  }
}
