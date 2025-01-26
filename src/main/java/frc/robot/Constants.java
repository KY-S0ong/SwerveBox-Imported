// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double kPTurning = 2;// 10
  public static final double kPDriving = 0.1;
  public static final double ks = 0.145;
  public static final double kv = 2.32;
  public static final double WHEELRADIUS = 0.0508; //

  /*------------------------------ Swerve Modules ---------------------------------------------------- */

  public static final int kFrontLeftDriveMotorPort = 10;
  public static final int kFrontLeftTurningMotorPort = 20;
  public static final boolean kFrontLeftDriveEncoderReversed = true; // false
  public static final boolean kFrontLeftTurningEncoderReversed = false; // false
  public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
  public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 2.6291; // 5.749
  public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false; // false

  public static final int kFrontRightDriveMotorPort = 11;
  public static final int kFrontRightTurningMotorPort = 21;
  public static final boolean kFrontRightDriveEncoderReversed = false; // true
  public static final boolean kFrontRightTurningEncoderReversed = false; // false
  public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
  public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 2.087; // 5.238
  public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false; // false

  public static final int kBackLeftDriveMotorPort = 12;
  public static final int kBackLeftTurningMotorPort = 22;
  public static final boolean kBackLeftDriveEncoderReversed = true; // true
  public static final boolean kBackLeftTurningEncoderReversed = false; // false
  public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
  public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 5.439; // 5.402
  public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false; // false

  public static final int kBackRightDriveMotorPort = 13;
  public static final int kBackRightTurningMotorPort = 23;
  public static final boolean kBackRightDriveEncoderReversed = false; // true
  public static final boolean kBackRightTurningEncoderReversed = false; // false
  public static final int kBackRightDriveAbsoluteEncoderPort = 3;
  public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.64; // 4.882
  public static final boolean kBackRightDriveAbsoluteEncoderReversed = false; // false

  /*------------------------------ DriveTrain Defined ---------------------------------------------------- */

  public static final double kWheelBase = Units.inchesToMeters(15.88);
  public static final double kTrackWidth = Units.inchesToMeters(15.88);

  public static final Translation2d leftFrontModule = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
  public static final Translation2d leftBackModule = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
  public static final Translation2d rightFrontModule = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);
  public static final Translation2d rightBackModule = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);

  public static ModuleConfig moduleConfig = new ModuleConfig(WHEELRADIUS, 4, 1, DCMotor.getFalcon500(1), 8.14, 40, 1);
  public static RobotConfig robotConfig = new RobotConfig(26, 9.675, moduleConfig,
      leftFrontModule, rightFrontModule, leftBackModule, rightBackModule);
  
    public static PathConstraints constraints = new PathConstraints(
            Constants.kPhysicalMaxSpeedMetersPerSecond, 
            Constants.kTeleDriveMaxAccelerationUnitsPerSecond, 
            Constants.kTeleDriveMaxAngularSpeedRadiansPerSecond, 
            Constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      leftFrontModule,
      leftBackModule,
      rightFrontModule,
      rightBackModule);

  /*------------------------------ Module Perameters ---------------------------------------------------- */

  public static final double kPhysicalMaxSpeedMetersPerSecond = 4;
  public static final double kPThetaController = 0.3;
  public static final double kThetaControllerConstraints = 0;
  public static final double kPXController = 0.05;
  public static final double kPYController = 0.05;
  public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
  public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 4;
  public static final double kDeadband = 0.12;
  public static final double kTeleDriveMaxSpeedMetersPerSecond = 3;
  public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 4;
  public static final double kdriveGearRation = 0.1481481481;// 0.1481481481 = 6.75:1
  

  /*------------------------------ Camera  ---------------------------------------------------- */
  public static final Pose3d kFarTargetPose = new Pose3d();
  public static final Transform3d kCameraToRobot = new Transform3d();
  public static final Matrix<N3, N1> StandardDev = null;
  public static final Matrix<N3, N1> visionStandardDev = null;
  public static final String kCamName = null;

  public static final String limeLight = "dalight";
  /*------------------------------ Autonomous P Values ---------------------------------------------------- */
  public static final double xP = 0.00001;
  public static final double yP = 0.0001;
  public static final double thetaP = 0.0001;

  // driving feed forward

  /*------------------------------ Xbox Constants ---------------------------------------------------- */
  public static final int buttonA = 1;
  public static final int buttonB = 2;
  public static final int buttonX = 3;
  public static final int buttonY = 4;
  public static final int buttonLB = 5;
  public static final int buttonRB = 6;
  public static final int buttonStart = 7;

  // pneumatics
  public static final int PCM = 2;

}
