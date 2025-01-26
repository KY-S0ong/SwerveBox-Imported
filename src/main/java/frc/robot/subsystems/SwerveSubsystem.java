package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
//import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.geometry.struct.Rotation2dStruct;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Classifier;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.LimelightHelpers.PoseEstimate;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
            Constants.kFrontLeftDriveMotorPort,
            Constants.kFrontLeftTurningMotorPort,
            Constants.kFrontLeftDriveEncoderReversed,
            Constants.kFrontLeftTurningEncoderReversed,
            Constants.kFrontLeftDriveAbsoluteEncoderPort,
            Constants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            Constants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            Constants.kFrontRightDriveMotorPort,
            Constants.kFrontRightTurningMotorPort,
            Constants.kFrontRightDriveEncoderReversed,
            Constants.kFrontRightTurningEncoderReversed,
            Constants.kFrontRightDriveAbsoluteEncoderPort,
            Constants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            Constants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            Constants.kBackLeftDriveMotorPort,
            Constants.kBackLeftTurningMotorPort,
            Constants.kBackLeftDriveEncoderReversed,
            Constants.kBackLeftTurningEncoderReversed,
            Constants.kBackLeftDriveAbsoluteEncoderPort,
            Constants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            Constants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            Constants.kBackRightDriveMotorPort,
            Constants.kBackRightTurningMotorPort,
            Constants.kBackRightDriveEncoderReversed,
            Constants.kBackRightTurningEncoderReversed,
            Constants.kBackRightDriveAbsoluteEncoderPort,
            Constants.kBackRightDriveAbsoluteEncoderOffsetRad,
            Constants.kBackRightDriveAbsoluteEncoderReversed);

    private SwerveModulePosition[] swerveModPose = new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(),
            backLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            backRight.getSwerveModulePosition()
    };
    private Pigeon2 gyro = new Pigeon2(0);
    private Pose2d initPose = new Pose2d();
    private Field2d field = new Field2d();

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            Constants.kDriveKinematics,
            getRotation2d(), swerveModPose, initPose);

    

    public SwerveSubsystem() {
        
        AutoBuilder.configure(this::getPose,
                this::resetOdometry,
                this::getRelatChassisSpeeds,
                this::setStatesFromChassisSpeeds,
                new PPHolonomicDriveController(
                        new PIDConstants(9, 0, 0),
                        new PIDConstants(5, 0, 0)),
                Constants.robotConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    }

    public Command followPathCommand(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            
            return AutoBuilder.followPath(path);

        } catch (Exception e) {
            return Commands.none();
        }

    }

    public void updateSwerveModPose() {
        this.swerveModPose = new SwerveModulePosition[] {
                frontLeft.getSwerveModulePosition(),
                backLeft.getSwerveModulePosition(),
                frontRight.getSwerveModulePosition(),
                backRight.getSwerveModulePosition()
        };
    }

    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState() };
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kTeleDriveMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void setAutoModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setAutoDesiredState(desiredStates[0]);
        frontRight.setAutoDesiredState(desiredStates[1]);
        backLeft.setAutoDesiredState(desiredStates[2]);
        backRight.setAutoDesiredState(desiredStates[3]);
    }

    public void zeroStates(){
        frontLeft.resetStates();
        frontLeft.resetStates();
        backLeft.resetStates();
        backRight.resetStates();
    }

    public void reset_encoders() {
        // setModuleStates(restStates);
        frontLeft.reset_encoders();
        frontRight.reset_encoders();
        backLeft.reset_encoders();
        backRight.reset_encoders();
    }

    public ChassisSpeeds getRelatChassisSpeeds() {
        return Constants.kDriveKinematics.toChassisSpeeds(getStates());
    }

    public void setStatesFromChassisSpeeds(ChassisSpeeds speeds) {
        ChassisSpeeds p = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
                -speeds.omegaRadiansPerSecond * 1.3);
        setModuleStates(Constants.kDriveKinematics.toSwerveModuleStates(p));
        
    }
    // public void setStatesFromPoints(ChassisSpeeds speeds){
    // previousSetPoint = swerveSetpointGenerator.generateSetpoint(previousSetPoint,
    // speeds, 0.2);
    // }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 361);
    }

    public void zeroHeading() {
        gyro.reset();

    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        // return new Pose2d(new Translation2d(gyro.getDisplacementX(),
        // gyro.getDisplacementY()), getRotation2d());
        Pose2d dis = poseEstimator.getEstimatedPosition();

        var visionEstimatedPose = LimelightHelpers.getBotPose2d(Constants.limeLight);
        try {
            if (visionEstimatedPose != null) {
                poseEstimator.addVisionMeasurement(visionEstimatedPose,
                        Timer.getFPGATimestamp());
            }
        } catch (Exception e) {
        }

        return new Pose2d(new Translation2d(dis.getX() + initPose.getX(), dis.getY() + initPose.getY()),
                dis.getRotation());
    }

    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(
                getRotation2d(),
                swerveModPose,
                newPose);
    }

    public void resetOdometry(Pose2d pose) {
        // odometer.resetPosition(getRotation2d(),swerveModPose ,pose);
        poseEstimator.resetPosition(pose.getRotation(), swerveModPose, pose);
    }

    @Override
    public void periodic() {
        updateSwerveModPose();
        updateShuffleBoard();
        updateFieldLocation();
        updateTelemetry();

        poseEstimator.update(getRotation2d(), swerveModPose);
        Logger.recordOutput("MyStates", getStates());
        Logger.recordOutput("MyPose", getPose());

    }

    private void updateShuffleBoard() {
        SmartDashboard.putNumber("fl encoder angle", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("fr encoder angle", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("bl encoder angle", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("br encoder angle", backRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Gyro", getHeading());
        SmartDashboard.putNumber("fl velcity", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("fr velocity", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("bl velocity", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("br velocity", backRight.getDriveVelocity());
        SmartDashboard.putNumber("translation x", getPose().getX());
        SmartDashboard.putNumber("translation y", getPose().getY());
        SmartDashboard.putNumber("rotation", getPose().getRotation().getDegrees());

    }

    private StructPublisher<Pose2d> pose2dPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose2d.struct).publish();

    private StructArrayPublisher<SwerveModuleState> swervePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

    private void updateTelemetry() {
        pose2dPublisher.set(getPose());
        swervePublisher.set(getStates());
    }

    private void updateFieldLocation() {
        field.setRobotPose(getPose());
    }

    private Orchestra orchestra = new Orchestra();
    private AudioConfigs configs = new AudioConfigs();

    public void loadSong(String song) {
        configs.AllowMusicDurDisable = true;
        
        orchestra.addInstrument(frontLeft.getDriveMotor());
        orchestra.addInstrument(frontLeft.getTurnMotor());
        orchestra.addInstrument(frontRight.getDriveMotor());
        orchestra.addInstrument(frontRight.getTurnMotor());
        orchestra.addInstrument(backLeft.getDriveMotor());
        orchestra.addInstrument(backLeft.getTurnMotor());
        orchestra.addInstrument(backRight.getDriveMotor());
        orchestra.addInstrument(backRight.getTurnMotor());

        orchestra.loadMusic(song);

    }

    public void playSong() {
        orchestra.play();
    }

    public void pauseSong() {
        orchestra.pause();
    }
}