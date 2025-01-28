package frc.robot.commands.driving;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class FOCdrivingCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction, preciseDriveFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    double xSpeed;
    double ySpeed;
    double turningSpeed;

    ChassisSpeeds chassisSpeeds;
    SwerveModuleState[] moduleStates;

    public FOCdrivingCommand(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction,
            Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction,
            Supplier<Boolean> preciseDriveFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.preciseDriveFunction = preciseDriveFunction;
        this.xLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        xSpeed = xSpdFunction.get();
        ySpeed = ySpdFunction.get();
        turningSpeed = turningSpdFunction.get();

        if (preciseDriveFunction.get() == true) {
            xSpeed = xSpeed / 2.9;
            ySpeed = ySpeed / 2.9;
            turningSpeed = turningSpeed / 1.5;
        }

        // 2. Apply deadband
        xSpeed = (Math.abs(xSpeed) > Constants.kDeadband) ? xSpeed : 0;
        ySpeed = (Math.abs(ySpeed) > Constants.kDeadband) ? ySpeed : 0;
        turningSpeed = (Math.abs(turningSpeed) > .1) ? turningSpeed : 0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * Constants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * Constants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds

        if (fieldOrientedFunction.get()) {
            // Relative to field 
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }
        // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        // x Speed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        SmartDashboard.putNumber("chassis desired speed x", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("chassis speed y", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("chassis turn", chassisSpeeds.omegaRadiansPerSecond);

        // 5. Convert chassis speeds to individual module states
        moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}