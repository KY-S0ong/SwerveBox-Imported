package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    // private final SimpleMotorFeedforward driveMotFF = new
    // SimpleMotorFeedforward(Constants.ks, Constants.kv);

    private final ProfiledPIDController proTurn = new ProfiledPIDController(Constants.kPTurning, 0, 0,
            new TrapezoidProfile.Constraints(80, 140));
    private final PIDController proDrive = new PIDController(Constants.kPDriving, 0, 0);
    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(Constants.ks, Constants.kv);
    double driveVolts;
    double turnVolts;
    double delta;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);
        TalonFXConfiguration driveConfigurationuration = new TalonFXConfiguration();
        TalonFXConfiguration turninConfiguration = new TalonFXConfiguration();

        // driveConfigurationuration.MotorOutput.Inverted = driveMotorReversed == false
        // ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        // turninConfiguration.MotorOutput.Inverted = turnMotorReversed == false ?
        // InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        driveConfigurationuration.Voltage.PeakForwardVoltage = 12;
        driveConfigurationuration.Voltage.PeakReverseVoltage = 12;
        driveConfigurationuration.CurrentLimits.StatorCurrentLimit = 100;
        driveConfigurationuration.CurrentLimits.SupplyCurrentLimit = 40;
        driveConfigurationuration.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfigurationuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        turninConfiguration.Voltage.PeakForwardVoltage = 10;
        turninConfiguration.Voltage.PeakReverseVoltage = 10;
        turninConfiguration.CurrentLimits.StatorCurrentLimit = 95;
        turninConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        turninConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        turninConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        driveMotor = new TalonFX(driveMotorId);
        turnMotor = new TalonFX(turnMotorId);
        driveMotor.getConfigurator().apply(driveConfigurationuration);
        turnMotor.getConfigurator().apply(turninConfiguration);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);
        turnMotor.setNeutralMode(NeutralModeValue.Coast);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);

        proTurn.enableContinuousInput(-Math.PI, Math.PI);
        reset_encoders();
    }

    public ParentDevice getDriveMotor() {
        return driveMotor;
    }

    public ParentDevice getTurnMotor() {
        return turnMotor;
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * Constants.WHEELRADIUS * Constants.kdriveGearRation * 2
                * Math.PI;
    }

    public double getTurningPosition() {
        return (turnMotor.getPosition().getValueAsDouble() * 2 * Math.PI * Constants.kdriveGearRation) % (2 * Math.PI);
    }

    public double getDriveVelocity() {
        return (Units.rotationsToRadians(driveMotor.getVelocity().getValueAsDouble()) * Constants.WHEELRADIUS
                * Constants.kdriveGearRation);
    }

    public double getTurningVelocity() {
        return turnMotor.getVelocity().getValueAsDouble() * 2 * Math.PI;
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);

    }

    public void reset_encoders() {
        driveMotor.setPosition(0);
        turnMotor.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        // return new SwerveModuleState(getDriveVelocity(), new
        // Rotation2d(getTurningPosition()));
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        try {
            if (Math.abs(state.speedMetersPerSecond) < 0.1) {
                stop();
                return;
            }
        } catch (Exception e) {
            System.err.println("state m/s likely null");
        }
        // angle corrected (no more negatives)
        state = new SwerveModuleState(state.speedMetersPerSecond, new Rotation2d(-state.angle.getRadians()));
        delta = state.angle.getDegrees() - getState().angle.getDegrees();
        if (Math.abs(delta) > 90.0) {
            state = new SwerveModuleState(-state.speedMetersPerSecond,
                    new Rotation2d((state.angle.getRadians() + Math.PI)));
        }
        driveVolts = driveFF.calculate(state.speedMetersPerSecond)
                + proDrive.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        turnVolts = proTurn.calculate(getState().angle.getRadians(), state.angle.getRadians());
        driveMotor.setVoltage(driveVolts);
        turnMotor.setVoltage(turnVolts);

    }

    public void setAutoDesiredState(SwerveModuleState state) {
        try {
            if (Math.abs(state.speedMetersPerSecond) < 0.1) {
                stop();
                return;
            }
        } catch (Exception e) {
            System.err.println("state m/s likely null");
        }
        // angle corrected (no more negatives)
        state = new SwerveModuleState(state.speedMetersPerSecond, new Rotation2d(-state.angle.getRadians() + Math.PI));
        delta = state.angle.getDegrees() - getState().angle.getDegrees();
        if (Math.abs(delta) > 90) {
            state = new SwerveModuleState(-state.speedMetersPerSecond,
                    new Rotation2d((state.angle.getRadians() + Math.PI)));
        }
        driveVolts = driveFF.calculate(state.speedMetersPerSecond)
                + proDrive.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        turnVolts = proTurn.calculate(getState().angle.getRadians(), state.angle.getRadians());
        driveMotor.setVoltage(driveVolts);
        turnMotor.setVoltage(turnVolts);

    }

    public void resetStates(){
        turnVolts = proTurn.calculate(getState().angle.getRadians(), 0);
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

}
