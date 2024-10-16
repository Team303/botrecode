package frc.robot.modules;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;
    private final SimpleMotorFeedforward driveFeedForward;

    private double lastVelocity = 0;

    public SwerveModule(int driveMotorId, int steerMotorId, int steerEncoderId) {
        driveMotor = new TalonFX(driveMotorId);
        steerMotor = new TalonFX(steerMotorId);
        steerEncoder = new CANcoder(steerEncoderId);

        steerEncoder.getConfigurator().apply(new CANcoderConfiguration());
        // drive motor config
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Slot0.kP = ModuleConstants.DRIVE_P;
        driveConfig.Slot0.kI = ModuleConstants.DRIVE_I;
        driveConfig.Slot0.kD = ModuleConstants.DRIVE_D;
        driveConfig.Slot0.kS = ModuleConstants.DRIVE_S;
        driveConfig.Slot0.kV = ModuleConstants.DRIVE_V;
        driveConfig.Slot0.kA = ModuleConstants.DRIVE_A;
        driveConfig.Voltage.PeakForwardVoltage = DriveConstants.MAX_VOLTAGE;
        driveConfig.Voltage.PeakReverseVoltage = -DriveConstants.MAX_VOLTAGE;
        driveConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.DRIVE_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(driveConfig);

        // steer motor config
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.Slot0.kP = ModuleConstants.STEER_P;
        steerConfig.Slot0.kI = ModuleConstants.STEER_I;
        steerConfig.Slot0.kD = ModuleConstants.STEER_D;
        steerConfig.Slot0.kS = ModuleConstants.STEER_S;
        steerConfig.Slot0.kV = ModuleConstants.STEER_V;
        steerConfig.Voltage.PeakForwardVoltage = DriveConstants.MAX_VOLTAGE;
        steerConfig.Voltage.PeakReverseVoltage = -DriveConstants.MAX_VOLTAGE;
        steerConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.TURN_CURRENT_LIMIT;
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerMotor.getConfigurator().apply(steerConfig);

        driveFeedForward = new SimpleMotorFeedforward(ModuleConstants.DRIVE_S, ModuleConstants.DRIVE_V,
                ModuleConstants.DRIVE_A);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getSteerAngle());

        double velocityRPS = state.speedMetersPerSecond / DriveConstants.WHEEL_CIRCUMFERENCE;

        // acceleration
        double acceleration = (velocityRPS - lastVelocity) / 0.02; // 20ms loop time
        lastVelocity = velocityRPS;

        // calc feedfrowrad
        double feedforward = driveFeedForward.calculate(velocityRPS, acceleration);

        driveMotor.setControl(new VelocityVoltage(velocityRPS).withFeedForward(feedforward));

        double steerPositionRotations = state.angle.getRotations();
        steerMotor.setControl(new PositionVoltage(steerPositionRotations));
    }

    private Rotation2d getSteerAngle() {
        return Rotation2d.fromRotations(steerEncoder.getAbsolutePosition().getValue());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveMotor.getPosition().getValue() * DriveConstants.DRIVE_ENCODER_DISTANCE_PER_PULSE,
                getSteerAngle());
    }
}
