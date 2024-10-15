package frc.robot.modules;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    public SwerveModule(int driveMotorId, int steerMotorId, int steerEncoderId) {
        driveMotor = new TalonFX(driveMotorId);
        steerMotor = new TalonFX(steerMotorId);
        steerEncoder = new CANcoder(steerEncoderId);

        steerEncoder.getConfigurator().apply(new CANcoderConfiguration());

        driveMotor.getConfigurator().apply(new TalonFXConfiguration());

        // drive motor config
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Slot0.kP = DriveConstants.DRIVE_P;
        driveConfig.Slot0.kI = DriveConstants.DRIVE_I;
        driveConfig.Slot0.kD = DriveConstants.DRIVE_D;
        driveMotor.getConfigurator().apply(driveConfig);

        // steer motor config
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.Slot0.kP = DriveConstants.STEER_P;
        steerConfig.Slot0.kI = DriveConstants.STEER_I;
        steerConfig.Slot0.kD = DriveConstants.STEER_D;
        steerMotor.getConfigurator().apply(steerConfig);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getSteerAngle());

        double velocityRPS = state.speedMetersPerSecond / DriveConstants.WHEEL_CIRCUMFERENCE;
        driveMotor.setControl(new VelocityVoltage(velocityRPS));

        double steerPositionRotations = state.angle.getRotations();
        steerMotor.setControl(new PositionVoltage(steerPositionRotations));
    }

    private Rotation2d getSteerAngle() {
        return Rotation2d.fromRotations(steerEncoder.getAbsolutePosition().getValue());
    }
}
