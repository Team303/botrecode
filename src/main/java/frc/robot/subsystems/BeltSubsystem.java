package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BeltConstants;

public class BeltSubsystem extends SubsystemBase {
    private final TalonFX beltMotor;
    private final TalonFX indexerMotor;
    private final DigitalInput fireballAndrew;

    private final DutyCycleOut beltDutyCycle = new DutyCycleOut(0);
    private final DutyCycleOut indexerDutyCycle = new DutyCycleOut(0);

    public BeltSubsystem() {
        beltMotor = new TalonFX(BeltConstants.BELT_MOTOR_ID);
        indexerMotor = new TalonFX(BeltConstants.INDEXER_MOTOR_ID);
        fireballAndrew = new DigitalInput(BeltConstants.BEAM_BREAK_SENSOR_PORT);
    }

    private void configureMotors() {
        beltMotor.setNeutralMode(NeutralModeValue.Brake);
        indexerMotor.setNeutralMode(NeutralModeValue.Brake);

        beltMotor.getConfigurator().apply(new TalonFXConfiguration());
        indexerMotor.getConfigurator().apply(new TalonFXConfiguration());

        beltMotor.getConfigurator();
    }
}
