package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
    private final CANSparkMax motor1;
    private final CANSparkMax motor2;
    private final CANSparkMax motor3;

    public MotorSubsystem() {
        motor1 = new CANSparkMax(2, MotorType.kBrushless);
        motor2 = new CANSparkMax(13, MotorType.kBrushless);
        motor3 = new CANSparkMax(3, MotorType.kBrushless);
    }

    public void setMotor2Speed(double speed) {
        motor2.set(speed);
    }

    @Override
    public void periodic() {
        // nothing rn
    }
}
