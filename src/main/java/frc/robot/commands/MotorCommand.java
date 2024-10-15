package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorSubsystem;

public class MotorCommand extends Command {
    private final MotorSubsystem m_Subsystem;

    public MotorCommand(MotorSubsystem subsystem) {
        m_Subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("started");
    }

    @Override
    public void execute() {
        m_Subsystem.setMotor2Speed(0);
        System.out.println("motor command ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
