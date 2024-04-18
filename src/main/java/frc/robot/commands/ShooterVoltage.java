package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class ShooterVoltage extends Command {
    private final Shooter shooter = Shooter.getInstance();
    private final double voltage;

    public ShooterVoltage(double voltage) {
        addRequirements(this.shooter);
        this.voltage = voltage;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooter.setVoltage(voltage);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setVoltage(0);
    }
}