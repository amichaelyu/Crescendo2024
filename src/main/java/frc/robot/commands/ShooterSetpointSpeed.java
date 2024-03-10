package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class ShooterSetpointSpeed extends Command {
    private final Shooter shooter = Shooter.getInstance();
    private final double speed;

    public ShooterSetpointSpeed(double speed) {
        addRequirements(this.shooter);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        shooter.setSpeed(speed);
    }

    @Override
    public void execute() {
        shooter.setSpeed(speed);
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