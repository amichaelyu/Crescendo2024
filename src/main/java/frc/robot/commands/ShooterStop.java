package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class ShooterStop extends Command {
    private final Shooter shooter = Shooter.getInstance();

    public ShooterStop() {
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooter.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return shooter.getVelocity() < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0);
    }
}