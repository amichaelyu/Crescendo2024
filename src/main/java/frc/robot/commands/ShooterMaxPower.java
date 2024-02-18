package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;


public class ShooterMaxPower extends Command {
    private final Shooter shooter;
    private double distance;

    public ShooterMaxPower(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooter.setSpeed(ShooterConstants.MAX_SPEED);
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