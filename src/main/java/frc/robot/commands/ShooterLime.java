package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;


public class ShooterLime extends Command {
    private final Shooter shooter = Shooter.getInstance();
    private final Limelight limelight = Limelight.getInstance();

    public ShooterLime() {
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.setSpeed(ShooterConstants.shooterMap.get(limelight.distanceToTarget()));
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