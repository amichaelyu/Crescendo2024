package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


public class ShooterSmartdashboardSpeed extends Command {
    private final Shooter shooter;
    private double speed = 0;

    public ShooterSmartdashboardSpeed(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(this.shooter);
        SmartDashboard.putNumber("shooter speed", speed);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooter.setSpeed(SmartDashboard.getNumber("shooter speed", 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}