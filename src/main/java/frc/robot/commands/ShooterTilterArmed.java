package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;


public class ShooterTilterArmed extends Command {
    private final Shooter shooter = Shooter.getInstance();
    private final Tilter tilter = Tilter.getInstance();
    private final Timer timer = new Timer();

    public ShooterTilterArmed() {}

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return (tilter.atSetpoint() && shooter.atSetpoint()) || timer.hasElapsed(3);
    }
}