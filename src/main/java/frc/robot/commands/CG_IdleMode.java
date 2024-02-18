package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.TilterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;

public class CG_IdleMode extends ParallelCommandGroup {
    public CG_IdleMode(Shooter shooter, Tilter tilter) {
        super(
                new ShooterVoltage(shooter, 0),
                new TilterSetpointPosition(tilter, TilterConstants.IDLE_POSITION)
        );
    }
}