package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TilterConstants;

public class CG_ShootingAmp extends ParallelDeadlineGroup {
    public CG_ShootingAmp() {
        super(
                new SequentialCommandGroup(
                        new ShooterTilterArmed(),
                        new IndexerKick()
                ),
                new ShooterSetpointSpeed(ShooterConstants.AMP_SPEED),
                new TilterSetpointPosition(TilterConstants.AMP_POSITION)
        );
    }
}