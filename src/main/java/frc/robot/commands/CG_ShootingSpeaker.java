package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TilterConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;

public class CG_ShootingSpeaker extends ParallelCommandGroup {
    public CG_ShootingSpeaker(Shooter shooter, Tilter tilter, Indexer indexer) {
        super(
                new ShooterSetpointSpeed(shooter, ShooterConstants.SPEAKER_SPEED),
                new TilterSetpointPosition(tilter, TilterConstants.SPEAKER_CENTER_POSITION),
                new SequentialCommandGroup(
                        new ShooterTilterArmed(shooter, tilter),
                        new IndexerKick(indexer)
                )
        );
    }
}