package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TilterConstants;

public class CG_ShootingSpeaker extends ParallelDeadlineGroup {
    public CG_ShootingSpeaker() {
        super(

                new ShooterSetpointSpeed(ShooterConstants.SPEAKER_SPEED),
                new TilterSetpointPosition(TilterConstants.SPEAKER_CENTER_POSITION),
                new SequentialCommandGroup(
                        new ShooterTilterArmed(),
                        new IndexerKick()
                )
        );
    }
}