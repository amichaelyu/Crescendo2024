package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.TilterConstants;

public class CG_IntakeIndexer extends ParallelCommandGroup {
    public CG_IntakeIndexer() {
        super(
                new IndexerRun(),
                new ShooterSetpointSpeed(0),
                new TilterSetpointPosition(TilterConstants.GROUND_INTAKE_POSITION),
                new IntakeRun()
        );
    }
}