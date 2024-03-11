package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants.TilterConstants;

public class CG_IntakeIndexer extends ParallelDeadlineGroup {
    public CG_IntakeIndexer() {
        super(
                new IndexerRun(),
                new ShooterSetpointSpeed(0),
                new TilterSetpointPositionThenPostition(TilterConstants.GROUND_INTAKE_POSITION, TilterConstants.IDLE_POSITION),
                new IntakeRun()
        );
    }
}