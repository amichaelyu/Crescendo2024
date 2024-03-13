package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TilterConstants;

public class CG_FlushShots extends ParallelCommandGroup {
    public CG_FlushShots() {
        super(
                new TilterSetpointPosition(TilterConstants.GROUND_INTAKE_POSITION),
                new ShooterSetpointSpeed(ShooterConstants.INTAKE_SPEED),
                new IntakeReverse(),
                new IndexerFlush()
        );
    }
}