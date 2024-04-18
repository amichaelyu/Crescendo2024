package frc.robot.commands;


import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoBuilderWithWait extends SequentialCommandGroup {
    public AutoBuilderWithWait(String autoName, double waitTimeSeconds) {
        super(
                new WaitCommand(waitTimeSeconds),
                AutoBuilder.buildAuto(autoName)
        );
    }
}