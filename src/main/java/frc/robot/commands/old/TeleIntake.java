package frc.robot.commands.old;



import frc.robot.subsystems.Intake;


import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.*;


public class TeleIntake extends Command {

      private final Intake m_intake = Intake.getInstance();
 


  public TeleIntake() {
      addRequirements(m_intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    m_intake.move(IntakeConstants.INTAKE_FORWARD);

  }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
     }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use the
    // scheduler to end the command when the button is released.
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the wheels when the command ends.
    m_intake.stop();
  }
}