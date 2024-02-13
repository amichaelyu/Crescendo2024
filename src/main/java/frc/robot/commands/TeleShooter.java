package frc.robot.commands;



import frc.robot.subsystems.Shooter;


import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.*;


public class TeleShooter extends Command {

      private final Shooter m_shooter;

 


  public TeleShooter(Shooter shooter) {

  
      m_shooter = shooter;
      addRequirements(m_shooter);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    m_shooter.launch(kShooterLaunchSpeed);
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
    m_shooter.stop();
  }
}