package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Indexer extends SubsystemBase {
  private final TalonFX m_indexMotor = new TalonFX(indexMotorID);

  /** Creates a new Indexer. */
  public Indexer() {
    

    m_indexMotor.getConfigurator().apply(new TalonFXConfiguration());

    m_indexMotor.setInverted(true);
    
  }

  
  public void move(double pwr) {
   
    m_indexMotor.set(pwr);

  }


 
  public void stop() {
    m_indexMotor.stopMotor();
  }

 
}