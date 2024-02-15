package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class Tilter extends SubsystemBase {
  private final TalonFX m_tilterMotor = new TalonFX(tilterMotorID);

  /** Creates a new Tilter. */
  public Tilter() {
    

    m_tilterMotor.getConfigurator().apply(new TalonFXConfiguration());

    m_tilterMotor.setInverted(true);
    m_tilterMotor.setNeutralMode(null);
    
  }

  
  public void move(double pwr) {
   
    m_tilterMotor.set(pwr);

  }


 
  public void stop() {
    m_tilterMotor.stopMotor();
  }

 
}