package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TilterConstants;

public class Tilter extends SubsystemBase {
  private final TalonFX m_tilterMotor = new TalonFX(TilterConstants.tilterMotorID);
  private boolean isHomed;

  /** Creates a new Tilter. */
  public Tilter() {
    m_tilterMotor.getConfigurator().apply(TilterConstants.talonFXConfigs);
    m_tilterMotor.setNeutralMode(NeutralModeValue.Brake);
    m_tilterMotor.setInverted(true);
    isHomed = false;
  }

  public void home() {
    m_tilterMotor.setControl(new VoltageOut(-1));
  }

  public void setPosition(double position) {
    if (isHomed) {
      m_tilterMotor.setControl(new MotionMagicVoltage(position));
    }
  }

  public void move(double pwr) {
    m_tilterMotor.set(pwr);
  }


 
  public void stop() {
    m_tilterMotor.stopMotor();
  }

 
}