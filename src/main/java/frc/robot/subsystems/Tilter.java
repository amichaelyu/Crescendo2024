package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TilterConstants;

public class Tilter extends SubsystemBase {
  private final TalonFX m_tilterMotor = new TalonFX(TilterConstants.tilterMotorID);
  private boolean isHomed;
  private DigitalInput limitSwitchTop;
  private final DigitalInput limitSwitchBottom;

  /**
   * Creates a new Tilter.
   */
  public Tilter() {
    limitSwitchBottom = new DigitalInput(TilterConstants.kLIFTER_LIMIT_BOTTOM);

    m_tilterMotor.getConfigurator().apply(TilterConstants.talonFXConfigs);
    m_tilterMotor.setPosition(0);

    TilterConstants.talonFXConfigs.HardwareLimitSwitch.ForwardLimitRemoteSensorID = TilterConstants.kLIFTER_LIMIT_BOTTOM;
    m_tilterMotor.setNeutralMode(NeutralModeValue.Brake);

    m_tilterMotor.setInverted(true);
    isHomed = false;
//    SmartDashboard.putNumber("tilter voltage", 0);
    SmartDashboard.putNumber("tilter p", 0);
  }

  @Override
  public void periodic() {
//      setVoltage(SmartDashboard.getNumber("tilter voltage", 0));
    SmartDashboard.putNumber("tilter rotations", m_tilterMotor.getPosition().getValue());

    TalonFXConfiguration tempConfig = TilterConstants.talonFXConfigs;
    tempConfig.Slot0.kP = SmartDashboard.getNumber("tilter p", 0);
    m_tilterMotor.getConfigurator().apply(tempConfig);

    if (isAtBottom()) {
      homed();
      System.out.println("Lifter at Bottom; not going down.");
    }
  }

  public void setVoltage(double voltage) {
      if (isAtBottom() && voltage < 0) {
        m_tilterMotor.setControl(new VoltageOut(0));
        System.out.println("Lifter at Bottom; not going down.");
      }
      else{
        m_tilterMotor.setControl(new VoltageOut(voltage));
      }
    }

  public void resetEncoder() {
    m_tilterMotor.setPosition(0);
  }

  public void homed() {
    isHomed = true;
//    TalonFXConfiguration config = TilterConstants.talonFXConfigs;
//    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
//    m_tilterMotor.getConfigurator().apply(config);
    resetEncoder();
  }

  public void setPosition(double position) {
    if (isHomed) {
      if (m_tilterMotor.getPosition().getValue() > position) {
        TalonFXConfiguration config = TilterConstants.talonFXConfigs;
        config.Slot0.kS = 1;
        m_tilterMotor.getConfigurator().apply(config);
      }
      else {
        TalonFXConfiguration config = TilterConstants.talonFXConfigs;
        config.Slot0.kS = 3;
        m_tilterMotor.getConfigurator().apply(config);
      }
      m_tilterMotor.setControl(new MotionMagicVoltage(position));
    }
  }

  public void move(double pwr) {
    if (isAtBottom() && pwr < 0) {
      m_tilterMotor.set(0);
      System.out.println("Lifter at Bottom; not going down.");
    }
    else{
      m_tilterMotor.set(pwr);
    }
  }

 
  public void stop() {
    m_tilterMotor.stopMotor();
  }
	public boolean isAtBottom() {
		return !limitSwitchBottom.get();
    }
  }