// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix6.hardware.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;


//test
public class arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final PIDController pid = new PIDController(1,0,0); // change for talonFX pid 
  
  private final TalonFX motor;
  private final TalonFX follower;
  private final CANcoder encoder;

  private final PositionVoltage positioner = new PositionVoltage(0);
  //final TrapezoidProfile m_profile = new TrapezoidProfile(
  // new TrapezoidProfile.Constraints(80, 160)
  //);
 //. TrapezoidProfile.State m_goal = new TrapezoidProfile.State(10, 0);
 // TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private final DutyCycleOut ds = new DutyCycleOut(0).withOverrideBrakeDurNeutral(true);

  public arm() {
    //right bicep
    motor = new TalonFX(42); //change back to 42
    encoder = new CANcoder(43); 

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 2.4;
    motor.getConfigurator().apply(slot0Configs,0.050);
    positioner.Slot = 0;
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.Feedback.FeedbackRemoteSensorID = 43;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motor.getConfigurator().apply(cfg);
    //left bicep
    follower = new TalonFX(41);
    follower.setControl(new Follower(42, true));
  }

  public void armToPosition(double targetPosition){
    SmartDashboard.putString("taggyvontaggems", "asdfghjkl");
    SmartDashboard.putNumber("Closed Loosp Output", motor.getClosedLoopOutput().getValue());
    SmartDashboard.putNumber("closed loop reference", motor.getClosedLoopReference().getValue());
    SmartDashboard.putNumber("closed loop error", motor.getClosedLoopError().getValue());
    motor.setControl(positioner.withPosition(targetPosition));
    //m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);
    // apply the setpoint to the control request
    //positioner.Position = m_setpoint.position;
    //positioner.Velocity = m_setpoint.velocity;
    //motor.setControl(positioner);
  }

  public void manual(double movement){
    motor.setControl(ds.withOutput(movement));
  }

  public double position(){
    return motor.getPosition().getValueAsDouble();
  }

  public void stop(){
    motor.set(0);
  }

  public void resetPosition() {
    encoder.setPosition(0);
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */

  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}