// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.Math;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class armManual extends Command {

  // TODO: move the controller to the RobotContainer.java file
  private final XboxController controller = new XboxController(0);

  //private final ExampleSubsystem m_subsystem;
  private final arm m_arm;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public armManual(arm arm) {
    addRequirements(arm);
    m_arm = new arm();
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.manual(controller.getLeftY()/4);
    SmartDashboard.putNumber("outP",controller.getLeftY()/4);
    //if (controller.getLeftY() > 0.5){
    //  m_arm.manual(0.25);
    //}
    //if (controller.getLeftY() < -1){
    //  m_arm.manual(-0.25);
    //}
    //if (controller.getLeftY() == 0){
    //  m_arm.stop();
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}