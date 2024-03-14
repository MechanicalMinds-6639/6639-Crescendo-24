// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;

public class LightCommand extends Command {
  /** Creates a new LightCommand. */
  private final Lights mLights;

  public LightCommand(Lights lights) {
    // Use addRequirements() here to declare subsystem dependencies.
    mLights = lights;
    addRequirements(mLights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mLights.indicateSensor();
    SmartDashboard.putBoolean("Sensor", mLights.getSensorValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
