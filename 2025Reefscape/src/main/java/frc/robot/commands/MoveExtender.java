// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MoveExtender extends Command {
  private Extender extender;
  private double inches;
  /** Creates a new GoOut. */
  public MoveExtender(Extender extender, double inches) {
    this.extender = extender;
    this.inches = inches;
    addRequirements(this.extender);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    extender.moveTo(inches);

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (extender.getMaxSensor()){
      extender.stopMotor();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return extender.atDistance() || extender.getMaxSensor();
  }
}
