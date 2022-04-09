// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.regex.MatchResult;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import frc.robot.util.RevColorSensor;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexorSubsystem;

public class RunIndexorColorCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IndexorSubsystem indexor;
  private final ColorMatch colorMatch;
  private final Color kBlue;
  private final Color kRed;
  private final RevColorSensor colorSensor;

  public RunIndexorColorCommand(IndexorSubsystem indexor) {
    this.indexor = indexor;
    colorMatch = new ColorMatch();
    colorSensor = new RevColorSensor();
    kBlue = new Color(0.169, 0.414, 0.416);
    kRed = new Color(0.513, 0.352, 0.135);
    colorMatch.addColorMatch(kBlue);
    colorMatch.addColorMatch(kRed);
    addRequirements(indexor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexor.setPourcentage(0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexor.setPourcentage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    ColorMatchResult matchResult = colorMatch.matchClosestColor(colorSensor.getColor());
    if (matchResult.color == kBlue && matchResult.confidence >= 0.96) {
        return true;
    }
    if (matchResult.color == kRed && matchResult.confidence >= 0.96) {
        return true;
    }
    return false;
  }
}
