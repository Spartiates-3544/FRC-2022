package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SpinUpShooterCommand extends CommandBase {

  private PIDController upperPID;
  private PIDController lowerPID;
  //private SimpleMotorFeedforward feedforward;
  double targetVelocity;
  ShooterSubsystem shooter;

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public SpinUpShooterCommand(double targetVelocity, ShooterSubsystem shooter) {
    lowerPID = new PIDController(Constants.ShooterConstants.kPLower, 0, 0);
    upperPID = new PIDController(Constants.ShooterConstants.kPUpper, 0, 0);
    this.targetVelocity = targetVelocity;
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      System.out.println("SpinUpShooterCommand started!");
  }

  @Override
  public void execute() {
    shooter.setMotors(9600 * Constants.ShooterConstants.feedforward);
    //shooter.setMotors(upperPID.calculate(shooter.getUpperShooterVelocity(), 9624), 0);
    //SmartDashboard.putNumber("PID Output shooter", lowerPID.calculate(shooter.getLowerShooterVelocity(), 9624));
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("SpinUpShooterCommand ended!");
    shooter.setMotors(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}