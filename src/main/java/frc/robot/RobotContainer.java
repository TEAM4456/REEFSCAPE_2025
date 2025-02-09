package frc.robot;

import java.io.IOException;
import java.time.Instant;
import java.util.function.BooleanSupplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.FollowPathCommand;
//import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Vision;
import frc.robot.Commands.toggleSpeed;
import frc.robot.Subsystems.ElevatorPivot;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.IntakePivot;
import frc.robot.Subsystems.Climber;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, Commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController second = new CommandXboxController(1);



  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value ;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
 // private final JoystickButton zeroGyro =
 //     new JoystickButton(driver, XboxController.Button.kY.value);
 //private final JoystickButton robotCentric =
 //     new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  /* Subsystems */
  private final Vision vision = new Vision();
  private final Swerve s_Swerve = new Swerve(vision);
  private final Elevator elevator = new Elevator();
  private final ElevatorPivot elevatorPivot = new ElevatorPivot();
  private final IntakePivot intakePivot = new IntakePivot();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();

  private final SendableChooser<Command> chooser;



  /** The container for the robot. Contains subsystems, OI devices, and Commands. */
  
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis)));

    //Puts Sendable Chooser on SmartDashboard
    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto:", chooser);


    // Configure the button bindings
    configureButtonBindings();  
  }

  //Create Automated Commands here that make use of multiple subsystems [can be used in autonomous or teleop]
    //(ex. auto coral station pickup: moves elevator and elevator pivot)
    //See Crescendo's code for examples

    
    //Create othe commands that require multiple subsystems here

    public Command stopMotorsAll(){
      return new ParallelCommandGroup(
        elevator.elevatorStopCommand(),
        elevatorPivot.elevatorPivotStopCommand(),
        intakePivot.intakePivotStopCommand(),
        intake.leftIntakeStopCommand(),
        intake.rightIntakeStopCommand(),
        climber.climberStopCommand()
      );
    }


  //Create Autonomous Routines here (sequences for first 15s of match)
  //See Crescendo's code for examples

    public Command testAutotoAprilTag13(){
      try {
        return new SequentialCommandGroup(
          AutoBuilder.followPath(PathPlannerPath.fromPathFile("Move Towards April Tag 13")),
          new WaitCommand(1)
        );
      } catch (FileVersionException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      } catch (ParseException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
      return null;
    }

    public Command aprilTag13Auto (){
      return new PathPlannerAuto("April Tag 13 Auto");
    }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */



  //Add options for autonomous routines so that they appear in sendable chooser on SmartDashboard
  //https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html 
  private void configureButtonBindings() {
    chooser.setDefaultOption("nothing", null);
    
    chooser.addOption("Test Auto", new PathPlannerAuto("TestAuto"));
    chooser.addOption("Move Towards April Tag 13", testAutotoAprilTag13());
    //chooser.addOption("Center 1-2",autoCenter12());
    //add rest of autonomous routines here
    
    
    /* Driver Buttons */
    //Driver #1
    driver.back().toggleOnTrue(
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)));
    
    driver.start().whileTrue(stopMotorsAll());
    //XBOX CODE FOR ELEVATOR UP AND ELEVATOR DOWN, RIGHT TRIGGER RAISES
    //THE ELEVATOR AND LEFT TRIGGER LOWERS THE ELEVATOR, a BUTTON STOPS THE ELEVATOR 
    //CHANGE MADE SATURDAY 1-25

    driver.rightTrigger().whileTrue(elevator.elevatorUpCommand());
    //driver.rightTrigger().whileFalse(elevator.elevatorStopCommand());
    driver.leftTrigger().whileTrue(elevator.elevatorDownCommand());
    

    //Driver #2
    second.back().toggleOnTrue(
      new toggleSpeed(
        s_Swerve,
        () -> -second.getRawAxis(translationAxis),
        () -> -second.getRawAxis(strafeAxis),
        () -> -second.getRawAxis(rotationAxis)));

    second.start().whileTrue(stopMotorsAll());
  }
  public Swerve getSwerve(){
    return s_Swerve;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  

  public Command getAutonomousCommand() {

    s_Swerve.resetModulesToAbsolute();
    return chooser.getSelected();
    //return null;
  }


}
