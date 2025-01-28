package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.playSong;
import frc.robot.commands.zeroHeading;
import frc.robot.commands.driving.FOCdrivingCommand;
import frc.robot.commands.driving.resetEncodersCommnad;
import frc.robot.subsystems.PDPSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.visionSystems;

public class RobotContainer {

    private final XboxController xc = new XboxController(0);
    //private final PS4Controller ps = new PS4Controller(0);
    //private final Joystick joystick = new Joystick(0);

    private SendableChooser<String> autoChooser;
    private SendableChooser<String> songChooser;
    private SendableChooser<String> playSong;
    private String songChoice = "blackbetty.chrp";
    private boolean playFunc = false;

    private final visionSystems Cameras = new visionSystems();
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(songChoice);
    
    private final FOCdrivingCommand focDrive = new FOCdrivingCommand(
            swerveSubsystem,
            () -> - xc.getLeftY(),
            () -> xc.getLeftX(),
            () -> xc.getRightX(),
            () -> !xc.getLeftBumperButton(),
            () -> xc.getRightBumperButton());

    /*private final FOCdrivingCommand focDrive = new FOCdrivingCommand(
        swerveSubsystem, 
        ()-> -joystick.getY(), 
        ()-> joystick.getX(), 
        ()-> joystick.getZ(), 
        ()-> !joystick.getRawButton(1), 
        ()-> joystick.getRawButton(2)
    );*/
        
    /*private final FOCdrivingCommand driveCommand = new FOCdrivingCommand(
        swerveSubsystem, 
        ()-> ps.getLeftX(), 
        ()-> ps.getLeftY(), 
        ()-> ps.getRightX(), 
        ()-> !ps.getL3Button(), 
        ()-> ps.getL2Button());*/

    
    public RobotContainer() {
        
        swerveSubsystem.setDefaultCommand(focDrive);
        configureButtonBindings();
        registerCommands();


        autoChooser = new SendableChooser<String>();
        
        autoChooser.addOption("TestRun", "TestRun");
        autoChooser.addOption("Calibration", "Calibration");
        autoChooser.setDefaultOption("New Path", "New Path");

        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        
        
    }

    private void registerCommands() {
        NamedCommands.registerCommand("zero", new zeroHeading(swerveSubsystem));
    }

    private void configureButtonBindings() {
        new JoystickButton(xc, Constants.buttonA).whileTrue(new zeroHeading(swerveSubsystem));
        new JoystickButton(xc, Constants.buttonB).whileTrue(new resetEncodersCommnad(swerveSubsystem));
        new JoystickButton(xc, Constants.buttonY).toggleOnTrue(new playSong(swerveSubsystem, playFunc));
        //new JoystickButton(joystick, 3).whileTrue(new zeroHeading(swerveSubsystem));
        //new JoystickButton(joystick, 4).whileTrue(new resetEncodersCommnad(swerveSubsystem));
        //new JoystickButton(joystick, 5).toggleOnTrue(new playSong(swerveSubsystem, playFunc));
        
    }

    public Command getAutonomousCommand() {
        /*try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(autoChooser.getSelected());
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }*/

        try {
             return swerveSubsystem.followPathCommand(autoChooser.getSelected());
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
       
    }

}