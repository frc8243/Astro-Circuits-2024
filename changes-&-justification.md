## What is this "parting gift" & Why it Exists
I'm graduating and while I'm excited for the future, I would like to let something I've noticed be known. I haven't gotten a chance to properly **teach**, I've really only been able to **handhold** through problems. 

Whilst this allowed us to have an unimaginably successful 2024 season, I feel that if I can teach more, aid more, and get more people to do what I love doing for this team and for this program, you (we?) will become so much more successful, maybe even making it to playoffs, or **beyond**!

I wanted to leave the team (and myself) an example of some robot code that ***properly balances both function and form***. Right now, the code isn't in the prettiest state, nor is it the most functional. While I may not be able to test a lot of the functions considering I don't have a complete robot in my workspace, the code provided should hopefully show what a good working and good looking robot project looks like, at least in my opinion.

Below my little letter will be an explaination of every change, and coming seperately will be my thoughts on *Better Utilization of Robot Vision*, since that doesn't fit with this letter. Hopefully this allows for better code and more success to be had in the future!

Thanks for a great three years,\
Signing out *(as a student)*\
Julien

## Changes
### General Minor Changes
- `@SupressWarnings('unused')` in ***every*** file. Unused imports aren't a source of robot issues, these warnings just serve as an annoyance
- `TODO` comments left around for additional potential minor QoL changes. 

### Simulation.
The worst thing about our switch to swerve was the new inability to simulate at least the *minor* functions of the robot. 2399 figured out swerve simulation, so that will be borrowed while trying to maintain the  current functionality of our robot. This requires a restructuring of the `Drivetrain` subsystem and folder, with `Drivetrain` not changing based on whether the robot is running in sim or not, but `SwerveModule` changing. The `updateInputs()` method of `SwerveModuleIO` will be called periodically. 


### Pose Estimation & Pose Handling

The `Drivetrain` subsystem should handle the robot pose and reporting where it is on the field, not the `Vision` subsystem. `Vision` should report to `Drivetrain`, where `Drivetrain` will report to the other subsystems and logs.

Additionally, the code currently relies on *all three* cameras being connected to the robot, so when one is disconnected or malfunctioning we have **no** vision based pose estimation. That's not great. 


### Splitting Constants into Multiple Files
Mananging the many different classes in [Constants](src\main\java\frc\robot\Constants.java) has been a source of about 30 different headaches of ours, from finding random values to solving merge conflicts. I saw (somewhere) that some teams split their constants into seperate files, as you would most classses. My approach is similar, with grouping each subsystems' constants into one file, such as `ModuleConstants` and `DriveConstants` being in `DrivetrainConstants` This should allow editing of multiple subsystems' constants on ***the same branch*** without merge conflicts.

Various things, such as `ConfigConstants`, which contains info on what motors or gyroscope is used, and `MotorConstants` which contains various stats for the various motors don't fit into a subsystem specifically, so they'll go in `RobotConstants`, 

### Moving Various Commands into Command Factories in One File
This one is purely to aid anyone reading this who is new to coding. The keyword `new` is a bit confusing, especially in its use in Java. Here's an example
```
driverController.rightBumper().onTrue(new TurnToSource(m_drivetrain, m_leds, driverController, m_alliance));
```

Looking at this, one would think that when `rightBumper()` returns true, a *new* `TurnToSource` is created, when in fact every time this button is pressed, its doing the same action.

Compare this to what it could look like using a Factory in a `driveCommands` file.

```
driverController.rightBumper().onTrue(DriveCommands.TurnToSource());
```
Much simpler, right? `DriveCommands`, when created in `RobotContainer`, already is given the parameters used for every command used for the drivetrain, which allows for much simpler code to be written. This should allow easier collaboration between drivers and programmers, since they can more easily understand the `configureButtonBindings()` method in `RobotContainer`

### SparkMAXs
Where do I even begin?\
We create these motor controllers in many ways, sometimes we were boring and used this, and created the object in one line
```
private static CANSparkMax shootMotor = new CANSparkMax(ShooterConstants.kShootMotorID, MotorType.kBrushless);
```
Other times we created them in the constructor of the subsystem, like this
```
public RollerClawReal() {
        rollerClawMotor = new CANSparkMax(RollerClawConstants.kRollerClawMotorID, MotorType.kBrushless);
        rollerClawEncoder = rollerClawMotor.getEncoder();
        rollerClawMotor.restoreFactoryDefaults();
        rollerClawMotor.setIdleMode(IdleMode.kBrake);
        rollerClawMotor.setSmartCurrentLimit(MotorConstants.kNEO550CurrentLimit);
        rollerClawMotor.burnFlash();
    }
```
And only *once*, we use the (in my opinion) best way to create these motors, which enforces a current limit. `MotorUtil.createSparkMAX` Every SparkMAX created on this branch will use this function, which I have adjusted just a tad to make using it simpler.

### Vision Commands
Vision is ***hard***.\
Relying on solely the camera output is not reliable, as seen at Miami Valley, where glare cost us many many shots and the LED strip on the left side of the robot.

Instead of relying on the direct camera output, we should rely on *odometry*, which is reliable even if camera output is lost for momentary amounts of time. I'll go more into this in a seperate document, but this requires some new thinking for commands.

Using our `SwerveDrivePoseEstimator`, all of the data from the wheels and the cameras on the robot is fed into an estimated robot pose. If this pose is accurate (it should be barring a wheel falling off and throwing off the wheel odometry, which in theory should cause the pose estimator to reject that module's odometry since its far out of line of the others), we can use the Robot's pose to trigger the shooter prespin or the `inRange` boolean, as well as giving us a ***constant*** angle to turn to *relative to the field*, not losing tracking when the motion blur of the camera causes the target to be lost.

The new `TurnToSpeaker` and `TurnToAmp` commands will use this strategy.

### Logging
Logging is arguably the most important function of our code. It allows us to see exactly what was inputted, what the robot did, and how long it took. Since we're not at the required programming experience for AdvantageKit, there's only a some minor improvements we can do.

We should log when the robot is doing something autonomously, whether its moving or turning or shooting. There's a clear lack of data regarding what the driver is commanding aside from the speeds of the robot, which is unfortunate.

Any driver assistance function, such as the `TurnTo(X)` commands, the `preSpinShooter` command, or anything else will put a boolean in the `DriverAssists` tree in SmartDashboard. an example would be
```
SmartDashboard.putBoolean("DriverAssists/PreSpinning", preSpinning);
```

The state of the LEDs should be logged as well to aid the human player in defending themselves for a mistaken drop, this can also be used to display which call is active in Elastic to compare against actual LED color.

### Prespin.
Our haphazard implementation of a prespin gave us many headaches, relying on the direct camera output is a bad idea, as previously stated, so what should we do? 

1. Use our Estimated Pose to determine if the Robot is in our alliance's wing
2. Give the driver a manual button to hold to prespin the shooter
3. Make sure the operator knows the shooter is spun up using LEDS *and* controller vibration

This should use a boolean in `Shooter`, called `preSpin`, when `preSpin` is true, the *front wheel* on the shooter should spin as hard as it can, or in other words run at `1` speed. There is a case for using specific RPMs for our shooter, but since we have a simple shooter design, there's really no point.