package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.ArmConstants

class ArmSubsystem(val topBreakerID: Int, val bottomBreakerID: Int, /*val elbowMotorID: Int,*/ val elevatorMotorID: Int) :SubsystemBase() {
    val elevatorMotor = CANSparkMax(elevatorMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
    var elevatorZeroed = false
    //val elbowMotor = CANSparkMax(elbowMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)


    val elevatorEncoder = elevatorMotor.getEncoder()
   // val elbowEncoder = elbowMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    //val forwardLimit = elevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    val reverseLimit = DigitalInput(bottomBreakerID);

    val forwardLimit = DigitalInput(topBreakerID)
   // var elbowPos = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
   // var elbowVelocity = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowVelocity").publish()
    var elevatorPos = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorPosition").publish()
    var elevatorVelocity = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorVelocity").publish()
    var rpmElevatorVelocity = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("rpmElevatorVelocity").publish()
    var ntDesiredElevator = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("DesiredElevator").publish()
    //var ntDesiredElbow = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("DesiredElbow").publish()

    var desiredElevatorState = 0.0
    //var desiredElbowState = 0.0

    var inst = NetworkTableInstance.getDefault()
    var table = inst.getTable("Arm")

   /// val elbowPid = PIDController(ArmConstants.elbowP, ArmConstants.elbowI, ArmConstants.elbowD)
    var elevatorPid = ProfiledPIDController(
        ArmConstants.elevatorP,
        ArmConstants.elevatorI,
        ArmConstants.elevatorD, ArmConstants.elevatorTrapezoidConstraints, 0.02)




    init {
       // elbowEncoder.setPositionConversionFactor(ArmConstants.elbowEncoderPosMultiplier)
       // elbowEncoder.setVelocityConversionFactor(ArmConstants.elbowEncoderVelocityMultiplier)
        elevatorEncoder.positionConversionFactor = ArmConstants.elevatorEncoderPositionConversionFactor
        elevatorEncoder.velocityConversionFactor = ArmConstants.elevatorEncoderVelocityMultiplier
    }
    /**
     * Sets the desired position
     *
     * The periodic function moves forever towards the position, so this is how you make it do something favorable. This basically tells the PID that it needs to go to elevatorPos
     *
     * @param[ElbowPos: Double] the new desired position for the elbow
     * @param[ElevatorPos: Double] the new desired position for the elevator
     */
    fun SetDesired(/*ElbowPos: Double,*/ ElevatorPos: Double)
    {

      //  desiredElbowState = ElbowPos

       // elevatorPid.reset(elevatorEncoder.position)
        elevatorPid.setConstraints(ArmConstants.elevatorTrapezoidConstraints)
        elevatorPid.setGoal(ElevatorPos)
    }
    /**
     * moves the arm towards the desired position
     *
     * The elevator needs to be zeroed for all of this.
     * Then, as long as the encoders are within the safe range, the motor moves according to the PID.
     */
    override fun periodic() {

        super.periodic()
        /** be setting the ntvalue for elbow position **/
        if (elevatorZeroed == true) {
            if(elevatorEncoder.position >= 0 || elevatorEncoder.position <= 0.85) {
                elevatorMotor.setVoltage(elevatorPid.calculate(elevatorEncoder.position))
            }
            else
            {
                elevatorMotor.set(0.0)
            }
           // elevatorPid.setpoint.position = desiredElevatorState
            // elbowPos.set(elbowEncoder.position)
            // ntDesiredElbow.set(desiredElbowState)
           // ntDesiredElevator.set(desiredElevatorState)
            //  println(elevatorEncoder.position)
            elevatorPos.set(elevatorEncoder.position)
            elevatorVelocity.set(elevatorEncoder.velocity)
            rpmElevatorVelocity.set(elevatorEncoder.velocity / ArmConstants.elevatorEncoderVelocityMultiplier)
            //these two ifs set the encoder when it hits a limit switch

            if (!reverseLimit.get()) //Line break false = it has been hit
            {
                // elevatorEncoder.position = ArmConstants.elevatorMinHeight
            } else {
            }

            if (forwardLimit.get()) {
                //elevatorEncoder.position = ArmConstants.elevatorMaxHeight
            }
        }
        /**
         * checks if or not elevator is at bottom
         *
         * looks to see if the bottom linebreak is hit
         *
         * @return true if it is at the bottom, otherwise false
         */
        fun bottomAligned(): Boolean {

            if (!reverseLimit.get()) {
                return true
            } else {
                return false
            }
        }
    }

    fun moveVoltage(volts: Double) {
        println("voltage")
        elevatorMotor.setVoltage(volts)
    }

}