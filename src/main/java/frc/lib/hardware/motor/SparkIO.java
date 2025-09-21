package frc.lib.hardware.motor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorOutputs;

import static com.revrobotics.spark.SparkBase.ControlType.*;

public class SparkIO extends MotorIO {
    private static class SparkController {
        public final SparkBase motor;
        public final SparkClosedLoopController controller;
        public final RelativeEncoder encoder;

        public SparkController(
            SparkBase motor
        ) {
            this.motor = motor;
            this.controller = motor.getClosedLoopController();
            this.encoder = motor.getEncoder();
        }
    }
    
    protected final SparkController main;
    protected final SparkController[] followers;

    protected SparkIO(SparkBase mainMotor, SparkBase... followers) {
        super(followers.length);
        
        main = new SparkController(mainMotor);

        this.followers = new SparkController[followers.length];

        for (int i = 0; i < followers.length; i++) {
            this.followers[i] = new SparkController(followers[i]);
        }
    }

    private static void loadOutputs(SparkController controller, MotorOutputs outputs) {
        outputs.statorCurrent = controller.motor.getOutputCurrent();

        // TODO: Figure out that stator current vs supply current actually is
        outputs.supplyCurrent = 0;

        outputs.voltage = controller.motor.getBusVoltage() * controller.motor.getAppliedOutput();
        outputs.position = controller.encoder.getPosition();
        outputs.velocity = controller.encoder.getVelocity();
        outputs.temperatureCelsius = controller.motor.getMotorTemperature();
    }

    @Override
    protected void updateOutputs(MotorOutputs[] outputs) {
        loadOutputs(main, outputs[0]);

        for (int i = 0; i < followers.length; i++) {
            loadOutputs(followers[i], outputs[i + 1]);
        }
    }

    @Override
    protected void setVoltage(double voltage) {
        main.controller.setReference(voltage, kVoltage);
    }

    @Override
    protected void setCurrent(double current) {
        main.controller.setReference(current, kCurrent);
    }

    @Override
    protected void setPosition(double position) {
        main.controller.setReference(position, kPosition);
    }

    @Override
    protected void setVelocity(double velocity) {
        main.controller.setReference(velocity, kVelocity);
    }
    
    @Override
    protected void setProfiledPosition(double position) {
        main.controller.setReference(position, kMAXMotionPositionControl);
    }

    @Override
    protected void setPercentage(double percentage) {
        main.controller.setReference(percentage, kDutyCycle);
    }

    @Override
    protected void setIdle() {
        setVoltage(0);
    }

    /**
     * Creates a SparkIO with a group of SparkMax motor controllers
     * @param type
     * @param main
     * @param followers
     * @return
     */
    public static SparkIO createSparkMaxIO(MotorType type, int main, int... followers) {
        SparkBase mainMotor = new SparkMax(main, type);

        SparkBase[] followerMotors = new SparkBase[followers.length];

        for (int i = 0; i < followers.length; i++) {
            followerMotors[i] = new SparkMax(followers[i], type);
        }

        return new SparkIO(mainMotor, followerMotors);
    }

    /**
     * Creates a SparkIO with a group of SparkMax motor controllers with the kBrushless motor type
     * @param main
     * @param followers
     * @return
     */
    public static SparkIO createSparkMaxIO(int main, int... followers) {
        return createSparkMaxIO(MotorType.kBrushless, main, followers);
    }


    /**
     * Creates a SparkIO with a group of SparkFlex motor controllers
     * @param type
     * @param main
     * @param followers
     * @return
     */
    public static SparkIO createSparkFlexIO(MotorType type, int main, int... followers) {
        SparkBase mainMotor = new SparkFlex(main, type);


        SparkBase[] followerMotors = new SparkBase[followers.length];

        for (int i = 0; i < followers.length; i++) {
            followerMotors[i] = new SparkFlex(followers[i], type);
        }

        return new SparkIO(mainMotor, followerMotors);
    }

    /**
     * Creates a SparkIO with a group of SparkFlex motor controllers with the kBrushless motor type
     * @param type
     * @param main
     * @param followers
     * @return
     */
    public static SparkIO createSparkFlexIO(int main, int... followers) {
        return createSparkFlexIO(MotorType.kBrushless, main, followers);
    }
}
