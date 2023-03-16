package frc.robot.util;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;

public final class ControllerIO {
    private static final ControllerType controller = OIConstants.controllerType;

    public static enum ControllerType {
        XBOX,
        LOGITECH,
        JOYSTICK
    }

    public static int getY() {
        switch (controller) {
            case XBOX:
                return 1;
            case LOGITECH:
                return 1;
            case JOYSTICK:
                return 1;
            default:
                return 1;
        }
    }

    public static int getX() {
        switch (controller) {
            case XBOX:
                return 0;
            case LOGITECH:
                return 0;
            case JOYSTICK:
                return 0;
            default:
                return 0;
        }
    }

    public static int getRot() {
        switch (controller) {
            case XBOX:
                return 4;
            case LOGITECH:
                return 2;
            case JOYSTICK:
                return 2;
            default:
                return 2;
        }
    }

    public static int inversionY() {
        switch (controller) {
            case XBOX:
                return 1;
            case LOGITECH:
                return 1;
            case JOYSTICK:
                return -1;
            default:
                return 1;
        }
    }

    public static int inversionX() {
        switch (controller) {
            case XBOX:
                return 1;
            case LOGITECH:
                return 1;
            case JOYSTICK:
                return -1;
            default:
                return 1;
        }
    }

    public static int inversionRot() {
        switch (controller) {
            case XBOX:
                return 1;
            case LOGITECH:
                return 1;
            case JOYSTICK:
                return 1;
            default:
                return 1;
        }
    }

    public static int getTrigger() {
        switch (controller) {
            case XBOX:
                return XboxController.Button.kBack.value;
            case LOGITECH:
                return 1;
            case JOYSTICK:
                return 1;
            default:
                return 1;
        }
    }
}

