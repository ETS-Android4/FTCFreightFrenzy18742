package org.firstinspires.ftc.teamcode;

/**
 * Class that turns a usual gamepad button into an ON/OFF switch
 */
public class ButtonSwitch {
    private boolean lastButtonState = false;
    private boolean switchState = false; //initial state is OFF

    /**
     * Change switch state to the opposite if button press is detected
     * @param buttonState current button state (gamepadN.xxx)
     * @return ON/OFF state
     */
    public boolean updateSwitchState(boolean buttonState) {
        if (buttonState && !lastButtonState)
            switchState = !switchState;
        lastButtonState = buttonState;
        return switchState;
    }
}
