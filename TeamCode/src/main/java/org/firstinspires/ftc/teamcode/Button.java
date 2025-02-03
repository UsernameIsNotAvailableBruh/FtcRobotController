package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.EnumMap;

// This button status thing is inspired by u/m0stlyharmless_user and u/fusionforscience on reddit from a post 8y ago :)
// also ryan if youre reading this i did NOT use ai for this, this is indeed my code believe it or not. I use used the links below to help out with the logic fr
// https://www.reddit.com/r/FTC/comments/5lpaai/comment/dbye175/?utm_source=share&utm_medium=web3x&utm_name=web3xcss&utm_term=1&utm_content=share_button
// https://www.reddit.com/r/FTC/comments/5lpaai/comment/dcerspj/?utm_source=share&utm_medium=web3x&utm_name=web3xcss&utm_term=1&utm_content=share_button
enum Status {
    notPressedYet,
    currentlyPressed,
    wasPressed
}
enum buttonName { //this and the buttonList can be optimized by only including buttons used.
    options,
    triangle,
    share,
    cross,
    square,
    circle,
    left_stick_button,
    right_stick_button,
    dpad_left,
    dpad_right,
    dpad_up,
    dpad_down,
    right_bumper,
    left_bumper,
    guide
}
enum Gpads { //GP stands for gamepad
    GP1,
    GP2
}
private class Buttons { //To add new buttons, just append it to the bottom of buttonName & append to buttonList.
    public  final EnumMap<buttonName, Status> buttonMap     = new EnumMap<>(buttonName.class);
    private final EnumMap<buttonName, Button> ButtonStorage = new EnumMap<>(buttonName.class);
    private Gpads GP;
    private final Gamepad gpad = new Gamepad();

    public Buttons(boolean isGamepad2) {
        GP = Gpads.GP1;
        if (isGamepad2) {
            GP = Gpads.GP2;
        }
        for (buttonName button : buttonName.values()) {
            ButtonStorage.put(button, new Button());
        }
    }

    public boolean wasPressedOrisPressed(buttonName button) {
        return buttonMap.get(button) == Status.wasPressed || buttonMap.get(button) == Status.currentlyPressed;
    }

    public boolean wasPressed(buttonName button) {
        return buttonMap.get(button) == Status.wasPressed;
    }

    public boolean isPressedCurrently(buttonName button) {
        return buttonMap.get(button) == Status.currentlyPressed;
    }

    public boolean NotPressed(buttonName button){
        return buttonMap.get(button) == Status.notPressedYet;
    }

    public void update(){
        if (GP == Gpads.GP1) {
            gpad.copy(gamepad1);
        }
        else {
            gpad.copy(gamepad2);
        }
        boolean[] buttonList = new boolean[]{gpad.options, gpad.triangle, gpad.share, gpad.cross, gpad.square, gpad.circle, gpad.left_stick_button, gpad.right_stick_button, gpad.dpad_left, gpad.dpad_right, gpad.dpad_up, gpad.dpad_down, gpad.right_bumper, gpad.left_bumper, gpad.guide};
        buttonName[] ButtonArr = buttonName.values();
        for (int i = 0; i< buttonList.length; i++) {
            buttonMap.put(ButtonArr[i], ButtonStorage.get(ButtonArr[i]).ButtonStatus(buttonList[i]));
        }
    }

    private class Button { //class in a class in a class for funsies
        private Status status = Status.notPressedYet;
        public Status ButtonStatus(boolean button) {
            if ( (button && status == Status.notPressedYet))
                status = Status.currentlyPressed;
            else if (!button && status == Status.currentlyPressed)
                status = Status.wasPressed;
            else if (status == Status.wasPressed)
                status = Status.notPressedYet;
            return status;
        }
    }
}