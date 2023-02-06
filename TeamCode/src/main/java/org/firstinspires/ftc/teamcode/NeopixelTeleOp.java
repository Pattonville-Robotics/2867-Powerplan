package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.encoders.ClawEncoder;
import org.firstinspires.ftc.teamcode.encoders.LinearSlideEncoder;
import org.firstinspires.ftc.teamcode.encoders.MecanumEncoder;

import com.diozero.api.I2CConstants;
import com.diozero.api.RuntimeIOException;
import com.diozero.devices.HD44780Lcd;
import com.diozero.devices.LcdConnection;
import com.diozero.devices.LcdConnection.PCF8574LcdConnection;
import com.diozero.util.Diozero;
import com.diozero.util.SleepUtil;

@TeleOp
public class NeopixelTeleOp extends LinearOpMode {

    int COLS = 0; // amt in columns in led grid
    int ROWS = 0; // amt in rows in led grid

    @Override
    public void runOpMode() throws InterruptedException {

        final GamepadEx controller1 = new GamepadEx(gamepad1);

        int device_address = LcdConnection.PCF8574LcdConnection.DEFAULT_DEVICE_ADDRESS;
        int controller = I2CConstants.CONTROLLER_1;

        // Initialise display
        LcdConnection lcd_connection = new PCF8574LcdConnection(controller, device_address);
        HD44780Lcd lcd = new HD44780Lcd(lcd_connection, COLS, ROWS);
//        try (LcdConnection lcd_connection = new PCF8574LcdConnection(controller, device_address);
//             HD44780Lcd lcd = new HD44780Lcd(lcd_connection, COLS, ROWS)) {
//               LcdApp.test(lcd);
//        } catch (RuntimeIOException e) {
//            Logger.error(e, "Error: {}", e);
//        } finally {
//            // Required if there are non-daemon threads that will prevent the
//            // built-in clean-up routines from running
//            Diozero.shutdown();
//        }

        // begin main logic
        lcd.setBacklightEnabled(true);

        byte[] smilie = HD44780Lcd.Characters.get("smilie");
        lcd.createChar(1, smilie);

        // face buttons
        if (gamepad2.a) ;
        if (gamepad2.x) ;
        if (gamepad2.y) ;
        if (gamepad2.b) ;



        telemetry.update();
        }
    }