package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;
import android.media.MediaPlayer.OnCompletionListener;
import android.media.SoundPool;
import android.provider.MediaStore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Nathan on 1/15/17.
 */
@Autonomous(name = "OHMYGOD IT PLAYS SOUND!!!", group = "Nathan's Junk")
// @Disabled
public class OhMyGodItPlaysSound extends LinearOpMode {

    public SoundPool.Builder mySound;
    public int soundID;

    @Override
    public void runOpMode() throws InterruptedException {
        MediaPlayer mp = MediaPlayer.create(hardwareMap.appContext, R.raw.doom);

        waitForStart(); // of course, no false starts in this place.

        mp.setOnCompletionListener(new OnCompletionListener() {

            @Override
            public void onCompletion(MediaPlayer mp) {
                // makes sure the MediaPlayer gets cleaned up
                mp.reset();
                mp.release();
                mp = null;
            }

        });
        mp.start();
    }

}
