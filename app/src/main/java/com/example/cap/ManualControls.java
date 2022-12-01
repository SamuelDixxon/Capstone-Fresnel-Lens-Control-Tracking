package com.example.cap;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;

import android.annotation.SuppressLint;
import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.os.Bundle;
import android.provider.Settings;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;

public class ManualControls extends AppCompatActivity {
    DatabaseReference databaseReference;
    boolean connected = false;
    TextView stats;
    Button Connect;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_manual_controls);

        boolean connect = haveNetworkConnection(); // call subroutine to see if tere is a connection to wifi, required for data pulls from web-server
        AlertDialog.Builder builder;
        if (!connect) { // if no wifi, create alert dialog to inform user this function is necessary for application functionality
            builder = new AlertDialog.Builder(this);
            builder.setMessage("Connect to wifi or quit")
                    .setCancelable(false)
                    .setPositiveButton("Connect to WIFI", (dialog, id) -> startActivity(new Intent(Settings.ACTION_WIFI_SETTINGS)))
                    .setNegativeButton("Quit", (dialog, id) -> openMain());
            AlertDialog alert = builder.create(); // creating alert builder
            alert.show(); // showing the content of the alert builder
        }

        Button GoBack = findViewById(R.id.B6); // Instantiate Main Return Button
        Connect = findViewById(R.id.B12); // Instantiate Connect to Device Button

        // All buttons below interact with the database flag reference node to
        // communicate with the ESP32 via wifi connection
        Button left = findViewById(R.id.B7); // Instantiate Left Motor Control Button
        Button right = findViewById(R.id.B8); // Instantiate Right Motor Control Button
        Button up = findViewById(R.id.B9); // Instantiate Up Motor Control Button
        Button down = findViewById(R.id.B10); // Instantiate Down Motor Control Button
        Button stop = findViewById(R.id.B11); // Instantiate Stop Motor Control Button

        stats = findViewById(R.id.status); // Instantiate the stats id

        GoBack.setOnClickListener((view -> openMain())); // Set button to go back to main
        Connect.setOnClickListener((view -> openConnection())); // Set button to connect to target
        left.setOnClickListener((view -> leftActuate())); // Set button to rotate left
        right.setOnClickListener((view -> rightActuate())); // Set button to rotate right
        up.setOnClickListener((view -> upActuate())); // Set button to tilt mirror up
        down.setOnClickListener((view -> downActuate())); // Set button to tilt mirror down
        stop.setOnClickListener((view -> Halt())); // Set button to stop all power to device

        // getting reference to the flags portion of the database
        databaseReference = FirebaseDatabase.getInstance().getReference().child("Flags");

    }

    public void openMain() { // Open Activity for MainActivity
        connected = false; // reset connected to be false
        databaseReference.child("Connect").setValue(0); // Reset the flag in database, as activity page is being left
        Intent intent = new Intent(this, com.example.cap.MainActivity.class);
        startActivity(intent); // start the new activity
    }

    public void openConnection() { // Open Activity for MainActivity
        connected = !connected;
        if (connected) {
            databaseReference.child("Connect").setValue(1); // update the flag in database
            stats.setText(R.string.Status2); // Set the string ot status 1 , or disconnected if we already set disconnected
            stats.setTextColor(Color.parseColor("#90EE90")); // set the color to green signifying connection
            Connect.setText(R.string.Status3);
        } else {
            databaseReference.child("Connect").setValue(0); // update the flag in database
            stats.setText(R.string.Status1); // Set the string to status 2 , or disconnected if we already set connect
            stats.setTextColor(Color.parseColor("#FFCCCB")); // set the color to red signifying no connection
            Connect.setText(R.string.Status4);
        }

    }

    public void leftActuate() { // Open Activity for MainActivity
        databaseReference.child("Left").setValue(1);
        // set the move left button flag to 1 in database
        // note the esp32 will latch this value for indicator of action and then update the value to 0
    }

    public void rightActuate() { // Open Activity for MainActivity
        databaseReference.child("Right").setValue(1);
        // set the move right button flag to 1 in database
        // note the esp32 will latch this value for indicator of action and then update the value to 0
    }

    public void upActuate() { // Open Activity for MainActivity
        databaseReference.child("Up").setValue(1);
        // set the move up button flag to 1 in database
        // note the esp32 will latch this value for indicator of action and then update the value to 0
    }

    public void downActuate() { // Open Activity for MainActivity
        databaseReference.child("Down").setValue(1);
        // set the move down button flag to 1 in database
        // note the esp32 will latch this value for indicator of action and then update the value to 0
    }

    public void Halt() {
        databaseReference.child("Stop").setValue(1); // Set the Halt flag in the database to true
    }

    private boolean haveNetworkConnection() {
        // initialize booleans assuming there is no wifi connection
        boolean haveConnectedWifi = false;
        boolean haveConnectedMobile = false;
        // Open connectivity manager and get the context for the system's wifi service
        ConnectivityManager cm = (ConnectivityManager) getSystemService(Context.CONNECTIVITY_SERVICE);
        // Create an array of network info to store the data from the connectivity manager getAllNetworkInfo method
        NetworkInfo[] netInfo = cm.getAllNetworkInfo();
        for (NetworkInfo ni : netInfo) {
            if (ni.getTypeName().equalsIgnoreCase("WIFI")) // Ignoring case (non case-sensitive) check for wifi
                if (ni.isConnected())
                    haveConnectedWifi = true; // retract assumption, as network method informs otherwise
            if (ni.getTypeName().equalsIgnoreCase("MOBILE")) // Ignoring case (non case-sensitive) check for Mobile
                if (ni.isConnected())
                    haveConnectedMobile = true; // retract assumption, as network method informs otherwise
        }

        if (!(haveConnectedWifi || haveConnectedMobile)) {
            Toast.makeText(ManualControls.this, "Must connect to wifi to use this activity page", Toast.LENGTH_SHORT).show();
        }

        return haveConnectedWifi || haveConnectedMobile; // returning or for the two boolean states, as either implies a network connection
    }
}
