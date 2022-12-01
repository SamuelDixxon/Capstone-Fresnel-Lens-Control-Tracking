package com.example.cap;
import android.content.Context;
import android.content.Intent;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.os.Bundle;
import android.provider.Settings;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ValueEventListener;
import androidx.annotation.NonNull;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;

public class Analytics extends AppCompatActivity { // Analytics activity class for displaying sensor data

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_analytics); // setting our content view


        Button GoBack = findViewById(R.id.B6); // Button go back gives the option to return to the previous activity page
        // Listener gives the actuation for when the button is pressed
        // openMain takes user back to the previous activity
        GoBack.setOnClickListener((view -> openMain()));

        TextView Sensor = findViewById((R.id.SR)); // Sensor text view for seeing which sensor corresponds to data reading
        TextView Readings = findViewById((R.id.SR2)); // Sensor reading for seeing which reading corresponds to sensor
        // This is pulled from the google cloud server, where the google firebase data is stored

        DatabaseReference databaseReference; // creating a reference to our database class through database API / package defined above

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

        databaseReference = FirebaseDatabase.getInstance().getReference().child("Sensors"); // get the reference to the database

        // calling add value event listener method
        // for getting the values from database.
        databaseReference.addListenerForSingleValueEvent(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                // this method is call to get the realtime
                // updates in the data.
                // this method is called when the data is
                // changed in our Firebase console.
                // below line is for getting the data from
                // snapshot of our database.
                String value = "Sensor Key";
                String readings = "Sensor Readings";

                // Iterative scheme to parse the readings for magentometer/phtodiode
                for ( DataSnapshot snapshot : dataSnapshot.getChildren() ) {
                    value = value + "\n\n" + snapshot.getKey(); // get the value
                    readings = readings + "\n\n";
                    for ( DataSnapshot snapshot2 : snapshot.child("reading").getChildren() ) {
                        value = value + "\n" + snapshot2.getKey();
                        readings = readings + "\n" +  snapshot2.getValue();
                    }
                }

                // after getting the value we are setting
                // our value to our text view in below line.
                Sensor.setText(value); // update Sensor text view
                Readings.setText(readings); // update Readings text view
            }

            @Override
            public void onCancelled(@NonNull DatabaseError error) {
                // calling on cancelled method when we receive
                // any error or we are not able to get the data.
                Toast.makeText(Analytics.this, "Fail to get data.", Toast.LENGTH_SHORT).show();
            }
        });
    }

    public void openMain() { // This function is exercised by the goBack button , to take the user back to main activity page
        Intent intent = new Intent(this, com.example.cap.MainActivity.class); // instatiate intent class to open new activity page
        startActivity(intent); // start the new activity page
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
            if (ni.getTypeName().equalsIgnoreCase("WIFI")) // Ignoring case (non case-sensitive) chekc for wifi
                if (ni.isConnected())
                    haveConnectedWifi = true; // retract assumption, as network method informs otherwise
            if (ni.getTypeName().equalsIgnoreCase("MOBILE")) // Ignoring case (non case-sensitive) chekc for Mobile
                if (ni.isConnected())
                    haveConnectedMobile = true; // retract assumption, as network method informs otherwise
        }

        if ( ! ( haveConnectedWifi || haveConnectedMobile ) ) {
            Toast.makeText(Analytics.this, "Must connect to wifi to use this activity page", Toast.LENGTH_SHORT).show();
        }
        return haveConnectedWifi || haveConnectedMobile; // returning or for the two boolean states, as eiter implies a network connection
    }
}