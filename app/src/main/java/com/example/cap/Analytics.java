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

public class Analytics extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_analytics);

        //TextView location_text;
        Button GoBack = findViewById(R.id.B6);
        GoBack.setOnClickListener((view -> openMain()));

        TextView Sensor = findViewById((R.id.SR));
        TextView Readings = findViewById((R.id.SR2));


        DatabaseReference databaseReference;

        boolean connect = haveNetworkConnection();
        AlertDialog.Builder builder;
        if (!connect) {
            builder = new AlertDialog.Builder(this);
            builder.setMessage("Connect to wifi or quit")
                    .setCancelable(false)
                    .setPositiveButton("Connect to WIFI", (dialog, id) -> startActivity(new Intent(Settings.ACTION_WIFI_SETTINGS)))
                    .setNegativeButton("Quit", (dialog, id) -> openMain());
            AlertDialog alert = builder.create();
            alert.show();
            //location_text = findViewById(R.id.location);
        }

        databaseReference = FirebaseDatabase.getInstance().getReference().child("PhotoDiode");


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
                for ( DataSnapshot snapshot : dataSnapshot.getChildren() ) {
                   value = value + "\n" + snapshot.getKey();
                   readings = readings + "\n" +  snapshot.getValue();
                }

                // after getting the value we are setting
                // our value to our text view in below line.
                Sensor.setText(value);
                Readings.setText(readings);
            }

            @Override
            public void onCancelled(@NonNull DatabaseError error) {
                // calling on cancelled method when we receive
                // any error or we are not able to get the data.
                Toast.makeText(Analytics.this, "Fail to get data.", Toast.LENGTH_SHORT).show();
            }
        });


    }

    public void openMain() {
        Intent intent = new Intent(this, com.example.cap.MainActivity.class);
        startActivity(intent);
    }

    private boolean haveNetworkConnection() {
        boolean haveConnectedWifi = false;
        boolean haveConnectedMobile = false;

        ConnectivityManager cm = (ConnectivityManager) getSystemService(Context.CONNECTIVITY_SERVICE);
        NetworkInfo[] netInfo = cm.getAllNetworkInfo();
        for (NetworkInfo ni : netInfo) {
            if (ni.getTypeName().equalsIgnoreCase("WIFI"))
                if (ni.isConnected())
                    haveConnectedWifi = true;
            if (ni.getTypeName().equalsIgnoreCase("MOBILE"))
                if (ni.isConnected())
                    haveConnectedMobile = true;
        }
        return haveConnectedWifi || haveConnectedMobile;
    }

}