package com.example.cap;
import android.content.Intent;
import android.os.Bundle;
import android.widget.Button;
import androidx.appcompat.app.AppCompatActivity;

public class MainActivity extends AppCompatActivity {
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState); // load the super class that is saved when left activity
        setContentView(R.layout.activity_main); // set the content view from layout

        Button button_analytics = findViewById(R.id.B1); // instantiate analytics button with id from xml
        button_analytics.setOnClickListener((view -> openAnalytics())); // actuate button to open analytics activity page

        Button button_info = findViewById(R.id.B2); // instantiate information button with id from xml
        button_info.setOnClickListener((view -> openInfo())); //  actuate button to open information activity page

        Button button_controls = findViewById(R.id.B3); // instantiate controls button with id from xml
        button_controls.setOnClickListener((view -> openControls())); // actuate button to open controls activity page

    }
    public void openAnalytics() { // Open Activity for Analytics
        Intent intent = new Intent(this, Analytics.class); //  create new intent object
        startActivity(intent); // start the intent
    }
    public void openInfo() { // Open Activity for Application Information // create new intent object
        Intent intent2 = new Intent(this, ApplicationInformation.class);
        startActivity(intent2); // start the intent
    }
    public void openControls() { // Open Activity for Manual Controls
        Intent intent3 = new Intent(this, ManualControls.class); // create new intent object
        startActivity(intent3); // start the new intent
    }
}