package com.example.cap;
import android.content.Intent;
import android.os.Bundle;
import android.widget.Button;

import androidx.appcompat.app.AppCompatActivity;

public class ApplicationInformation extends AppCompatActivity {
    // Application Information page for the application

    @Override
    protected void onCreate(Bundle savedInstanceState) { // onCreate method to be run once when application loads
        super.onCreate(savedInstanceState); // super method for saved instance of previous state
        setContentView(R.layout.activity_application_information); // set the content view for the activity
        Button GoBack = findViewById(R.id.B5); // find the GoBack button by the xml id
        GoBack.setOnClickListener((view -> openMain())); // actuate button to return to main
    }

    public void openMain() { // openMain method to return to the main activity page
        Intent intent = new Intent(this, com.example.cap.MainActivity.class); // instantiate the new intent
        startActivity(intent); // start the intent to return to previous page
    }


}