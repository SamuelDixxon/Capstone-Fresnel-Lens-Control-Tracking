package com.example.cap;
import android.content.Intent;
import android.os.Bundle;
import android.widget.Button;

import androidx.appcompat.app.AppCompatActivity;

public class ApplicationInformation extends AppCompatActivity {


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_application_information);
        Button GoBack = findViewById(R.id.B5);
        GoBack.setOnClickListener((view -> openMain()));
    }

    public void openMain() {
        Intent intent = new Intent(this, com.example.cap.MainActivity.class);
        startActivity(intent);
    }


}