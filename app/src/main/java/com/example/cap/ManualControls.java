package com.example.cap;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.os.Bundle;
import android.widget.Button;

public class ManualControls extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_manual_controls);
        Button GoBack = findViewById(R.id.B6);
        GoBack.setOnClickListener((view -> openMain()));
    }

    public void openMain() {
        Intent intent = new Intent(this, com.example.cap.MainActivity.class);
        startActivity(intent);
    }

}