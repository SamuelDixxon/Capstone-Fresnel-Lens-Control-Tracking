package com.example.cap;
import android.content.Intent;
import android.os.Bundle;
import android.widget.Button;
import androidx.appcompat.app.AppCompatActivity;

public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        Button button_analytics = findViewById(R.id.B1);
        button_analytics.setOnClickListener((view -> openAnalytics()));

        Button button_info = findViewById(R.id.B2);
        button_info.setOnClickListener((view -> openInfo()));

    }

    public void openAnalytics() {
        Intent intent = new Intent(this, Analytics.class);
        startActivity(intent);
    }

    public void openInfo() {
        Intent intent2 = new Intent(this, ApplicationInformation.class);
        startActivity(intent2);
    }

}