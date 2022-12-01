import { initializeApp } from "https://www.gstatic.com/firebasejs/9.14.0/firebase-app.js";
import { getDatabase, ref, onValue } from "https://www.gstatic.com/firebasejs/9.14.0/firebase-database.js";

export default function apprun() {

    // Import the functions you need from the SDKs you need
    // TODO: Add SDKs for Firebase products that you want to use
    // https://firebase.google.com/docs/web/setup#available-libraries

    // Your web app's Firebase configuration
    // For Firebase JS SDK v7.20.0 and later, measurementId is optional
    const firebaseApp = initializeApp({
        apiKey: "AIzaSyD-r5pSqbAmOCJXReD2wzWpUTZDK0DSAy0",
        authDomain: "capstone-database-c7175.firebaseapp.com",
        databaseURL: "https://capstone-database-c7175-default-rtdb.firebaseio.com",
        projectId: "capstone-database-c7175",
        storageBucket: "capstone-database-c7175.appspot.com",
        messagingSenderId: "900141330072",
        appId: "1:900141330072:web:ca69e08bdc299b68f8eaed",
        measurementId: "G-QM1XBWXWKW"
    }); // initializing app based on specific parameters unique to the application

    const db = getDatabase(firebaseApp); // get the reference to the firebase
    const sensors = ref(db, "/Sensors");

    onValue(sensors, (snapsot) => { // creating onValue to  reference the parent nodes of the sensors

        const sensor1 = ref(db, "/Sensors/Magnetometer1/reading/"); // getting reference to the reading of sensor1
        var d1, d2, d3, x, y, z; // instantiating variables to hold the values for the sensor data

        onValue(sensor1, (snapshot) => { // when any child of sensor1 changes, get new data
            x = snapshot.val()["timestamp"]; // child node key-value pair get time stamp
            d1 = snapshot.val()["data"]; // child node key-value pair get the data
        });


        const sensor2 = ref(db, "/Sensors/Magnetometer2/reading/"); // getting reference reading of sensor2

        onValue(sensor2, (snapshot) => { // when any child of sensor2 changes, get new data
            y = snapshot.val()["timestamp"]; // child node key-value pair get time stamp
            d2 = snapshot.val()["data"]; // child node key-value pair get data
        });

        const sensor3 = ref(db, "/Sensors/PhotoDiode/reading/"); // getting reference to the reading of sensor3

        onValue(sensor3, (snapshot) => { // when any child of sensor2 changes, get new data
            z = snapshot.val()["timestamp"]; // child node key-value pair get time stamp
            d3 = snapshot.val()["data"]; // child node key-value pair get data
        });

        google.charts.load('current', { 'packages': ['table'] }); // load the google chart api table

        // set OnLoadCallback method is to draw the table once the new data is acquired
        google.charts.setOnLoadCallback(() => drawTable(d1, d2, d3, x, y, z));

    });


    function drawTable(d1, d2, d3, x, y, z) { // draw the table is called from setOnLoadCallback method
        // this actually draws the table in the html with visualization tool from api
        var data = new google.visualization.DataTable();
        data.addColumn('string', 'Sensor'); // column 1 for sensors
        data.addColumn('number', 'Reading'); // column 2 for the physical reading of sensors
        data.addColumn('string', 'Date'); // column 3 for the data of the reading
        data.addRows([ // add rows for each unique sensor
            ['Magnetometer1', d1, x], // similar to a dictionary Sensor, data, timestamp
            ['Magnetometer2', d2, y], // similar to a dictionary Sensor, data, timestamp
            ['Photodiode1', d3, z], // similar to a dictionary Sensor, data, timestamp
        ]);

        // connecting the html with the table
        var table = new google.visualization.Table(document.getElementById('table_div'));
        // drawing the table with the defined width and height
        table.draw(data, { showRowNumber: true, width: '80%', height: '50%' });
    };
};