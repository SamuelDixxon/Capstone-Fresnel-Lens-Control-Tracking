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
    const sensors = ref(db, "/Sensors/");


    onValue(sensors, (snapshot) => { // creating onValue to  reference the parent nodes of the sensors
        var x = [];
        snapshot.forEach((c1) => { // parsing the data to get the snapshot for the sensors and readings
            c1.forEach((c2) => { // iterating across readings
                var toAppend = [c1.key]; // getting sensor key string value
                c2.forEach((c3) => { // getting the value of the 
                    toAppend.push(c3.val()); // pusing the value of the new readings children, ie data, unit, and 
                });
                x.push(toAppend); // appending row to data in order to write to table
            });

        });

        // below is tests ran to determine parsing scheme

        // const sensor1 = ref(db, "/Sensors/Magnetometer1/reading/"); // getting reference to the reading of sensor1
        // var d1, d2, d3, x, y, z; // instantiating variables to hold the values for the sensor data

        // onValue(sensor1, (snapshot) => { // when any child of sensor1 changes, get new data
        //     x = snapshot.val()["timestamp"]; // child node key-value pair get time stamp
        //     d1 = snapshot.val()["data"]; // child node key-value pair get the data
        // });


        // const sensor2 = ref(db, "/Sensors/Magnetometer2/reading/"); // getting reference reading of sensor2

        // onValue(sensor2, (snapshot) => { // when any child of sensor2 changes, get new data
        //     y = snapshot.val()["timestamp"]; // child node key-value pair get time stamp
        //     console.log(y);
        // });

        // const sensor3 = ref(db, "/Sensors/PhotoDiode/reading/"); // getting reference to the reading of sensor3

        // onValue(sensor3, (snapshot) => { // when any child of sensor2 changes, get new data
        //     z = snapshot.val()["timestamp"]; // child node key-value pair get time stamp
        //     d3 = snapshot.val()["data"]; // child node key-value pair get data
        // });

        // set OnLoadCallback method is to draw the table once the new data is acquired

        google.charts.setOnLoadCallback(() => drawTable(x)); // setting on callloadback method to update table with new data


    });

    const mag1 = ref(db, "/Sensors/Magnetometer1"); // getting reference to mag1

    onValue(mag1, (snapshot) => { // creating onValue to  reference the parent nodes of the magnetometer data sensor for line chart
        var y = [["Time", "Sensor"]]; // storing data in variable at top
        snapshot.forEach((c1) => { // parsing the data to get the snapshot for the sensors and readings
            var toAppend = [c1.val()["timestamp"], c1.val()["reading"]]; // getting (x,y) coordinate for line graph
            y.push(toAppend); // appending row to data in order to write to table
        });
        console.log(y);
        google.charts.setOnLoadCallback(() => drawChart1(y)); // setting on callloadback method to update table with new data
    });


};

//google.load('visualization', '1', { packages: ['controls'], callback: drawChart1 });
google.load('visualization', '1', { packages: ['controls'], callback: drawChart1 });
google.charts.load('current', { 'packages': ['corechart'] });


function drawTable(data1) {
    if (Array.isArray(data1)) {
        if (data1.length != 0) {

            var data = new google.visualization.DataTable();
            data.addColumn('string', 'Sensor');
            data.addColumn('number', 'Reading');
            data.addColumn('number', 'Timestamp');
            data.addColumn('string', 'Unit');

            // debug logs
            // console.log("Non-empty! ");
            // console.log(data1.length);
            // console.log(data1);

            data.addRows(data1); // adding parsed data array

            var dashboard = new google.visualization.Dashboard(document.querySelector('#dashboard'));

            var stringFilter = new google.visualization.ControlWrapper({
                controlType: 'StringFilter',
                containerId: 'string_filter_div',
                options: {
                    filterColumnIndex: 0
                }
            });

            var numberRangeFilter = new google.visualization.ControlWrapper({
                controlType: 'NumberRangeFilter',
                containerId: 'numnber_range_filter_div',
                options: {
                    filterColumnIndex: 2,
                    minValue: 0,
                    maxValue: 1000,
                    ui: {
                        label: 'Timestamp'
                    }
                }
            });

            var table = new google.visualization.ChartWrapper({
                chartType: 'Table',
                containerId: 'table_div',
                options: {
                    showRowNumber: true
                }
            });

            dashboard.bind([stringFilter, numberRangeFilter], [table]);
            dashboard.draw(data);

        } else {
            console.log("empty!");
        }
    } else {
        console.log("Not an array, please pass data as an array!");
    }

};


// loading google packages for drawing charts
// google.charts.setOnLoadCallback(drawChart2);

function drawChart1(d) {
    if (Array.isArray(d)) {
        if (d.length > 1) {
            console.log(d);
            var data = google.visualization.arrayToDataTable(d);
            var options = {
                title: 'Sensor Value Reading vs. Time',
                hAxis: { title: 'Time (thousands of second)', minValue: 0, maxValue: 1000 },
                vAxis: { title: 'Sensor Value Reading', minValue: 0, maxValue: 1 },
                legend: 'none'
            };

            var chart = new google.visualization.ScatterChart(document.getElementById('chart_div_1'));
            chart.draw(data, options);
        }
    }
}

function drawChart2() {
    var data = google.visualization.arrayToDataTable([
        ['Time', 'Sensor'],
        [8, 12],
        [4, 5.5],
        [11, 14],
        [4, 5],
        [3, 3.5],
        [6.5, 7]
    ]);

    var options = {
        title: 'Sensor Value Reading vs. Timestamp',
        hAxis: { title: 'Time', minValue: 0, maxValue: 15 },
        vAxis: { title: 'Sensor Value Reading', minValue: 0, maxValue: 15 },
        legend: 'none'
    };

    var chart = new google.visualization.ScatterChart(document.getElementById('chart_div_2'));

    chart.draw(data, options);
};
