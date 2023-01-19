import { getDatabase, ref, update, onValue, get } from "https://www.gstatic.com/firebasejs/9.14.0/firebase-database.js";

export function main(firebaseApp) {
    // creating buttons and adding their listeners for changing
    const b1 = document.getElementById('b1');
    b1.addEventListener('click', build_table);
    const b2 = document.getElementById('b2');
    b2.addEventListener('click', build_graph);
    const b3 = document.getElementById('b3');
    b3.addEventListener('click', build_control);
    const b4 = document.getElementById('b4');
    b4.addEventListener('click', build);

    google.load('visualization', '1.1', { packages: ['controls'] });
    google.charts.load('current', { 'packages': ['corechart'] });

    function draw_table(jsondat) {
        var rows = [];
        if (Object.keys(jsondat).length > 0) {
            Object.keys(jsondat).forEach(function (key) {
                var jsondatnest = jsondat[key];
                var row = [];
                Object.keys(jsondatnest).forEach(function (key1) {
                    row.push(jsondatnest[key1]);
                });
                rows.push(row);
            });
        }

        var data = new google.visualization.DataTable();
        data.addColumn('string', 'Sensor');
        data.addColumn('number', 'Reading');
        data.addColumn('number', 'Timestamp');
        data.addRows(rows);

        var string_filter = document.createElement('div');
        string_filter.id = 'string_filter_div';
        var number_filter = document.createElement('div');
        number_filter.id = 'numnber_range_filter_div';
        var table_div = document.createElement('div');
        table_div.id = 'table_div';
        var dash = document.createElement('div')
        dash.id = 'dashboard';
        dash.className = 'dash';
        dash.appendChild(string_filter);
        dash.appendChild(number_filter);
        dash.appendChild(table_div);
        document.getElementsByTagName('body')[0].appendChild(dash);


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
    };


    function draw_graph(jsondat) {

        var rows = [[{ label: 'number' }, { type: 'number' }]];
        var rows2 = [[{ label: 'number' }, { type: 'number' }]];
        if (jsondat != null) {
            console.log("made it");

            if (Object.keys(jsondat).length > 0) {
                Object.keys(jsondat).forEach(function (key) {

                    if (jsondat[key]["sensor"] == "Magnetometer1") {
                        var row = [];
                        row = [jsondat[key]["timestamp"], jsondat[key]["data"]];
                        rows.push(row);
                    }
                    if (jsondat[key]["sensor"] == "Magnetometer2") {
                        var row = [];
                        row = [jsondat[key]["timestamp"], jsondat[key]["data"]];
                        rows2.push(row);
                    }
                });

                var data_mag1 = google.visualization.arrayToDataTable(rows, false);
                var data_mag2 = google.visualization.arrayToDataTable(rows2, false);

                var options1 = {
                    title: 'Sensor Value Reading vs. Timestamp',
                    // hAxis: { title: 'Time', minValue: 0, maxValue: 15 },
                    // vAxis: { title: 'Sensor Value Reading 1', minValue: 0, maxValue: 15 },
                    legend: 'none',
                    width: 750,
                    height: 500
                };

                var options2 = {
                    title: 'Sensor Value Reading vs. Timestamp',
                    // hAxis: { title: 'Time', minValue: 0, maxValue: 15 },
                    // vAxis: { title: 'Sensor Value Reading 2', minValue: 0, maxValue: 15 },
                    legend: 'none',
                    width: 750,
                    height: 500
                };

                var tbl = document.createElement('table');
                tbl.id = 'tbl';
                tbl.className = 'columns';
                var row = document.createElement('tr');
                var cell1 = document.createElement('td');
                var cell2 = document.createElement('td');
                var graph1 = document.createElement('div');
                graph1.id = 'chart_div1';
                var graph2 = document.createElement('div');
                graph2.id = 'chart_div2';
                cell1.appendChild(graph1);
                cell2.appendChild(graph2);
                row.appendChild(cell1); row.appendChild(cell2);
                tbl.appendChild(row);
                document.getElementsByTagName('body')[0].appendChild(tbl);

                var chart = new google.visualization.ScatterChart(document.getElementById('chart_div1'));
                var chart2 = new google.visualization.ScatterChart(document.getElementById('chart_div2'))
                chart.draw(data_mag1, options1);
                chart2.draw(data_mag2, options2);
                console.log("test");
                document.getElementById('b2').value = 'on';
            };

        }
    }

    function build_table() {

        // build_table gets the data from a http request to an api endpoint 
        // created by gateway api, a microservice offered from AWS. This
        // request triggers an AWS lambda script to write all the data from
        // the MySQL database hosted on AWS into the API endpoint 

        var b1 = document.getElementById('b1').value;
        var b2 = document.getElementById('b2').value;
        var b3 = document.getElementById('b3').value;
        var b4 = document.getElementById('b4').value;

        if (b1 == "off" & b4 == "off") {

            if (b2 == "on") {
                var table = document.getElementById('tbl');
                table.remove();
                document.getElementById('b2').value = 'off';
            }

            if (b3 == "on") {
                var box = document.getElementById('box');
                box.remove();
                document.getElementById('b3').value = 'off';
            }


            google.charts.load('current', { 'packages': ['corechart'] });
            const xhr = new XMLHttpRequest(); // create Http request to the endpoint that stores data from AWS Lambda Script and Gateway API
            xhr.open('GET', 'https://kcze3io03f.execute-api.us-east-2.amazonaws.com/default/testing123');
            xhr.responseType = 'json';
            xhr.send();
            xhr.onreadystatechange = (e) => {
                var jsondat = xhr.response;
                if (jsondat != null) {
                    draw_table(jsondat);
                } else {
                    console.log("Null contents");
                }
            };

            document.getElementById('b1').value = 'on';

        } else if (b1 == "off" & b4 == "on") {

            google.charts.load('current', { 'packages': ['corechart'] });
            const xhr = new XMLHttpRequest(); // create Http request to the endpoint that stores data from AWS Lambda Script and Gateway API
            xhr.open('GET', 'https://kcze3io03f.execute-api.us-east-2.amazonaws.com/default/testing123');
            xhr.responseType = 'json';
            xhr.send();
            var rows = [];
            xhr.onreadystatechange = (e) => {
                var jsondat = xhr.response;
                if (jsondat != null) {
                    draw_table(jsondat);
                } else {
                    console.log("Null contents");
                }
            };
            document.getElementById('b1').value = 'on';

        } else {
            var table = document.getElementById('dashboard');
            table.remove();
            document.getElementById('b1').value = 'off';
        }
    };


    function build_graph() {

        var b1 = document.getElementById('b1').value;
        var b2 = document.getElementById('b2').value;
        var b3 = document.getElementById('b3').value;
        var b4 = document.getElementById('b4').value;

        if (b2 == "off" & b4 == "off") {

            if (b1 == "on") {
                var table = document.getElementById('dashboard');
                table.remove();
                document.getElementById('b1').value = 'off'; // reset b1 button to off
            }

            if (b3 == "on") {
                var box = document.getElementById('box');
                box.remove();
                document.getElementById('b3').value = 'off'; // reset b3 button to off
            }

            const xhr = new XMLHttpRequest(); // create Http request to the endpoint that stores data from AWS Lambda Script and Gateway API
            xhr.open('GET', 'https://kcze3io03f.execute-api.us-east-2.amazonaws.com/default/testing123');
            xhr.responseType = 'json';
            xhr.send();
            xhr.onreadystatechange = (e) => {
                var jsondat = xhr.response;
                draw_graph(jsondat);
            };

            document.getElementById('b2').value = 'on';

        } else if (b2 == "off" & b4 == "on") {

            const xhr = new XMLHttpRequest(); // create Http request to the endpoint that stores data from AWS Lambda Script and Gateway API
            xhr.open('GET', 'https://kcze3io03f.execute-api.us-east-2.amazonaws.com/default/testing123');
            xhr.responseType = 'json';
            xhr.send();
            xhr.onreadystatechange = (e) => {
                var jsondat = xhr.response;
                draw_graph(jsondat);
            };

            document.getElementById('b2').value = 'on';

        } else {
            var table = document.getElementById('tbl');
            table.remove();
            document.getElementById('b2').value = 'off';
            document.getElementById('b2').value = 'off';
        }
    }

    function build_control() {

        const db = getDatabase(firebaseApp); // get the reference to the firebase

        const x_pwm = ref(db, "/Flags/Motor/"); // referencing x for motor control of rotational motor
        const y_pwm = ref(db, "/Flags/Motor/"); // referencing y for motor control of linear motor

        var password = window.prompt("Enter credential for motor for motor control", "");

        var d = ref(db, "/Flags/" + password + "/");

        onValue(d, (snapshot) => {

            if (snapshot.val() != null) {
                var b1 = document.getElementById('b1').value;
                var b2 = document.getElementById('b2').value;
                var b3 = document.getElementById('b3').value;
                var b4 = document.getElementById('b4').value;


                if (b3 == 'off' & b4 == 'off') {
                    if (b1 == 'on') { // buildable button off , so programmatically removing table if loaded
                        var table = document.getElementById('dashboard');
                        table.replaceChildren();
                        table.remove();
                        document.getElementById('b1').value = 'off'; // reset b1 button to off
                    }
                    if (b2 == 'on') { // buildable button off , so programmatically removing graphs if loaded
                        var table = document.getElementById('tbl');
                        tbl.replaceChildren();
                        tbl.remove();
                        document.getElementById('b2').value = 'off';
                    }


                    var joystick = document.createElement('div');
                    joystick.id = 'joyDiv';
                    joystick.style = 'width:400px;height:400px;margin-bottom:20px;margin:auto;';
                    document.getElementsByTagName('body')[0].appendChild(joystick);
                    var joyParam = { "title": "joystick3" };
                    var joy = new JoyStick('joyDiv', joyParam);
                    var box = document.createElement('div');
                    box.id = 'box';
                    box.appendChild(joystick);

                    var cont = document.createElement('div');
                    cont.style = 'width:400px;height:400px;margin-bottom:20px;margin:auto;';
                    cont.id = 'container';

                    var joy_d = document.createElement('input');
                    var txt = document.createElement('txt');
                    txt.innerHTML = "Direction: ";
                    joy_d.setAttribute('type', 'text');
                    joy_d.id = 'joy3Direzione';

                    cont.appendChild(txt);
                    cont.appendChild(joy_d);
                    cont.appendChild(document.createElement('br'));

                    var x_curr = document.createElement('input');
                    var txt = document.createElement('txt');
                    txt.innerHTML = "Current X: ";
                    x_curr.setAttribute('type', 'text');
                    x_curr.id = 'joy3X';

                    cont.appendChild(txt);
                    cont.appendChild(x_curr);
                    cont.appendChild(document.createElement('br'));

                    var y_curr = document.createElement('input');
                    var txt = document.createElement('txt');
                    txt.innerHTML = "Current Y: ";
                    y_curr.setAttribute('type', 'text');
                    y_curr.id = 'joy3Y';

                    cont.appendChild(txt);
                    cont.appendChild(y_curr);
                    cont.appendChild(document.createElement('br'));

                    box.appendChild(cont);
                    document.getElementsByTagName('body')[0].appendChild(box);
                    document.getElementById('b3').value = 'on';

                    var joy3IinputPosX = document.getElementById("joy3PosizioneX");
                    var joy3InputPosY = document.getElementById("joy3PosizioneY");
                    var joy3Direzione = document.getElementById("joy3Direzione");
                    var joy3X = document.getElementById("joy3X");
                    var joy3Y = document.getElementById("joy3Y");

                    // All functions below are time-dependent. They are essentially call-back functions
                    // dependent on the second specifier which is an integer representing time in milliseconds
                    setInterval(function () { joy3Direzione.value = joy.GetDir(); }, 50); // update the direction of joystick
                    setInterval(function () { joy3X.value = joy.GetX(); joy.GetX(); }, 50); // update the absolute x-pos
                    setInterval(function () { joy3Y.value = joy.GetY(); }, 50); // update te absolute y-pos 
                    setInterval(function () { update(x_pwm, { 'x': parseInt(joy.GetX()) }); }, 100); // update the NoSQL database node for https req from esp32
                    setInterval(function () { update(y_pwm, { 'y': parseInt(joy.GetY()) }); }, 50);  // update the NoSQL database node for https req from esp32


                } else if (b3 == 'off' & b4 == 'on') {

                    var joystick = document.createElement('div');
                    joystick.id = 'joyDiv';
                    joystick.style = 'width:400px;height:400px;margin-bottom:20px;margin:auto;';
                    document.getElementsByTagName('body')[0].appendChild(joystick);
                    var joyParam = { "title": "joystick3" };
                    var joy = new JoyStick('joyDiv', joyParam);
                    var box = document.createElement('div');
                    box.id = 'box';
                    box.appendChild(joystick);

                    var cont = document.createElement('div');
                    cont.style = 'width:400px;height:400px;margin-bottom:20px;margin:auto;';
                    cont.id = 'container';

                    var joy_d = document.createElement('input');
                    var txt = document.createElement('txt');
                    txt.innerHTML = "Direction: ";
                    joy_d.setAttribute('type', 'text');
                    joy_d.id = 'joy3Direzione';

                    cont.appendChild(txt);
                    cont.appendChild(joy_d);
                    cont.appendChild(document.createElement('br'));

                    var x_curr = document.createElement('input');
                    var txt = document.createElement('txt');
                    txt.innerHTML = "Current X: ";
                    x_curr.setAttribute('type', 'text');
                    x_curr.id = 'joy3X';

                    cont.appendChild(txt);
                    cont.appendChild(x_curr);
                    cont.appendChild(document.createElement('br'));

                    var y_curr = document.createElement('input');
                    var txt = document.createElement('txt');
                    txt.innerHTML = "Current Y: ";
                    y_curr.setAttribute('type', 'text');
                    y_curr.id = 'joy3Y';

                    cont.appendChild(txt);
                    cont.appendChild(y_curr);
                    cont.appendChild(document.createElement('br'));

                    box.appendChild(cont);
                    document.getElementsByTagName('body')[0].appendChild(box);
                    document.getElementById('b3').value = 'on';

                    var joy3IinputPosX = document.getElementById("joy3PosizioneX");
                    var joy3InputPosY = document.getElementById("joy3PosizioneY");
                    var joy3Direzione = document.getElementById("joy3Direzione");
                    var joy3X = document.getElementById("joy3X");
                    var joy3Y = document.getElementById("joy3Y");

                    // All functions below are time-dependent. They are essentially call-back functions
                    // dependent on the second specifier which is an integer representing time in milliseconds
                    setInterval(function () { joy3Direzione.value = joy.GetDir(); }, 50); // update the direction of joystick
                    setInterval(function () { joy3X.value = joy.GetX(); joy.GetX(); }, 50); // update the absolute x-pos
                    setInterval(function () { joy3Y.value = joy.GetY(); }, 50); // update te absolute y-pos 
                    setInterval(function () { update(x_pwm, { 'x': parseInt(joy.GetX()) }); }, 100); // update the NoSQL database node for https req from esp32
                    setInterval(function () { update(y_pwm, { 'y': parseInt(joy.GetY()) }); }, 50);  // update the NoSQL database node for https req from esp32

                } else {
                    var joystick = document.getElementById('box');
                    joystick.remove();
                    document.getElementById('b3').value = 'off';
                }

            } else {
                alert("Entered wrong password, please try again!");
            }
        });
        // note the references may need to be scaled either post https request on esp32 program side
        // or pre https request in this javascript, as the motors have different configurations 
        // and their own respective delays
    }

    function build() {
        var compose = document.getElementById('b4').value;
        if (compose == 'on') {
            document.getElementById('b4').value = 'off';
        } else {
            document.getElementById('b4').value = 'on';
        }
    }

}

export function sleep(milliseconds) {
    // sleep method in milliseconds to include delay in code
    // Get the current data
    const date = Date.now();
    let currentDate = null;
    // do while loop to allow delay to happen until parameter
    // milliseconds is achieved
    do {
        currentDate = Date.now();
    } while (currentDate - date < milliseconds);
};