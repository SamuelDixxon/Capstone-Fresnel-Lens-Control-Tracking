import { getDatabase, ref, update, onValue, get } from "https://www.gstatic.com/firebasejs/9.14.0/firebase-database.js";

export function main(firebaseApp) { // main export function called in indext.html js script

    // creating buttons and adding their listeners for changing state
    const b1 = document.getElementById('b1');
    b1.addEventListener('click', build_table);
    const b2 = document.getElementById('b2');
    b2.addEventListener('click', build_graph);
    const b3 = document.getElementById('b3');
    b3.addEventListener('click', build_control);
    const b4 = document.getElementById('b4');
    b4.addEventListener('click', build_control_txt);
    // const b5 = document.getElementById('b5');
    // b5.addEventListener('click', build);

    google.load('visualization', '1.1', { packages: ['controls'] }); // loading graphical packages
    google.charts.load('current', { 'packages': ['corechart', 'table'] }); // loading chart specific packages

    function draw_table(jsondat) { // drawing the table

        var rows = [];

        // Populating random data for testing

        for (let i = 0; i < 50; i++) {
            if (i % 2 == 0) {
                rows.push(["Magnetometer1", Math.random(), Math.random(), Math.random(), (Math.random() * 50) + 10, (Math.random() * 45) + 10, null, null, (Math.random() * 10) + 5, Date.now()]);
            } else {
                rows.push(["Accelerometer1", Math.random() * 4 + 5, Math.random() * 3 - 2, Math.random() * 10 - 8, null, null, (Math.random() * 50) + 10, (Math.random() * 45) + 10, (Math.random() * 10) + 5, Date.now()]);
            }
        }

        // if (Object.keys(jsondat).length > 0) {
        //     Object.keys(jsondat).forEach(function (key) {
        //         var jsondatnest = jsondat[key];
        //         var row = [];
        //         Object.keys(jsondatnest).forEach(function (key1) {
        //             row.push(jsondatnest[key1]);
        //         });
        //         rows.push(row);
        //     });
        // }

        var data = new google.visualization.DataTable(); // create new data table ( empty will add more )

        data.addColumn('string', 'Sensor ID'); // sensor id which is searchable by the filter div below

        data.addColumn('number', 'x <br>(g/s or micro Tesla)'); // x reading for accelerometer / magnetometer
        data.addColumn('number', 'y <br>(g/s or micro Tesla)'); // y reading for accelerometer / magnetometer
        data.addColumn('number', 'z <br>(g/s or micro Tesla)'); // z reading for accelerometer / magnetometer

        data.addColumn('number', 'Computed Azimuth Angle <br>(degree)');  // Computed Azimuth Angle
        data.addColumn('number', 'Actual Azimuth Angle <br>(degree)'); // Actual Azimuth Angle
        data.addColumn('number', 'Computed Elevation Angle <br>(degree)'); // Computed Elevation Angle
        data.addColumn('number', 'Actual Elevation Angle <br>(degree)'); // Actual Elevation angle
        data.addColumn('number', 'Temperature <br>(Celsius)'); // Temperature
        data.addColumn('number', 'Timestamp'); // Timestamp

        data.addRows(rows); // add rows to the data object

        var string_filter = document.createElement('div'); // create string filter 
        string_filter.id = 'string_filter_div'; // string filter div create the element


        var number_filter_1 = document.createElement('div'); // create number filter
        number_filter_1.id = 'number_range_filter_div1';
        var number_filter_2 = document.createElement('div'); // create number filter
        number_filter_2.id = 'number_range_filter_div2';
        var number_filter_3 = document.createElement('div'); // create number filter
        number_filter_3.id = 'number_range_filter_div3';

        var filt_wrap = document.createElement('div');
        var table_div = document.createElement('div');
        var dash = document.createElement('div');
        filt_wrap.id = 'filter_wrapper';
        table_div.id = 'table_div';
        dash.id = 'dashboard';
        dash.className = 'dash';
        dash.appendChild(string_filter);
        filt_wrap.appendChild(number_filter_1);
        filt_wrap.appendChild(number_filter_2);
        filt_wrap.appendChild(number_filter_3);
        dash.appendChild(filt_wrap);
        dash.appendChild(table_div);
        document.getElementsByTagName('body')[0].appendChild(dash);


        var dashboard = new google.visualization.Dashboard(document.querySelector('#dashboard'));

        var stringFilter = new google.visualization.ControlWrapper({
            controlType: 'StringFilter',
            containerId: 'string_filter_div',
            options: {
                filterColumnIndex: 0,
                allowHtml: true
            }
        });

        var numberRangeFilter1 = new google.visualization.ControlWrapper({
            controlType: 'NumberRangeFilter',
            containerId: 'number_range_filter_div1',
            options: {
                filterColumnIndex: 1,
                // minValue: 0,
                // maxValue: 1000,
                ui: {
                    allowHtml: true,
                    label: 'X m/s^2 or uT'
                }
            }
        });

        var numberRangeFilter2 = new google.visualization.ControlWrapper({
            controlType: 'NumberRangeFilter',
            containerId: 'number_range_filter_div2',
            options: {
                filterColumnIndex: 2,
                // minValue: 0,
                // maxValue: 1000,
                ui: {
                    allowHtml: true,
                    label: 'Y m/s^2 or uT'
                }
            }
        });

        var numberRangeFilter3 = new google.visualization.ControlWrapper({
            controlType: 'NumberRangeFilter',
            containerId: 'number_range_filter_div3',
            options: {
                filterColumnIndex: 3,
                // minValue: 0,
                // maxValue: 1000,
                ui: {
                    allowHtml: true,
                    label: 'Z m/s^2 or uT'
                }
            }
        });

        var table = new google.visualization.ChartWrapper({
            chartType: 'Table',
            containerId: 'table_div',
            options: {
                showRowNumber: false,
                allowHtml: true
            }
        });

        dashboard.bind([stringFilter, numberRangeFilter1, numberRangeFilter2, numberRangeFilter3], [table]);
        dashboard.draw(data);

    };


    function draw_graph(jsondat) {

        var rows_acc_x = [[{ label: 'number' }, { type: 'number' }]];
        var rows_acc_y = [[{ label: 'number' }, { type: 'number' }]];
        var rows_acc_z = [[{ label: 'number' }, { type: 'number' }]];

        var rows_mag_x = [[{ label: 'number' }, { type: 'number' }]];
        var rows_mag_y = [[{ label: 'number' }, { type: 'number' }]];
        var rows_mag_z = [[{ label: 'number' }, { type: 'number' }]];

        var rows_t_d = [[{ label: 'number' }, { type: 'number' }]];
        var rows_a_d = [[{ label: 'number' }, { type: 'number' }]];
        var rows_e_d = [[{ label: 'number' }, { type: 'number' }]];

        if (jsondat != null | 1) {

            // if (Object.keys(jsondat).length > 0) {
            //     Object.keys(jsondat).forEach(function (key) {

            //         if (jsondat[key]["sensor"] == "Magnetometer1") {
            //             var row = [];
            //             row = [jsondat[key]["timestamp"], jsondat[key]["x"]];
            //             rows.push(row);
            //         }
            //         if (jsondat[key]["sensor"] == "Accelerometer1") {
            //             var row = [];
            //             row = [jsondat[key]["timestamp"], jsondat[key]["x"]];
            //             rows2.push(row);
            //         }
            //     });


            for (let i = 0; i < 50; i++) {
                var t = Date.now();
                sleep(1);
                rows_acc_x.push([t, Math.random()]);
                rows_acc_y.push([t, Math.random()]);
                rows_acc_z.push([t, Math.random()]);
                rows_mag_x.push([t, Math.random()]);
                rows_mag_y.push([t, Math.random()]);
                rows_mag_z.push([t, Math.random()]);
                rows_t_d.push([t, Math.random()]);
                rows_a_d.push([t, Math.random()]);
                rows_e_d.push([t, Math.random()]);
            }


            var acc_x = google.visualization.arrayToDataTable(rows_acc_x, false);
            var acc_y = google.visualization.arrayToDataTable(rows_acc_y, false);
            var acc_z = google.visualization.arrayToDataTable(rows_acc_z, false);

            var mag_x = google.visualization.arrayToDataTable(rows_mag_x, false);
            var mag_y = google.visualization.arrayToDataTable(rows_mag_y, false);
            var mag_z = google.visualization.arrayToDataTable(rows_mag_z, false);

            var t_d = google.visualization.arrayToDataTable(rows_t_d, false);
            var a_d = google.visualization.arrayToDataTable(rows_a_d, false);
            var e_d = google.visualization.arrayToDataTable(rows_e_d, false);

            var options_acc_x = {
                title: 'Accelerometer X vs. Timestamp',
                // customizable axis choise below default is autofit
                // hAxis: { title: 'Time', minValue: 0, maxValue: 15 },
                // vAxis: { title: 'Sensor Value Reading 1', minValue: 0, maxValue: 15 },
                hAxis: { title: 'Time (timestamp)' },
                vAxis: { title: 'Accelerometer X (m/s^2)' },
                legend: 'none',
                width: 500,
                height: 300,
            };

            var options_acc_y = {
                title: 'Accelerometer Y vs. Timestamp',
                // customizable axis choise below default is autofit
                // hAxis: { title: 'Time', minValue: 0, maxValue: 15 },
                // vAxis: { title: 'Sensor Value Reading 2', minValue: 0, maxValue: 15 },
                hAxis: { title: 'Time (timestamp)' },
                vAxis: { title: 'Accelerometer Y (m/s^2)' },
                width: 500,
                height: 300
            };

            var options_acc_z = {
                title: 'Accelerometer Value Z vs. Timestamp',
                // customizable axis choise below default is autofit
                // hAxis: { title: 'Time', minValue: 0, maxValue: 15 },
                // vAxis: { title: 'Sensor Value Reading 2', minValue: 0, maxValue: 15 },
                hAxis: { title: 'Time (timestamp)' },
                vAxis: { title: 'Accelerometer Z (m/s^2)' },
                width: 500,
                height: 300
            };

            var options_mag_x = {
                title: 'Magnetometer X vs. Timestamp',
                // customizable axis choise below default is autofit
                // hAxis: { title: 'Time', minValue: 0, maxValue: 15 },
                // vAxis: { title: 'Sensor Value Reading 1', minValue: 0, maxValue: 15 },
                hAxis: { title: 'Time (timestamp)' },
                vAxis: { title: 'Magnetometer X (g)' },
                legend: 'none',
                width: 500,
                height: 300,
            };

            var options_mag_y = {
                title: 'Magnetometer Y Reading vs. Timestamp',
                // customizable axis choise below default is autofit
                // hAxis: { title: 'Time', minValue: 0, maxValue: 15 },
                // vAxis: { title: 'Sensor Value Reading 2', minValue: 0, maxValue: 15 },
                hAxis: { title: 'Time (timestamp)' },
                vAxis: { title: 'Magnetometer Y (g)' },
                width: 500,
                height: 300
            };

            var options_mag_z = {
                title: 'Magnetometer Z Reading vs. Timestamp',
                // customizable axis choise below default is autofit
                // hAxis: { title: 'Time', minValue: 0, maxValue: 15 },
                // vAxis: { title: 'Sensor Value Reading 2', minValue: 0, maxValue: 15 },
                hAxis: { title: 'Time (timestamp)' },
                vAxis: { title: 'Magnetometer Z (g)' },
                width: 500,
                height: 300
            };

            var options_t_d = {
                title: 'Temperature Reading vs. Timestamp',
                // customizable axis choise below default is autofit
                // hAxis: { title: 'Time', minValue: 0, maxValue: 15 },
                // vAxis: { title: 'Sensor Value Reading 2', minValue: 0, maxValue: 15 },
                hAxis: { title: 'Time (timestamp)' },
                vAxis: { title: 'Temperature Percent Difference (%)' },
                width: 500,
                height: 300
            };

            var options_e_d = {
                title: 'Elevation Reading % Difference vs. Timestamp',
                // customizable axis choise below default is autofit
                // hAxis: { title: 'Time', minValue: 0, maxValue: 15 },
                // vAxis: { title: 'Sensor Value Reading 2', minValue: 0, maxValue: 15 },
                hAxis: { title: 'Time (timestamp)' },
                vAxis: { title: 'Elevation Angle Difference (%)' },
                width: 500,
                height: 300
            };

            var options_a_d = {
                title: 'Azimuth Reading % Difference vs. Timestamp',
                // customizable axis choise below default is autofit
                // hAxis: { title: 'Time', minValue: 0, maxValue: 15 },
                // vAxis: { title: 'Sensor Value Reading 2', minValue: 0, maxValue: 15 },
                hAxis: { title: 'Time (timestamp)' },
                vAxis: { title: 'Azimuth Percent Difference (%)' },
                width: 500,
                height: 300
            };


            // linking the HTML below with js
            var tbl1 = document.createElement('table'); // create table element
            tbl1.id = 'tbl1'; // give table an id
            tbl1.className = 'columns'; // assign the table a class name

            // // linking the HTML below with js
            // var tbl2 = document.createElement('table'); // create table element
            // tbl2.id = 'tbl2'; // give table an id
            // tbl2.className = 'columns'; // assign the table a class name

            // // linking the HTML below with js
            // var tbl3 = document.createElement('table'); // create table element
            // tbl2.id = 'tbl3'; // give table an id
            // tbl2.className = 'columns'; // assign the table a class name
            var t1 = document.createElement('h');
            t1.id = 't1';
            t1.className = 't';
            t1.innerHTML = "Magnetometer Parametric Data";
            var row1 = document.createElement('tr'); // creating first row of table of graphs
            row1.id = 'row1'; // assigning id to row 1
            var t2 = document.createElement('h');
            t2.id = 't2';
            t2.className = 't';
            t2.innerHTML = "Magnetometer Parametric Data";
            var row2 = document.createElement('tr'); // creating first row of table of graphs
            row2.id = 'row2'; // assigning id to row 2
            t2.innerHTML = "Magnetometer Parametric Data";
            var t3 = document.createElement('h');
            t3.id = 't3';
            t3.className = 't';
            t3.innerHTML = "Percent Difference Parametric Data";
            var row3 = document.createElement('tr'); // creating first row of table of graphs
            row3.id = 'row3'; // assigning id to row 2 

            var cell1 = document.createElement('th'); // creating first cell corresponding to accelerometer1_x
            var cell2 = document.createElement('th'); // creating second cell corresponding to accelerometer2_y
            var cell3 = document.createElement('th'); // creating second cell corresponding to accelerometer2_z

            var cell4 = document.createElement('th'); // creating first cell corresponding to accelerometer1_x
            var cell5 = document.createElement('th'); // creating second cell corresponding to accelerometer2_y
            var cell6 = document.createElement('th'); // creating second cell corresponding to accelerometer2_z

            var cell7 = document.createElement('th'); // creating first cell corresponding to percent difference in temperature
            var cell8 = document.createElement('th'); // creating second cell corresponding to percent difference in azimuth
            var cell9 = document.createElement('th'); // creating second cell corresponding to percent difference in elevation

            var graph1 = document.createElement('div'); // stylistically creating the div for the first graph
            graph1.id = 'chart_div1'; // giving the first graph a html id

            var graph2 = document.createElement('div'); // stylistically creating the div for the second graph
            graph2.id = 'chart_div2'; // giving the second graph a html id

            var graph3 = document.createElement('div'); // stylistically creating the div for the third graph
            graph3.id = 'chart_div3'; // giving the second graph a html id

            var graph4 = document.createElement('div'); // stylistically creating the div for the fourth graph
            graph4.id = 'chart_div4'; // giving the first graph a html id

            var graph5 = document.createElement('div'); // stylistically creating the div for the fifth graph
            graph5.id = 'chart_div5'; // giving the second graph a html id

            var graph6 = document.createElement('div'); // stylistically creating the div for the sixth graph
            graph6.id = 'chart_div6'; // giving the second graph a html id

            var graph7 = document.createElement('div'); // stylistically creating the div for the seventh graph
            graph7.id = 'chart_div7'; // giving the second graph a html id

            var graph8 = document.createElement('div'); // stylistically creating the div for the eigth graph
            graph8.id = 'chart_div8'; // giving the second graph a html id

            var graph9 = document.createElement('div'); // stylistically creating the div for the ninth graph
            graph9.id = 'chart_div9'; // giving the second graph a html id


            cell1.appendChild(graph1); // putting graph 1 in the respective column 1
            cell2.appendChild(graph2); // putting graph 2 in the respective column 2 
            cell3.appendChild(graph3); // putting graph 3 in the respective column 3

            cell4.appendChild(graph4); // putting graph 1 in the respective column 4
            cell5.appendChild(graph5); // putting graph 2 in the respective column 5 
            cell6.appendChild(graph6); // putting graph 3 in the respective column 6

            cell7.appendChild(graph7); // putting graph 1 in the respective column 4
            cell8.appendChild(graph8); // putting graph 2 in the respective column 5 
            cell9.appendChild(graph9); // putting graph 3 in the respective column 6

            row1.appendChild(cell1); // appending cell1 to the first row
            row1.appendChild(cell2); // appending cell2 to the first row
            row1.append(cell3); // appending cell3 to the first row
            row2.appendChild(cell4); // appending cell1 to the first row
            row2.appendChild(cell5); // appending cell2 to the first row
            row2.append(cell6); // appending cell3 to the first row
            row3.appendChild(cell7); // appending cell1 to the first row
            row3.appendChild(cell8); // appending cell2 to the first row
            row3.append(cell9); // appending cell3 to the first row

            tbl1.append(t1); // append title 1 
            tbl1.appendChild(row1); // appending the row to the table
            tbl1.append(t2); // append title 1 
            tbl1.appendChild(row2); // appending the row to the table
            tbl1.append(t3); // append title 1 
            tbl1.append(row3); // appending the row to the table

            document.getElementsByTagName('body')[0].appendChild(tbl1); // appending the table to the main html page

            var chart1 = new google.visualization.ScatterChart(document.getElementById('chart_div1')); // creating chart div1 object utilizing google visualization api
            var chart2 = new google.visualization.ScatterChart(document.getElementById('chart_div2')); // creating chart div2 object utilizing google visualization api
            var chart3 = new google.visualization.ScatterChart(document.getElementById('chart_div3')); // creating chart div3 object utilizing google visualization api
            var chart4 = new google.visualization.ScatterChart(document.getElementById('chart_div4')); // creating chart div1 object utilizing google visualization api
            var chart5 = new google.visualization.ScatterChart(document.getElementById('chart_div5')); // creating chart div2 object utilizing google visualization api
            var chart6 = new google.visualization.ScatterChart(document.getElementById('chart_div6')); // creating chart div3 object utilizing google visualization api
            var chart7 = new google.visualization.ScatterChart(document.getElementById('chart_div7')); // creating chart div1 object utilizing google visualization api
            var chart8 = new google.visualization.ScatterChart(document.getElementById('chart_div8')); // creating chart div2 object utilizing google visualization api
            var chart9 = new google.visualization.ScatterChart(document.getElementById('chart_div9')); // creating chart div3 object utilizing google visualization api


            chart1.draw(acc_x, options_acc_x); // drawing the first chart
            chart2.draw(acc_y, options_acc_y); // drawing the second chart
            chart3.draw(acc_z, options_acc_z); // drawing the third chart
            chart4.draw(mag_x, options_mag_x); // drawing the fourth chart
            chart5.draw(mag_y, options_mag_y); // drawing the fifth chart
            chart6.draw(mag_z, options_mag_z); // drawing the sixth chart
            chart7.draw(t_d, options_t_d); // drawing the seventh chart
            chart8.draw(a_d, options_a_d); // drawing the eight chart
            chart9.draw(e_d, options_e_d); // drawing the ninth chart

            document.getElementById('b2').value = 'on'; // turn button on to signify the page is loaded 
            //};

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

        if (b1 == "off") {

            if (b2 == "on") {
                var t1 = document.getElementById('tbl1');
                t1.remove();
                document.getElementById('b2').value = 'off';
            }

            if (b3 == "on") {
                var box = document.getElementById('box');
                box.remove();
                document.getElementById('b3').value = 'off';
            }


            google.charts.load('current', { 'packages': ['corechart'] });
            const xhr = new XMLHttpRequest(); // create Http request to the endpoint that stores data from AWS Lambda Script and Gateway API
            xhr.open('GET', 'https://kcze3io03f.execute-api.us-east-2.amazonaws.com/default/testing123'); // puzzle
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

            draw_table(); // samuel-d

        }

        document.getElementById('b1').value = 'on';

        // } else if (b1 == "off" & b5 == "on") {

        //     google.charts.load('current', { 'packages': ['corechart'] });
        //     const xhr = new XMLHttpRequest(); // create Http request to the endpoint that stores data from AWS Lambda Script and Gateway API
        //     xhr.open('GET', 'https://kcze3io03f.execute-api.us-east-2.amazonaws.com/default/testing123');
        //     xhr.responseType = 'json';
        //     xhr.send();
        //     var rows = [];
        //     xhr.onreadystatechange = (e) => {
        //         var jsondat = xhr.response;
        //         if (jsondat != null) {
        //             draw_table(jsondat);
        //         } else {
        //             console.log("Null contents");
        //         }
        //     };
        //     document.getElementById('b1').value = 'on';

        // } else {
        //     var table = document.getElementById('dashboard');
        //     table.remove();
        //     document.getElementById('b1').value = 'off';
        // }
    };


    function build_graph() {

        var b1 = document.getElementById('b1').value;
        var b2 = document.getElementById('b2').value;
        var b3 = document.getElementById('b3').value;


        if (b2 == "off") {

            if (b1 == "on") {
                var d = document.getElementById('dashboard');
                d.remove();
                document.getElementById('b1').value = 'off'; // reset b1 button to off
            }

            if (b3 == "on") {
                var box = document.getElementById('box');
                box.remove();
                document.getElementById('b3').value = 'off'; // reset b3 button to off
            }

            var jsondat = null;
            draw_graph(jsondat);

            // const xhr = new XMLHttpRequest(); // create Http request to the endpoint that stores data from AWS Lambda Script and Gateway API
            // xhr.open('GET', 'https://kcze3io03f.execute-api.us-east-2.amazonaws.com/default/testing123');
            // xhr.responseType = 'json';
            // xhr.send();
            // xhr.onreadystatechange = (e) => {
            //     var jsondat = xhr.response;
            //     draw_graph(jsondat);
            // };

            document.getElementById('b2').value = 'on';

            // } else if (b2 == "off" & b5 == "on") {

            //     const xhr = new XMLHttpRequest(); // create Http request to the endpoint that stores data from AWS Lambda Script and Gateway API
            //     xhr.open('GET', 'https://kcze3io03f.execute-api.us-east-2.amazonaws.com/default/testing123');
            //     xhr.responseType = 'json';
            //     xhr.send();
            //     xhr.onreadystatechange = (e) => {
            //         var jsondat = xhr.response;
            //         draw_graph(jsondat);
            //     };

            //     document.getElementById('b2').value = 'on';

        } else {
            var t1 = document.getElementById('tbl1');
            t1.remove();
            document.getElementById('b2').value = 'off';
        }
    };

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


                if (b3 == 'off') {
                    if (b1 == 'on') { // buildable button off , so programmatically removing table if loaded
                        var table = document.getElementById('dashboard');
                        table.replaceChildren();
                        table.remove();
                        document.getElementById('b1').value = 'off'; // reset b1 button to off
                    }
                    if (b2 == 'on') { // buildable button off , so programmatically removing graphs if loaded
                        var t1 = document.getElementById('tbl1');
                        tbl1.remove();
                        document.getElementById('b2').value = 'off';
                    }


                    var joystick = document.createElement('div');
                    joystick.id = 'joyDiv';
                    joystick.style = 'width:500px;height:500px;margin-bottom:20px;margin:auto;';
                    document.getElementsByTagName('body')[0].appendChild(joystick);
                    var joyParam = { "title": "joystick3", "autoReturnToCenter": true };

                    var joy = new JoyStick('joyDiv', joyParam);
                    var box = document.createElement('div');
                    box.id = 'box';
                    box.appendChild(joystick);

                    var cont = document.createElement('div');
                    cont.style = 'width:500;height:500px;margin-bottom:20px;margin:auto;';
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


                    // } else if (b3 == 'off') {

                    //     var joystick = document.createElement('div');
                    //     joystick.id = 'joyDiv';
                    //     joystick.style = 'width:400px;height:400px;margin-bottom:20px;margin:auto;';
                    //     document.getElementsByTagName('body')[0].appendChild(joystick);
                    //     var joyParam = { "title": "joystick3" };
                    //     var joy = new JoyStick('joyDiv', joyParam);
                    //     var box = document.createElement('div');
                    //     box.id = 'box';
                    //     box.appendChild(joystick);

                    //     var cont = document.createElement('div');
                    //     cont.style = 'width:400px;height:400px;margin-bottom:20px;margin:auto;';
                    //     cont.id = 'container';

                    //     var joy_d = document.createElement('input');
                    //     var txt = document.createElement('txt');
                    //     txt.innerHTML = "Direction: ";
                    //     joy_d.setAttribute('type', 'text');
                    //     joy_d.id = 'joy3Direzione';

                    //     cont.appendChild(txt);
                    //     cont.appendChild(joy_d);
                    //     cont.appendChild(document.createElement('br'));

                    //     var x_curr = document.createElement('input');
                    //     var txt = document.createElement('txt');
                    //     txt.innerHTML = "Current X: ";
                    //     x_curr.setAttribute('type', 'text');
                    //     x_curr.id = 'joy3X';

                    //     cont.appendChild(txt);
                    //     cont.appendChild(x_curr);
                    //     cont.appendChild(document.createElement('br'));

                    //     var y_curr = document.createElement('input');
                    //     var txt = document.createElement('txt');
                    //     txt.innerHTML = "Current Y: ";
                    //     y_curr.setAttribute('type', 'text');
                    //     y_curr.id = 'joy3Y';

                    //     cont.appendChild(txt);
                    //     cont.appendChild(y_curr);
                    //     cont.appendChild(document.createElement('br'));

                    //     box.appendChild(cont);
                    //     document.getElementsByTagName('body')[0].appendChild(box);
                    //     document.getElementById('b3').value = 'on';

                    //     var joy3IinputPosX = document.getElementById("joy3PosizioneX");
                    //     var joy3InputPosY = document.getElementById("joy3PosizioneY");
                    //     var joy3Direzione = document.getElementById("joy3Direzione");
                    //     var joy3X = document.getElementById("joy3X");
                    //     var joy3Y = document.getElementById("joy3Y");

                    //     // All functions below are time-dependent. They are essentially call-back functions
                    //     // dependent on the second specifier which is an integer representing time in milliseconds
                    //     setInterval(function () { joy3Direzione.value = joy.GetDir(); }, 50); // update the direction of joystick
                    //     setInterval(function () { joy3X.value = joy.GetX(); joy.GetX(); }, 50); // update the absolute x-pos
                    //     setInterval(function () { joy3Y.value = joy.GetY(); }, 50); // update te absolute y-pos 
                    //     setInterval(function () { update(x_pwm, { 'x': parseInt(joy.GetX()) }); }, 100); // update the NoSQL database node for https req from esp32
                    //     setInterval(function () { update(y_pwm, { 'y': parseInt(joy.GetY()) }); }, 50);  // update the NoSQL database node for https req from esp32


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
    };

    function build_control_txt() {

        const db = getDatabase(firebaseApp); // get the reference to the firebase

        const x_pwm = ref(db, "/Flags/Motor/"); // referencing x for motor control of rotational motor
        const y_pwm = ref(db, "/Flags/Motor/"); // referencing y for motor control of linear motor

        var password = window.prompt("Enter credential for motor for motor control", "");

        var button = document.createElement('button');

        button.onclick = function update_joystick() {
            try {
                update(x_pwm, { 'x': parseInt(x.value) });
                update(y_pwm, { 'y': parseInt(y.value) });
                x.value = "";
                y.value = "";
            } catch {
                console.log("not found!"); // debugging and error caught
            }
        };

        var d = ref(db, "/Flags/" + password + "/");

        onValue(d, (snapshot) => {
            if (snapshot.val() != null) {

                var box = document.createElement('div');
                box.id = 'box';

                var x = document.createElement('input');
                x.id = 'x';
                x.textContent = 'Enter x coordinate';
                var y = document.createElement('input');
                y.id = 'y';
                y.textContent = 'Enter y coordinate';
                button.innerHTML = "Update Position";

                box.appendChild(button);
                box.appendChild(document.createElement("br"));
                var x_txt = document.createElement('div');
                x_txt.innerHTML = "X position";
                box.appendChild(x_txt);
                box.appendChild(x);
                box.appendChild(document.createElement("br"));
                var y_txt = document.createElement('div');
                y_txt.innerHTML = "Y position";
                box.appendChild(y_txt);
                box.appendChild(y);

                document.getElementsByTagName('body')[0].appendChild(box);
                document.getElementById('b4').value = 'on';

            } else {
                alert("Wrong Password!");
            }
        });
    }

    // function build() {
    //     var compose = document.getElementById('b5').value;
    //     if (compose == 'on') {
    //         document.getElementById('b5').value = 'off';
    //     } else {
    //         document.getElementById('b5').value = 'on';
    //     }
    // }

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