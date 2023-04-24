import json
import pymysql
import time

rds_host = 'capstone-test.cwlcubc14m5f.us-east-2.rds.amazonaws.com'
name = 'admin'
password = 'EuqmBmOUwNXcPlrHYJFM!'
db_name = 'capstone'
port = 3306

POST_RAW_PATH = "/update-database"
GET_RAW_PATH = "/get-data"

def lambda_handler(event, context):
    '''
    lambda_handler : handles the data from the HTTP requests from the micrcontroller (POST) and the website (GET)
    event : dictionary : contains all the data from the request and parameters to be used for gathering data for posting
    context :  <class 'awslambdaric.lambda_context.LambdaContext'> : gives the context parameter to the lambda function
    '''
    
    print(f"Type of context: {type(context)})") # getting te type of the context
    
    '''' below partitions the lamdba function into two main branchs, get and post request '''
    
    if event['resource'] == POST_RAW_PATH:
        try:
            # Get the connection to the database 
            conn = pymysql.connect(host='capstone-test.cwlcubc14m5f.us-east-2.rds.amazonaws.com', database='capstone',
                             user='admin', password='EuqmBmOUwNXcPlrHYJFM', port=3306)  
            body = event['body'] # get the event body tag to access the post request elements
            # the body takes the following format ( for operational cases ! )
            # print(body)
            # print(type(body))
            
            if (str(type(body)) == '''<class 'str'>'''):
                body = json.loads(body)
            else:
                print("it is not a string")
            
            cursor = conn.cursor() # create the cursor to interact with the database
            
            if ( body["sensor"] == "Magnetometer1"):
                tup = ('Magnetometer1', float(body['x']), float(body['y']), float(body['z']), float(body['a_a']), float(body['c_a']), int(time.time())) #   inserting these value
                sql = '''insert into Data(sensor, x, y, z, actual_azimuth, computed_azimuth, timestamp) values('%s', '%f', '%f', '%f', '%f', '%f', '%d')''' % tup
                cursor.execute(sql) # executing the sql in python
                cursor.connection.commit() # commit the change
                print("connection complete") # debug to print that the connection and insertion was complete
            elif ( body["sensor"] == "Accelerometer1"):
                tup = ('Accelerometer1', float(body['x']), float(body['y']), float(body['z']), float(body['a_e']), float(body['c_e']), int(time.time())) # inserting these value
                sql = '''insert into Data(sensor, x, y, z, actual_elevation, computed_elevation, timestamp) values('%s', '%f', '%f', '%f', '%f', '%f', '%d')''' % tup
                cursor.execute(sql) # executing the sql in python
                cursor.connection.commit() # commit the change
                print("connection complete") # debug to print that the connection and insertion was complete
            else:
                return {
                    'statusCode': 400, # define this as a bad request, as the data does not correspond to either of 2 sensors being used 
                    'headers': {
                    'Access-Control-Allow-Origin': '*',
                    'Access-Control-Allow-Headers': 'Content-Type,X-Amz-Date,Authorization,X-Api-Key,X-Amz-Security-Token',
                    'Access-Control-Allow-Credentials': 'true',
                    'Content-Type': 'application/json'
                    },
                    'body': json.dumps('''{"Error": "Couldn't Access Data"}''') # posting an error to the return statement
                }
                
            return {
                'statusCode': 200, # status code is good for returning the data to the server
                'headers': {
                'Access-Control-Allow-Origin': '*',
                'Access-Control-Allow-Headers': 'Content-Type,X-Amz-Date,Authorization,X-Api-Key,X-Amz-Security-Token',
                'Access-Control-Allow-Credentials': 'true',
                'Content-Type': 'application/json'
                },
                'body': json.dumps('''{"Success": "Accessed database and appended sensor data"}''') # posting an error that the "server" couldn't find requested resource
                }
            
        except:
            
            print("GOT AN ERROR")
            
            return {
            'statusCode': 404, # error 404 that the "server" couldn't find requested resource
            'headers': { 
            'Access-Control-Allow-Origin': '*',
            'Access-Control-Allow-Headers': 'Content-Type,X-Amz-Date,Authorization,X-Api-Key,X-Amz-Security-Token',
            'Access-Control-Allow-Credentials': 'true',
            'Content-Type': 'application/json'
            },
            'body': json.dumps('''{"Error": "Couldn't Access Data"}''') # posting an error that the "server" couldn't find requested resource
            }
        
    elif event['resource'] == GET_RAW_PATH:
        
        try:
            conn = pymysql.connect(host='capstone-test.cwlcubc14m5f.us-east-2.rds.amazonaws.com', database='capstone',
                             user='admin', password='EuqmBmOUwNXcPlrHYJFM', port=3306)
                             
            cursor = conn.cursor() # create the cursor to interact with the database
            sql = '''select * from Data''' # get all the data from the 
            cursor.execute(sql) # executing the sql in python
            result = cursor.fetchall() # fetching all the data at the reference
    
            class create_dict(dict): 
              
                # __init__ function 
                def __init__(self): 
                    self = dict() 
                      
                # Function to add key:value 
                def add(self, key, value): 
                    self[key] = value
            body = create_dict()        
            for row in result:
                sensor_key = row[1]
                x = row[2]
                y = row[3]
                z = row[4]
                # ae represents actual elevation and ce represents the computed elevation
                # actual elevation corresponds to the angle computed by the accelerometer
                # computed elevation corresponds to the value that the system will be updated to (web-based parametric data)
                # aa represents actual azimuth and ca represents the computed azimuth 
                # actual azikmuth corresponds to the angle computed by the magnetometer
                # computed azimuth corresponds to the value that the system will be updated to (web-based parametric data)
                if sensor_key == "Accelerometer1":
                    body.add(row[0],({"name":row[1],"x":row[2],"y":row[3],"z":row[4],"ae": row[7], "ce": row[8], "t": row[9]}))
                elif sensor_key == "Magnetometer1":
                    body.add(row[0],({"name":row[1],"x":row[2],"y":row[3],"z":row[4],"aa": row[5], "ca": row[6], "t": row[9]}))
            return {
            'statusCode': 200,
            'headers': {
            'Access-Control-Allow-Origin': '*',
            'Access-Control-Allow-Headers': 'Content-Type,X-Amz-Date,Authorization,X-Api-Key,X-Amz-Security-Token',
            'Access-Control-Allow-Credentials': 'true',
            'Content-Type': 'application/json'
            },
            'body': json.dumps(body)
            }
            
        except:
            
            return {
            'statusCode': 200,
            'headers': {
            'Access-Control-Allow-Origin': '*',
            'Access-Control-Allow-Headers': 'Content-Type,X-Amz-Date,Authorization,X-Api-Key,X-Amz-Security-Token',
            'Access-Control-Allow-Credentials': 'true',
            'Content-Type': 'application/json'
            },
            'body': json.dumps('Couldnt access database!')
            }
        
  
