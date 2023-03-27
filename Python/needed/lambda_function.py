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
            
            print(body)
            
            cursor = conn.cursor() # create the cursor to interact with the database
            
            if ( body['sensor'] == "Magnetometer1"):
                tuple = (body['sensor'], float(body['x']), float(body['y']), float(body['z']), float(body['a_a']), float(body['c_a']), float(body['t_a']), float(body['t_m']), int(time.time()))
                sql = '''insert into Data(sensor, x, y, z, actual_azimuth, computed_azimuth, temperature_ambient, temperature_target, timestamp) values('%s', '%f', '%f', '%f', '%f', '%f','%f', '%f', '%d')''' % tuple
                cursor.execute(sql) # executing the sql in python
                cursor.connection.commit() # commit the change
            elif ( body['sensor'] == "Accelerometer1"):
                tuple = (body['sensor'], float(body['x']), float(body['y']), float(body['z']), float(body['a_a']), float(body['a_a']), float(body['c_a']), float(body['t_m']), int(time.time()))
                sql = '''insert into Data(sensor, x, y, z, actual_elevation, computed_elevation, temperature_ambient, temperature_target, timestamp) values('%s', '%f', '%f', '%f', '%f', '%f','%f', '%f', '%d')''' % tuple
                ursor.execute(sql) # executing the sql in python
                cursor.connection.commit() # commit the change
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
            
            return {
            'statusCode': 404,
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
            body.add(row[0],({"sensor":row[1],"data":row[2],"timestamp":row[3]}))
        
        
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
