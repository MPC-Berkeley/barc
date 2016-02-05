# Dator - Data Aggregation for Robots
Data aggregation and control program selection for groups of robots

Dator is a lightweight data aggregation platform for groups of robots.  It's built to address the following needs:

* Aggregation of data from multiple robots to a central platform.
* HTTP(S) command polling and data push from robots on private networks to a centralized platform, avoiding the need to bridge into the local robot's network to control and anlyze data.
* RESTful access of aggregated data for simple data anlaysis and command generation by remote systems.
* Management of data sets across multiple hardware and software modifications.
* **Optionally:** Easy iteration of program update, deploy and test to a group of robots.

The main role of the platform is to provide a standard way to record data and actuation events from one or more local computers (robots) for later analysis of robot performance and simulation of new control regimens.

# Setup
The application is a standard Django Server 1.8 application using Python 2.7.x.   If you have python 2.7 installed, you should be able to:

1. Clone the repository to a local directory
2. Install PIP if you haven't already.
3. **pip install -r requirements.txt**
4. **./manage.py runserver**

# API
The server offers an RESFUL web api to store and retrieve sensor signals and actuation events: 

* GET to view either a list of resource **e.g. GET /api/v1/event/** or a single resource **e.g. GET /api/v1/event/15/** 
* POST to create a resource, **e.g. POST /api/v1/event/**
* PUT to update a resource, **e.g. PUT /api/v1/event/1/** 
* DELETE to delete a resource **e.g. DELETE /api/v1/event/1/**.  
 
You can look at  /vm/data_connection.py to see examples of calling the server.

Users register a computer instance before using most commands.  Registration returns a local_computer resource with an id that is used to identfiy other API requests.

## Resource Endpoints
### Global fields
All resources include the following fields

* UUID - a uniqe identifier chosen on creation
* created_at - the UTC timestamp of creation of the resource
* updated_at - the UTC timestamp the last modification of the resource

---
### /api/v1/local_computer/
The local computer resource tracks a single registered device. It's id is available as a filter for all other resource.
#### Filter parameters
None

---
#### /api/v1/experiment
An experiment has a start and end date and is used to identify signals collected when a certain experiment or 
control regime is in effect.

#### Filter parameters
* name

#### Fields
* name 
* started_at
* ended_at
* experiment_id

---
### /api/v1/event/
An event is used to record the time of a notable event on the local_computer

#### Filter parameters
* local_computer_id
* created_at
* type
* experiment_id

#### Fields
* type = models.CharField(max_length=32)
* info = models.TextField
* experiment_id 

---
### /api/v1/signal/
A signal is a pointer to a floating point time series. A signal's data can be accessed or updated via the signal data api below.

#### Filter parameters
* local_computer_id
* experiment_id

#### Fields
* name = models.CharField(max_length=128)

---
### /api/v1/blob/
A blob is a pointer to a blob of binary data.  A blob's data can be accessed or updated via the signal data api below.

#### Filter parameters
* local_computer_id
* experiment_id
* name

#### Fields
* name = models.CharField(max_length=128)

---
### /api/v1/setting/
A string setting

#### Filter parameters
* local_computer_id
* experiment_id
* key


#### Fields
* key = models.CharField(max_length=128)
* value = models.CharField(max_length=128)

---
### /api/v1/command/
A command signals a local_computer to take an action.  The vm app uses the command resource to indicate program load and stop requests.  Commands types are arbitrary and interpreted by the local_computer receiving them.

#### Filter parameters
* local_computer_id
* is_executed - Set True if the command has been executed by the local computer.

#### Fields
* type = models.IntegerField(default=COMMAND_NOOP)
* json_command = models.CharField(max_length="512")

---
#### /api/v1/program/
A program resource tracks and optionally contains code to be loaded and run on the local_computer

#### Filter parameters
* local_computer_id

#### Fields
* code = models.TextField(null=True)
* description = models.TextField(null=True)
* name = models.CharField(max_length=128)
* sleep_time_sec = models.FloatField(default=1.0)



## Data endpoints
POSTing data to blob and signal endpoints update the data in the related objects.   

### /data_api/v1/signal/\<signal_id\>/

POST data points to the signal to **append**.  Data should be JSON data in the body of the post and the content_type of the POST should be "application/json".  The format of the data  is [[<value1>, <seconds1>], [<value2>],[<seconds2>],...] 
where value is a floating point number and seconds is floating point seconds since the epoch (Jan 1, 1970).  The data should be sorted in time-increasing order.
Data points are formatted as a JSON array of arrays: [ <data frame1>, <data frame2>, ... , <data frameN] where data frame are arrays of floats 
where the last float is presumed to be a UTC timestamp (sec since the epoch): [<value1>, <value2>, ..., <valueN>,<timestamp>].  
It is presumed that all data frames have the same number of values.  When storing a data frame where one of the values is empty, the
value should be represented as 'nan'.

GET will get **all** the data stored to a signal.

---
### /data_api/v1/blob/\<blob_id\>/
POST a blob of data to **overwrite** the data in the given blob. Data should be posted as "application/octet" data and will need to be string-encoded if it is binary data.

