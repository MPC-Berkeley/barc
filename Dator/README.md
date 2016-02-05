# Dator - Cloud Data For Robots
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
The server offers an RESTFUL web api to store and retrieve sensor signals and actuation events: 

* GET to view either a list of resource **e.g. GET /api/v1/event/** or a single resource **e.g. GET /api/v1/event/15/** 
* POST to create a resource, **e.g. POST /api/v1/event/**
* PUT to update a resource, **e.g. PUT /api/v1/event/1/** 
* DELETE to delete a resource **e.g. DELETE /api/v1/event/1/**.  
 
You can look at  /vm/data_connection.py to see examples of calling the server.  

Users register a computer instance before using most commands.  Registration returns a local_computer resource with an 
id that is used to identifiy other API requests.

* [Resources](#resources)
* [Resource Utilities](#resource_utilities)
* [Data Resources](#data_resources)

---

---

## <a name="resources"></a>Resources
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

---

---


## <a name="resource_utilities"></a>Resources Search and Clone utilities
These resource urls allow the user to find signals and clone experiments within a local_computer resource.

---

### /data_api/v1/local_computer/\<local_computer_id\>/find_signals/
GET signals objects based on matching experiments and strings.

#### URL parameters
* __experiment__ A comma-separated list of strings to match experiment names
* __signal__ A comma-separated list of string to match signal names
* __include_data__ Include data from signals if this parameter is included in the URL.

#### Example

__GET http://\<server\>/api/v1/local_computer/1/find_signals?experiment=exp1,exp2&signal=MDot,Z&include_data__

Will get all signals from local_computer with id 1 from experiments with names containing 'exp1' or 'exp2' and with 
signal names containing either 'MDot' or 'Z'
  
The search will return a json list of objects e.g.

```
[
    {
        signal_name: 'exp1',
        pk: 254,
        
        fields: {
            group: [ ],
            name: "test_signal_saw_1",
            local_computer: 2,
            created_at: "2015-09-18T16:33:09Z",
            system: null,
            updated_at: "2015-09-18T16:33:09Z",
            experiment: null,
            uuid: "1271fab9-f3ce-44d9-b3d8-5f9f5409bd40"
        },
        model: "data_api.signal",
        data:[
            [
                -11,
                1441260334.53325
            ],
            [
                -22,
                1441260336.70355
            ]
        ],
    }
]
```

---

### /api/v1/local_computer/\<local_computer_id>\/experiment/\<experiment_id\>/clone_experiment/
POST to this URL will create a new experiment with copied signals, blobs and settings from the given experiment

#### URL parameters
* __name__ The name of the experiment your are cloning

#### Example

__POST http://\<server\>/api/v1/local_computer/1/experiment/10/clone_experiment/?name="new_experiment"__

Will clone all the signals, blobs and settings from experiment with id 10 from local_computer with id 1 to a new 
experiment called "new_experiment".

The post will return status 200 if it succeeds as well as json object with the id of the newly created experiment. e.g.
 
```{'id': '122'}```

---

---

## <a name="data_resources"></a>Data Resources
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

GET will return the data stored to a blob.
