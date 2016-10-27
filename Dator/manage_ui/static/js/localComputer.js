function LocalComputer($scope, $routeParams, $interval, Restangular, $location, UserStateService) {


    var ALL_EXPERIMENTS = {'id':'0','name':'all'};

    $scope.uiState= UserStateService.initState(
        LOCAL_COMPUTER_STATE,
        {
            editComputer: false,
            showPrograms: false,
            programsCount: 0,
            showSignals: false,
            signalsCount: 0,
            showSettings: false,
            settingsCount: 0,
            showEvents: false,
            eventCount: 0,
            showBlobs: false,
            blobCount: 0,
            experiment: ALL_EXPERIMENTS
        });


    /**
     *
     * Sets experiments from the db.
     */
    $scope.getExperiments = function(){
        return Restangular.all("experiment").getList({format:'json', local_computer_id: $scope.localComputer.id}).
            then(function (data){
                $scope.experiments = data;
                $scope.setExperimentChoices();
        });
    };

    $scope.experimentChoices= [ALL_EXPERIMENTS];

    /**
     * Populate experiment choices from $scope.experiments
     */
    $scope.setExperimentChoices=function() {
        $scope.experimentChoices = [ALL_EXPERIMENTS];
        _.each($scope.experiments, function(experiment){
            console.log(experiment);

            $scope.experimentChoices.push({'id':experiment.id, 'name':experiment.name,
                                          'media_link': experiment.media_link});

        });
    };

    $scope.toggleStateClass =function(toggleVar){
        if (toggleVar)
            return "fa fa-minus-circle";
        return "fa fa-plus-circle";
    };

    $scope.toggleVar = function(toggleVar ){
        $scope.uiState[toggleVar] = !$scope.uiState[toggleVar];
    };

    $scope.saveComputer = function () {
        $scope.localComputer.save().then(function (computer) {
            alert("Saved Computer");
            $scope.getComputer();
        }, function (reason) {
            alert("Failed to Save " + reason);
        });
    };

    $scope.getComputer = function () {
        return Restangular.one("local_computer", $routeParams.id).get().then(function (localComputer) {
            $scope.localComputer = localComputer;
            $scope.getEvents();
            $scope.getSignals();
            $scope.getSettings();
            $scope.getEvents();
            $scope.getBlobs();
            $scope.getExperiments();
        }, function (reason) {
            alert("Couldn't load localComputer: " + reason);
        });
    };

    $scope.getComputerStatus = function () {
        return Restangular.one("local_computer", $routeParams.id).get().then(function (localComputer) {
            $scope.localComputer.is_running = localComputer.is_running;
        }, function (reason) {
            console.log("Couldn't load localComputer: " + reason);
        });
    };

    $scope.getPrograms = function () {
        return Restangular.all("program").getList().then(function (programs) {
            $scope.programs = programs;
            $scope.uiState.programsCount = programs.length;
        }, function (reason) {
            alert("Couldn't load programs: " + reason)
        });
    };

    /**
     * Send a COMMAND_DONE to the local computer
     */
    $scope.stopComputer = function () {
        var command = {};
        command.type = COMMAND_DONE;
        command.local_computer_id = $scope.localComputer.id;


        return Restangular.all("command").post(command, "", {}, {}).then(function () {
            alert("Sent shutdown to box");
        }, function (reason) {
            alert("Couldn't shutdown local computer: " + reason);
        });
    };

    /**
     * Send a COMMAND_LOAD_PROGRAM to the local computer
     */
    $scope.loadProgram = function (program_id) {
        var command = {};
        command.type = COMMAND_LOAD_PROGRAM;
        command.local_computer_id = $scope.localComputer.id;
        command.json_command = JSON.stringify({'program_id': program_id});


        return Restangular.all("command").post(command, "", {}, {}).then(function () {
            alert("Requested program start");
        }, function (reason) {
            alert("Couldn't start program on local computer: " + reason);
        });
    };

    /**
     * Send a COMMAND_STOP_PROGRAM to the local computer
     */
    $scope.stopProgram = function (program_id) {
        var command = {};
        command.type = COMMAND_STOP_PROGRAM;
        command.local_computer_id = $scope.localComputer.id;
        command.json_command = JSON.stringify({'program_id': program_id});


        return Restangular.all("command").post(command, "", {}, {}).then(function () {
            alert("Requested program stop");
        }, function (reason) {
            alert("Couldn't stop program on local computer: " + reason);
        });
    };


    $scope.filterExperiment = function() {

        var experiment_id = $scope.uiState.experiment.id;

        var arguments = {format:'json', local_computer_id: $scope.localComputer.id};

        if ($scope.uiState.experiment.id != 0) {
            arguments.experiment_id = $scope.uiState.experiment.id;
        }

        $scope.signals = Restangular.all("signal").getList(arguments).
            then(function (data){
                $scope.signals = data;
                $scope.uiState.signalsCount=data.length;
            });

        $scope.uiState.signalsCount = $scope.signals.length;

        $scope.settings = Restangular.all("setting").getList(arguments).
            then(function (data){
                $scope.settings = data;
                $scope.uiState.settingsCount = data.length;

                for (var i = 0; i < data.length; ++i) {

                    if (data[i].key == "video") {
                        $scope.uiState.experiment.media_link = data[i].value;
                    }
                }
            });
    }


    $scope.deleteExperiment = function() {
        if ($scope.uiState.experiment.id == 0) {
            alert("Please delete individual experiment");
            return;
        }

        console.log($scope.uiState.experiment);

        Restangular.one("experiment").get({format:'json', name: $scope.uiState.experiment.name}).
            then(function(experiment) {
                // console.log(experiment.objects[0].name);
                experiment.remove({name: experiment.objects[0].name}).then(function(removeStatus) {
                    $scope.uiState.experiment.id = 0;
                    $scope.filterExperiment();
                    $scope.experiments = $scope.getExperiments();
                });
            });
    }

    /**
     * Get a list of signals associated with the computer.
     */
    $scope.getSignals = function () {
        return Restangular.all("signal").getList({format:'json', local_computer_id: $scope.localComputer.id}).
            then(function (data){
                $scope.signals = data;
                $scope.uiState.signalsCount=data.length;
        });
    };

    /**
     * Get a list of settings associated with the computer.
     */
    $scope.getSettings = function () {
        return Restangular.all("setting").getList({format:'json', local_computer_id: $scope.localComputer.id}).
            then(function (data){
                $scope.settings = data;
                $scope.uiState.settingsCount = data.length;
        });
    };


    /**
     * Get a list of events associated with the computer.
     */
    $scope.getEvents = function () {
        return Restangular.all("event").getList({format:'json', local_computer_id: $scope.localComputer.id}).
            then(function (data){
                $scope.events = data;
                $scope.uiState.eventCount = data.length;
        });
    };
    /**
     * Get a list of blobs associated with the computer.
     */
    $scope.getBlobs = function(){
        return Restangular.all("blob").getList({format:'json', local_computer_id: $scope.localComputer.id}).
            then(function (data){
                $scope.blobs = data;
                $scope.uiState.blobCount=data.length;
        });
    };

    $scope.getComputer();
    $scope.getPrograms();


    var promise = $interval($scope.getComputerStatus, 5000);

    // Cancel interval on page changes
    $scope.$on('$destroy', function () {
        if (angular.isDefined(promise)) {
            $interval.cancel(promise);
            promise = undefined;
        }
    });

    $scope.newProgram = function(){
        return Restangular.all("program").post({name:"new program", code:""}).then(function (created){
            $scope.getPrograms();
        });
    };

    $scope.deleteProgram = function(programId){
        var program = _.find($scope.programs, function(program){
            return program.id == programId;
        });
        return Restangular.one("program", program.id).get().then(function(foundProgram){
            foundProgram.remove().then(
                function(removeStatus){
                    $scope.getPrograms();
                });
        });
    };

    $scope.displaySignal = function(signalId){
        window.open("/dator/#/signal_graph/" + $routeParams.id + "/" + signalId, '_blank');
    };

    $scope.save_media_link = function() {
        console.log($scope.uiState.experiment.id);
        console.log($('#media-link').val());

        $.ajax({
            url: '/data_api/v1/experiment_media/' + $scope.uiState.experiment.id + '/',
            type : 'POST',

            data : JSON.stringify({
                'media' : $('#media-link').val()
            }),

            success : function(json) {
                console.log(json);
            },

            // handle a non-successful response
            error : function(xhr,errmsg,err) {
                console.log(xhr.status + ": " + xhr.responseText); // provide a bit more info about the error to the console
            }
        })

        location.reload();
    }
}


angular.module('Ruenoor').controller('LocalComputer',
    ['$scope', '$routeParams', '$interval', 'Restangular',  '$location', 'UserStateService', LocalComputer]);
