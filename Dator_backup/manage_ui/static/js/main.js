var app = angular.module('Ruenoor');

function MainControl($scope, $location, Restangular){

    //
    // Programs
    //
    $scope.getPrograms = function(){
        return Restangular.all("program").getList().then(function(programs){
            $scope.programs = programs;
        }, function(error){
            alert("Error loading programs "  + error);
        });
    };

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

    //
    // Local Computers
    //
    $scope.getComputers =  function(){
        return Restangular.all("local_computer").getList().then(function(localComputers){
            $scope.localComputers = localComputers;
        }, function(error){
                return alert("Error loading local computers " + error);
            }
        );
    };

    $scope.deleteComputer = function(computerId){
        var computer = _.find($scope.localComputers, function(aComputer){
            return aComputer.id == computerId;
        });
        return Restangular.one("local_computer", computer.id).get().then(function(foundComputer){
            foundComputer.remove().then(
                function(removeStatus){
                    $scope.getComputers();
                });
        });
    };


    $scope.claimComputer = function(){
        // do nothing right now;
    };
    //
    // Init
    //
    $scope.getPrograms();
    $scope.getComputers();


}

app.controller('MainControl',['$scope', '$location', 'Restangular',  MainControl]);