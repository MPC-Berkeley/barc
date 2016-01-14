var app=angular.module('Ruenoor');

function robotControl($scope, $routeParams, Restangular) {

    $scope.aceLoaded = function (_editor) {
        $scope.editor = _editor;
        _editor.setReadOnly(false);
        Restangular.one("program", $routeParams.id).get().then(function (program) {
            $scope.program = program;
            $scope.editor.getSession().setValue($scope.program.code);
        });
    };

    $scope.aceChanged = function (e) {
        //
    };

    $scope.saveFile = function () {
        console.log($scope.editor.getSession().getValue());
        $scope.program.code = $scope.editor.getSession().getValue();
        $scope.program.save();
        alert("Program file saved");
    }

}

app.controller("RobotControl", ['$scope', '$routeParams', 'Restangular', robotControl]);
