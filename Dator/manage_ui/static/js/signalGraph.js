/**
 * Created by brucewootton on 9/20/15.
 */
function SignalGraph($scope, $routeParams, Restangular, $http, $timeout) {
    $scope.graphSignals = {
        data: [],
        labels: ['time']
    };
    $scope.graphEvents = {};

    $scope.loadData = function (signalId) {

        Restangular.one('signal', signalId).get({format: 'json'}).then(function (signal) {
            console.log("got signal id " + signalId);
            $http.get('/data_api/v1/signal/' + signalId + '/?format=json').then(function (response) {
                console.log("signalData " + signalId);
                mergeSignal(response.data);
                $scope.graphSignals.labels.push(signal.name);
                $timeout(function () {
                    console.log("timeout signal " + signalId);
                    new Dygraph(document.getElementById("signals"), $scope.graphSignals.data,
                        {
                            draw_points: true,
                            title: "Signal - " + signal.name,
                            labels: $scope.graphSignals.labels

                        }
                    )
                });
            });
        });

    };

    function mergeSignal(signalData) {

        var sortedSignalData = signalData.sort(function(a,b){
            return a[0]-b[0];
        });
        $scope.graphSignals.data = _.map(sortedSignalData, function (datum) {
            return [new Date(datum[0] * 1000), datum[1]];
        });
    }

    $scope.loadData($routeParams.signal_id);

}

angular.module('Ruenoor').controller('SignalGraph',
    ['$scope', '$routeParams', 'Restangular', '$http', '$timeout', SignalGraph]);
