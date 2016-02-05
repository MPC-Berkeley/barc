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
                            title: "Signal " + signalId,
                            labels: $scope.graphSignals.labels

                        }
                    )
                });
            });
        });

    };

    function mergeSignal(signalData) {

        var sortedSignalData = signalData.sort(function(a,b){
            return a[1]-b[1];
        });
        $scope.graphSignals.data = _.map(signalData, function (datum) {
            return [new Date(datum[1] * 1000), datum[0]];
        });
    }

    $scope.loadData($routeParams.signal_id);

}

angular.module('Ruenoor').controller('SignalGraph',
    ['$scope', '$routeParams', 'Restangular', '$http', '$timeout', SignalGraph]);