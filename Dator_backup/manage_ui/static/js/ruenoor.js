var app = angular.module('Ruenoor', [ 'restangular', 'ngRoute', 'ui.ace'])
    .config(function (RestangularProvider, $routeProvider, $httpProvider) {

        // set routes
        $routeProvider.when('/editor/:id', {
            templateUrl: '/static/templates/editor.html'
        }).when('/local_computer/:id', {
            templateUrl: '/static/templates/local_computer.html'
        }).when('/', {
            templateUrl: '/static/templates/main.html'
        }).when('/signal_graph/:local_computer_id/:signal_id', {
            templateUrl: '/static/templates/signal_graph.html'
        });

        // configure restangular to work with tastypie, which returns data in an objects list, meta data in a meta object
        RestangularProvider.setBaseUrl("/api/v1");

        RestangularProvider.setResponseExtractor(function (response, operation, what, url) {
            var newResponse;

            if (operation === "getList") {
                newResponse = response.objects;
                newResponse.metadata = response.meta;
            } else {
                newResponse = response;
            }

            return newResponse;
        });

        RestangularProvider.setRequestSuffix('/?format=json');
        $httpProvider.defaults.xsrfCookieName = 'csrftoken';
        $httpProvider.defaults.xsrfHeaderName = 'X-CSRFToken';
    });

/**
 * Use this directive along with ng-click to pop up a confirmation dialog before passing through the ng-click.
 */
app.directive('ngConfirmClick', [
  function(){
      return {
          priority: -1,
          restrict: 'A',
          link: function (scope, element, attrs) {
              element.bind('click', function (e) {
                  if (attrs.confirmCondition === undefined || attrs.confirmCondition === "true") {
                      var message = attrs.ngConfirmClick;
                      if (message && !confirm(message)) {
                          e.stopImmediatePropagation();
                          e.preventDefault();
                      }
                  }
              });
          }
      }
  }
]);

// user state labels for various services.
var LOCAL_COMPUTER_STATE = "local_computer_state";

function userStateService() {
    self = this;
    self.states = {};

    self.initState = function(stateString, initObject)
    {
        if (self.states[stateString] == undefined)
            self.states[stateString] = angular.copy(initObject);
        return self.states[stateString];
    };

    self.getState = function(stateString){
        return self.states(stateString);
    }
}

app.service('UserStateService', [userStateService]);
//app.config(['$httpProvider', function ($httpProvider) {
//    $httpProvider.defaults.xsrfCookieName = 'csrftoken';
//    $httpProvider.defaults.xsrfHeaderName = 'X-CSRFToken';
//}]);

