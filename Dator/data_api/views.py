import csv
import json
import StringIO

from django.core import serializers
from django.core.urlresolvers import reverse
from django.db.models import Q
from django.http import HttpResponseRedirect, HttpResponse
from django.shortcuts import render, render_to_response

# Create your views here.
from django.template.context_processors import csrf
from django.views.decorators.csrf import csrf_exempt
import operator
from requests import Response
from data_api.models import Signal, Blob, LocalComputer, Experiment, Setting


def noop_view(request):
    c={}
    c.update(csrf(request))
    return render_to_response('noop.html', c)


@csrf_exempt
def setting_data(request, setting_id):
    try:
        setting = Setting.objects.get(id=setting_id)
    except Setting.DoesNotExist as e:
        return HttpResponse({'status: failed - Setting requested does not exist.'}, status=404)

    if request.method=='POST':
        try:
            json_dict = json.loads(request.body)
            setting.value = json_dict['value']
            setting.save()
            response_dict={'status': 'succeeded'}
            return HttpResponse(json.dumps(response_dict), status=200, content_type='application/json')
        except BaseException as e:
            return HttpResponse({'status': 'failed {}'.format(e)}, status=500)
    elif request.method == 'GET':
        try:
            body = setting.value
            return HttpResponse(body, status=200, content_type="application/json")
        except BaseException as e:
            return HttpResponse({'status': 'failed {}'.format(e)}, status=500)


@csrf_exempt
def blob_data(request, blob_id):
    """
    Set or get data for a json blob.
    :param request:
    :param blob_id:
    :return:
    """
    try:
        json_blob = Blob.objects.get(id=blob_id)
    except Blob.DoesNotExist as e:
        return HttpResponse({'status: failed - Blob requested does not exist.'}, status=404)
    if request.method=='POST':
        try:
            json_blob.set_data(request.body)
            response_dict={'status': 'succeeded'}
            if json_blob.mime_type is None:
                json_blob.mime_type = 'application/json'
            return HttpResponse(json.dumps(response_dict), status=200, content_type='application/json')
        except BaseException as e:
            return HttpResponse({'status': 'failed {}'.format(e)}, status=500)
    elif request.method == 'GET':
        try:
            body = json_blob.get_data()

            if json_blob.mime_type is None:
                return HttpResponse(body, status=200, content_type="application/octet-stream")
            else:
                response = HttpResponse(body, status=200, content_type=json_blob.mime_type)
                if json_blob.mime_type == 'image/jpeg':
                    response['Content-Disposition'] = 'attachment; filename="{}.jpeg"'.format(json_blob.name)

                return response
        except BaseException as e:
            return HttpResponse({'status': 'failed {}'.format(e)}, status=500)

def signal_data(request, signal_id):
    """
    Add data points to or get a signal. Incoming/Outgoing signal in json body.  Format:
    [[<value>,<utc time in millisec since epoch>], ...]
    signal points must be in ascending order of occurence.
    """
    try:
        signal = Signal.objects.get(id=signal_id)
        print signal
    except Signal.DoesNotExist as e:
        return HttpResponse({'status': 'failed - Signal requested does not exist'}, status=404)

    if request.method=='POST':
        try:
            data = json.loads(request.body)
            signal.add_points(data)
            response_dict = {'status': 'succeeded'}
            return HttpResponse(response_dict, status=200, content_type="application/json")
        except BaseException as e:
            return HttpResponse({'status': 'failed{}'.format(e)}, status=500)

    elif request.method == 'GET':
        try:
            content_type = 'application/json'
            if 'format' in request.GET and request.GET['format'] == 'csv':
                content_type = 'text/csv'
                raw_data = StringIO.StringIO()

                data = signal.get_data()
                print data
                writer = csv.writer(raw_data)
                for dat in data:
                    writer.writerow(dat)

                body = raw_data.getvalue()
            else:
                body = json.dumps(signal.get_data())
            return HttpResponse(body, status=200, content_type=content_type)
        except BaseException as e:
            return HttpResponse({'status': 'failed {}'.format(e)}, status=500)


@csrf_exempt
def experiment_media(request, experiment_id):
    try:
        experiment = Experiment.objects.get(id=experiment_id)
    except Experiment.DoesNotExist as e:
        return HttpResponse({'status': 'failed - Experiment requested does not exist'}, status=404)

    if request.method == 'POST':
        try:
            data = json.loads(request.body)
            media_link = data['media']
            experiment.media_link = media_link
            experiment.save()
            response_dict = {'status': 'succeeded'}
            return HttpResponse(response_dict, status=200, content_type="application/json")
        except BaseException as e:
            return HttpResponse({'status': 'failed{}'.format(e)}, status=500)


def claim_local_computer(request, local_computer_id):
    """
    Add the claiming user to the local computer's access group.
    :param request: A post request with the LocalComputer token in the parameters.
    :param local_computer_id:
    :return: HTTP Response containing a JSON status message
    """
    try:
        local_computer = LocalComputer.objects.get(id=local_computer_id)
        token = request.POST['token']
        if local_computer.group.user_set.all().count() > 1:
            # each local computer has a lc user already
            return HttpResponse({'status': '403 - Computer already claimed'}, status=403)

        if token == local_computer.registration_token:
            request.user.groups.add(local_computer.group)
            return HttpResponse({'status': '200 - Computer successfully claimed'}, status=200)
        else:
            return HttpResponse({'status': '403 - Wrong token presented to claim computer'}, status=403)
    except Exception as e:
        return HttpResponse({'status': "500 - unexpected error {}".format(e)}, status=500)


def clone_experiment(request, local_computer_id, source_experiment_id):

    try:
        if request.method == 'POST':
            source_experiment = Experiment.objects.get(local_computer_id=local_computer_id, id=source_experiment_id)
            if not 'name' in request.POST:
                return HttpResponse({'status': "500 - HTTP GET not supported"}, status=500)
            else:
                experiment = source_experiment.clone(request.POST['name'])
            return HttpResponse({'id': experiment.id}, status=200, content_type="application/json")
        else:
            return HttpResponse({'status': "500 - HTTP GET not supported"}, status=500)
    except Exception as e:
        return HttpResponse({'status': "500 - unexpected error {} {}".format(e, e.message)}, status=500)

EXPERIMENT = 'experiment'
SIGNAL = 'signal'
INCLUDE_DATA = 'include_data'
def find_signals(request, local_computer_id):

    if EXPERIMENT in request.GET:
        exp_s = request.GET[EXPERIMENT].strip().split(",")
        exp_q = [Q(experiment__name__contains=token) for token in exp_s]
    if SIGNAL in request.GET:
        sig_s = request.GET[SIGNAL].strip().split(",")
        sig_q = [Q(name__contains=token) for token in sig_s]
    if EXPERIMENT in request.GET and SIGNAL in request.GET:
        response_list = Signal.objects.filter(reduce(operator.or_, exp_q), reduce(operator.or_, sig_q))
    elif EXPERIMENT in request.GET:
        response_list = Signal.objects.filter(reduce(operator.or_, exp_q))
    elif SIGNAL in request.GET:
        response_list = Signal.objects.filter(reduce(operator.or_, sig_q))
    else:
        response_list = Signal.objects.all()


    if INCLUDE_DATA in request.GET:
        #response_map = dict((s.id, s) for s in response_list.all())
        signals = json.loads(serializers.serialize('json', response_list))
        print("got signals loaded")
        for signal in signals:
            print("getting signal {}".format(str(signal['pk'])))
            signal['data'] = Signal.objects.get(id=signal['pk']).get_data()
        print "done with data"
        return HttpResponse(json.dumps(signals), status=200, content_type="application/json")
    else:
        return HttpResponse(serializers.serialize('json', response_list), status=200, content_type="application/json")
