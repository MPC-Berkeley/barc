import json
from django.core.urlresolvers import reverse
from django.http import HttpResponseRedirect, HttpResponse
from django.shortcuts import render, render_to_response

# Create your views here.
from django.template.context_processors import csrf
from django.views.decorators.csrf import csrf_exempt
from requests import Response
from data_api.models import Signal, Blob, LocalComputer


def noop_view(request):
    c={}
    c.update(csrf(request))
    return render_to_response('noop.html', c)

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
            body = json.dumps(signal.get_data())
            return HttpResponse(body, status=200, content_type="application/json")
        except BaseException as e:
            return HttpResponse({'status': 'failed {}'.format(e)}, status=500)

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